#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import queue
import shutil
import subprocess
import threading
import tempfile

import rospy
from std_msgs.msg import String, Bool

try:
    from gtts import gTTS
    _HAS_GTTS = True
except Exception:
    _HAS_GTTS = False


class TTSSpeakNode:
    """
    Node TTS:
      - Sub:  /tts/text (String)
      - Pub:  /tts/is_speaking (Bool)
      - Opt:  /asr/enable (Bool) pour couper l'écoute pendant qu'il parle

    IMPORTANT ARCHI:
      - Ce node peut couper l'ASR pendant qu'il parle (lock de sécurité).
      - MAIS par défaut il NE réactive PAS l'ASR tout seul.
        C'est reception_manager qui décide quand écouter.
    """

    def __init__(self):
        rospy.init_node("speak_tts")

        # -------------------------
        # Params
        # -------------------------
        self.language = rospy.get_param("~language", "fr")
        self.slow = bool(rospy.get_param("~slow", False))

        # coupe ASR pendant qu'on parle ?
        self.manage_asr = bool(rospy.get_param("~manage_asr", True))

        # ⚠️ Nouveau: auto-resume (désactivé par défaut)
        self.auto_resume_asr = bool(rospy.get_param("~auto_resume_asr", False))
        self.asr_resume_delay = float(rospy.get_param("~asr_resume_delay_sec", 0.4))

        # volume mpg123 (0-100)
        self.mpg123_gain = int(rospy.get_param("~mpg123_gain", 80))

        # taille max queue (sécurité)
        self.queue_max = int(rospy.get_param("~queue_max", 30))

        # moteur audio
        self.mpg123_path = shutil.which("mpg123")
        self.aplay_path = shutil.which("aplay")

        if not _HAS_GTTS:
            rospy.logerr("❌ gTTS n'est pas importable. Installe: pip install gTTS")
        if self.mpg123_path is None:
            rospy.logwarn("⚠️ mpg123 introuvable. Installe: sudo apt install -y mpg123")
        if self.aplay_path is None:
            rospy.logwarn("⚠️ aplay introuvable. (alsa-utils) Installe: sudo apt install -y alsa-utils")

        # -------------------------
        # ROS I/O
        # -------------------------
        rospy.Subscriber("/tts/text", String, self.cb_tts_text)

        # latch=True utile: quand un node démarre, il récupère tout de suite le dernier état
        self.pub_speaking = rospy.Publisher("/tts/is_speaking", Bool, queue_size=10, latch=True)
        self.pub_asr_enable = rospy.Publisher("/asr/enable", Bool, queue_size=10)

        # -------------------------
        # Queue + worker
        # -------------------------
        self.q = queue.Queue(maxsize=self.queue_max)
        self._stop = False
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self._set_speaking(False)
        rospy.loginfo("🔊 TTS Node initialisé")
        rospy.loginfo("🎤 Synthèse vocale prête. Envoie un texte sur /tts/text pour tester.")
        rospy.loginfo(f"🔒 manage_asr={self.manage_asr} | auto_resume_asr={self.auto_resume_asr}")

    # -------------------------
    # ROS callbacks
    # -------------------------
    def cb_tts_text(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        try:
            self.q.put_nowait(text)
        except queue.Full:
            rospy.logwarn("⚠️ Queue TTS pleine -> message ignoré")

    # -------------------------
    # Worker
    # -------------------------
    def _worker_loop(self):
        while (not rospy.is_shutdown()) and (not self._stop):
            try:
                text = self.q.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                self._speak(text)
            except Exception as e:
                rospy.logerr(f"❌ Erreur TTS: {e}")

            self.q.task_done()

    # -------------------------
    # Core
    # -------------------------
    def _speak(self, text: str):
        # 1) Coupe ASR avant de parler (lock de sécurité)
        if self.manage_asr:
            self.pub_asr_enable.publish(Bool(False))

        # 2) Publie état speaking
        self._set_speaking(True)
        rospy.loginfo(f"[TTS] → {text}")

        if not _HAS_GTTS:
            rospy.logerr("❌ gTTS indisponible -> impossible de parler.")
            self._set_speaking(False)
            # ⚠️ Ne pas auto-resume ici non plus (sauf si explicitement demandé)
            if self.manage_asr and self.auto_resume_asr:
                time.sleep(self.asr_resume_delay)
                self.pub_asr_enable.publish(Bool(True))
            return

        with tempfile.TemporaryDirectory(prefix="tiago_tts_") as tmpdir:
            mp3_path = os.path.join(tmpdir, "tts.mp3")

            tts = gTTS(text=text, lang=self.language, slow=self.slow)
            tts.save(mp3_path)

            played = False

            if self.mpg123_path is not None:
                cmd = [self.mpg123_path, "-q", "-f", str(int(self.mpg123_gain * 327.67)), mp3_path]
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
                played = True

            if not played:
                rospy.logerr("❌ Aucun lecteur MP3 dispo (mpg123). Installe mpg123 pour avoir le son.")

        # 3) Fin de parole
        self._set_speaking(False)

        # 4) Optionnel: réactiver l'ASR (mais en prod: plutôt False pour laisser le manager décider)
        if self.manage_asr and self.auto_resume_asr:
            time.sleep(self.asr_resume_delay)
            self.pub_asr_enable.publish(Bool(True))

    def _set_speaking(self, val: bool):
        self.pub_speaking.publish(Bool(bool(val)))

    # -------------------------
    # Shutdown
    # -------------------------
    def shutdown(self):
        self._stop = True
        try:
            self._set_speaking(False)
        except Exception:
            pass


if __name__ == "__main__":
    node = TTSSpeakNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()

