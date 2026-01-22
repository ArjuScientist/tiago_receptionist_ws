#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import time
import rospy
from std_msgs.msg import Bool, String


class ReceptionManager:
    def __init__(self):
        rospy.init_node("reception_manager")

        # =========================
        # Params
        # =========================
        self.greet_cooldown = float(rospy.get_param("~greet_cooldown", 8.0))

        # Session behavior
        # - session_latch_sec: après un True visage, on considère "présent" pendant N secondes
        #   même si la caméra flappe / perd le visage.
        self.session_latch_sec = float(rospy.get_param("~session_latch_sec", 6.0))

        # - end_session_absence_sec: si > 0, on termine la session si aucun True visage
        #   depuis ce temps (mais seulement si la session est déjà engagée).
        #   Mets 0 pour ne JAMAIS couper une session à cause du visage.
        self.end_session_absence_sec = float(rospy.get_param("~end_session_absence_sec", 0.0))

        # TTS
        self.tts_cooldown_sec = float(rospy.get_param("~tts_cooldown_sec", 0.35))
        self.use_tts_done = bool(rospy.get_param("~use_tts_done", False))
        self.tts_fallback_wps = float(rospy.get_param("~tts_fallback_wps", 2.3))

        # Debug
        self.debug_state_log = bool(rospy.get_param("~debug_state_log", True))
        self.debug_state_log_period = float(rospy.get_param("~debug_state_log_period", 2.0))

        # =========================
        # State
        # =========================
        self.state = "INIT"

        # Face trigger (simple)
        self.raw_face = False
        self.last_face_true_ts = 0.0  # dernier timestamp où face_event=True a été reçu

        # Greeting / session
        self.last_greet_ts = 0.0
        self.session_active = False  # "on a engagé une interaction"

        # Context
        self.user_name = ""
        self.emotion = "neutral"

        # Locks / flags
        self.speaking_lock = False
        self.listening_lock = False
        self.asr_armed = False  # one-shot enable per state

        # TTS state from speak_tts
        self.tts_speaking = False
        self.tts_last_change_ts = time.time()
        self.pending_tts_done_deadline = 0.0

        # Last messages
        self.last_asr_text = ""
        self.last_ia_text = ""

        # =========================
        # Publishers
        # =========================
        self.pub_tts = rospy.Publisher("/tts/text", String, queue_size=10)
        self.pub_asr_enable = rospy.Publisher("/asr/enable", Bool, queue_size=10, latch=True)
        self.pub_ia_input = rospy.Publisher("/ia/input_text", String, queue_size=10)
        self.pub_ia_user = rospy.Publisher("/ia/user_name", String, queue_size=10)

        # =========================
        # Subscribers
        # =========================
        rospy.Subscriber("/face_event", Bool, self.cb_face)
        rospy.Subscriber("/emotion/state", String, self.cb_emotion)
        rospy.Subscriber("/asr/text_out", String, self.cb_asr)
        rospy.Subscriber("/ia/text_out", String, self.cb_ia)
        rospy.Subscriber("/tts/is_speaking", Bool, self.cb_tts_speaking)

        # Safe defaults
        self.set_asr(False)
        self.goto("WAIT_FOR_PERSON")

    # =========================
    # Callbacks
    # =========================
    def cb_face(self, msg: Bool):
        now = time.time()
        self.raw_face = bool(msg.data)

        # IMPORTANT: un seul True suffit => on timestamp dès qu'on voit True
        if self.raw_face:
            self.last_face_true_ts = now

            # Engage session latch (presence)
            self.session_active = True

    def cb_emotion(self, msg: String):
        self.emotion = msg.data.strip() if msg.data else "neutral"

    def cb_tts_speaking(self, msg: Bool):
        val = bool(msg.data)
        if val != self.tts_speaking:
            self.tts_speaking = val
            self.tts_last_change_ts = time.time()

        # Enforce ASR OFF while speaking
        if self.tts_speaking:
            self.set_asr(False)

        # Unlock "speaking_lock" when TTS ends
        if (not self.tts_speaking) and self.speaking_lock:
            self.speaking_lock = False

    def cb_asr(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        rospy.loginfo(f"[reception_manager] 👂 ASR reçu: '{text}' (state={self.state})")

        # Stop listening immediately (one-shot)
        self.last_asr_text = text
        self.set_asr(False)
        self.asr_armed = False

        if self.state == "WAIT_NAME":
            name = self.extract_name(text)
            if name:
                self.user_name = name
                self.pub_ia_user.publish(String(self.user_name))
                self.goto("CONFIRM_NAME")
            else:
                self.say("Désolé, je n’ai pas compris votre prénom. Pouvez-vous répéter ?")
                self.goto("WAIT_NAME")

        elif self.state == "DIALOG_WAIT_USER":
            rospy.loginfo("[reception_manager] ➜ Envoi vers IA: /ia/input_text")
            self.pub_ia_input.publish(String(text))
            self.goto("DIALOG_WAIT_IA")

        else:
            rospy.logwarn(f"[reception_manager] ASR ignoré (state={self.state}). "
                          f"Attendu: WAIT_NAME ou DIALOG_WAIT_USER.")

    def cb_ia(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        rospy.loginfo(f"[reception_manager] 🤖 IA reçu: '{text}' (state={self.state})")
        self.last_ia_text = text

        if self.state == "DIALOG_WAIT_IA":
            self.goto("SAY_IA")
        else:
            # Tolérance: si on est en dialogue mais pas exactement WAIT_IA, on dira dès que possible
            if self.state in ("DIALOG_WAIT_USER", "SAY_IA") and (not self.speaking_lock) and (not self.tts_lock_active()):
                self.goto("SAY_IA")

    # =========================
    # Helpers
    # =========================
    def goto(self, new_state: str):
        rospy.loginfo(f"[reception_manager] {self.state} -> {new_state}")
        self.state = new_state
        self.asr_armed = False

    def tts_lock_active(self) -> bool:
        """True if TTS is speaking or just ended (cooldown to avoid echo)."""
        if self.tts_speaking:
            return True
        if (time.time() - self.tts_last_change_ts) < self.tts_cooldown_sec:
            return True
        return False

    def set_asr(self, enable: bool):
        # Never allow ASR while TTS speaks/cooldown
        if enable and self.tts_lock_active():
            enable = False

        self.listening_lock = bool(enable)
        self.pub_asr_enable.publish(Bool(bool(enable)))

    def say(self, text: str):
        text = (text or "").strip()
        if not text:
            return

        self.asr_armed = False
        self.speaking_lock = True

        rospy.loginfo(f"[reception_manager] 🗣️ SAY -> /tts/text : {text}")
        self.pub_tts.publish(String(text))

        # Fallback unlock (if /tts/is_speaking is missed)
        words = max(1, len(text.split()))
        duration = words / self.tts_fallback_wps + 0.3
        self.pending_tts_done_deadline = time.time() + duration

        # Ensure ASR OFF while speaking
        self.set_asr(False)

    def extract_name(self, text: str) -> str:
        t = text.lower().strip()
        t = re.sub(r"[^\w\s'-]", " ", t, flags=re.UNICODE)
        t = re.sub(r"\s+", " ", t).strip()

        patterns = [
            r"je m'appelle\s+(.+)$",
            r"moi c'est\s+(.+)$",
            r"mon prénom c'est\s+(.+)$",
            r"prénom\s+(.+)$",
            r"c'est\s+(.+)$",
        ]

        candidate = None
        for p in patterns:
            m = re.search(p, t)
            if m:
                candidate = m.group(1).strip()
                break

        if not candidate:
            tokens = t.split()
            if len(tokens) == 1:
                candidate = tokens[0]
            else:
                return ""

        name = candidate.split()[0].strip(" '-").capitalize()

        # blacklist minimal: éviter de prendre "Bonjour" comme prénom
        blacklist = {
            "bonjour", "salut", "merci", "comment", "vous", "appelez", "appelle",
            "oui", "non", "ok", "daccord", "d'accord",
        }
        if name.lower() in blacklist:
            return ""

        if not re.match(r"^[A-Za-zÀ-ÖØ-öø-ÿ][A-Za-zÀ-ÖØ-öø-ÿ'-]{1,19}$", name):
            return ""

        return name

    def face_latched(self) -> bool:
        """Presence latched after any True for session_latch_sec."""
        if self.last_face_true_ts <= 0.0:
            return False
        return (time.time() - self.last_face_true_ts) <= self.session_latch_sec

    # =========================
    # Main loop
    # =========================
    def step(self):
        now = time.time()

        if self.debug_state_log:
            rospy.loginfo_throttle(
                self.debug_state_log_period,
                f"[reception_manager] state={self.state} session_active={self.session_active} "
                f"face_latched={self.face_latched()} raw_face={self.raw_face} "
                f"tts={self.tts_speaking} speak_lock={self.speaking_lock} asr_armed={self.asr_armed}"
            )

        # fallback unlock if TTS state missed
        if self.speaking_lock and now >= self.pending_tts_done_deadline and (not self.tts_speaking):
            self.speaking_lock = False

        # Optional end session if no face True for long time (only if enabled)
        if self.end_session_absence_sec > 0 and self.session_active:
            if (now - self.last_face_true_ts) >= self.end_session_absence_sec:
                self.goto("END_SESSION")

        # ----------------------------
        # State machine
        # ----------------------------
        if self.state == "WAIT_FOR_PERSON":
            # Engage on any face latched (i.e., any True seen recently)
            if self.face_latched() and (now - self.last_greet_ts) >= self.greet_cooldown:
                self.goto("GREET")

        elif self.state == "GREET":
            if (not self.speaking_lock) and (not self.tts_lock_active()):
                self.last_greet_ts = now
                self.say("Bonjour.")
                self.goto("ASK_NAME")

        elif self.state == "ASK_NAME":
            if (not self.speaking_lock) and (not self.tts_lock_active()):
                self.say("Comment vous appelez-vous ?")
                self.goto("WAIT_NAME")

        elif self.state == "WAIT_NAME":
            if (not self.speaking_lock) and (not self.tts_lock_active()) and (not self.asr_armed):
                self.set_asr(True)
                self.asr_armed = True

        elif self.state == "CONFIRM_NAME":
            if (not self.speaking_lock) and (not self.tts_lock_active()):
                self.say(f"Enchanté {self.user_name}.")
                self.goto("DIALOG_WAIT_USER")

        elif self.state == "DIALOG_WAIT_USER":
            if (not self.speaking_lock) and (not self.tts_lock_active()) and (not self.asr_armed):
                self.set_asr(True)
                self.asr_armed = True

        elif self.state == "DIALOG_WAIT_IA":
            pass

        elif self.state == "SAY_IA":
            if (not self.speaking_lock) and (not self.tts_lock_active()):
                self.say(self.last_ia_text)
                self.goto("DIALOG_WAIT_USER")

        elif self.state == "END_SESSION":
            if (not self.speaking_lock) and (not self.tts_lock_active()):
                self.user_name = ""
                self.last_asr_text = ""
                self.last_ia_text = ""
                self.asr_armed = False
                self.session_active = False
                self.last_face_true_ts = 0.0
                self.set_asr(False)
                self.goto("WAIT_FOR_PERSON")

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    ReceptionManager().run()

