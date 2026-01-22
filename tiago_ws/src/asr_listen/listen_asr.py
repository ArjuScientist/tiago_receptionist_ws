#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import os
import shutil
import subprocess
import numpy as np
import rospy
from std_msgs.msg import String, Bool

import sounddevice as sd
from faster_whisper import WhisperModel

try:
    from scipy.signal import resample_poly
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


class WhisperPushToTalk:
    def __init__(self):
        rospy.init_node("listen_asr")

        # Params
        self.list_devices = rospy.get_param("~list_devices", False)
        self.device_index = rospy.get_param("~device_index", None)

        self.mic_sample_rate = int(rospy.get_param("~mic_sample_rate", 44100))
        self.asr_sample_rate = int(rospy.get_param("~asr_sample_rate", 16000))

        self.model_size = rospy.get_param("~model_size", "base")
        self.compute_type = rospy.get_param("~compute_type", "int8")
        self.language = rospy.get_param("~language", "fr")

        # Record behavior
        self.record_sec = float(rospy.get_param("~record_sec", 4.0))  # durée fixe
        self.beam_size = int(rospy.get_param("~beam_size", 2))
        self.vad_filter = bool(rospy.get_param("~vad_filter", False))  # optionnel maintenant que onnxruntime est là

        # -------------------------
        # Beeps (début/fin écoute)
        # -------------------------
        self.enable_beeps = bool(rospy.get_param("~enable_beeps", True))
        self.beep_start_wav = rospy.get_param("~beep_start_wav", "")
        self.beep_stop_wav = rospy.get_param("~beep_stop_wav", "")
        self.aplay_path = shutil.which("aplay")

        rospy.loginfo("🎧 Whisper Push-to-talk init...")
        rospy.loginfo(f"   device_index={self.device_index}")
        rospy.loginfo(f"   mic_sample_rate={self.mic_sample_rate} asr_sample_rate={self.asr_sample_rate}")
        rospy.loginfo(f"   model_size={self.model_size} compute_type={self.compute_type} language={self.language}")
        rospy.loginfo(f"   record_sec={self.record_sec}s beam_size={self.beam_size} vad_filter={self.vad_filter}")
        rospy.loginfo(f"   beeps={self.enable_beeps} aplay={'OK' if self.aplay_path else 'MISSING'}")

        if self.list_devices:
            self.print_devices()
            rospy.signal_shutdown("device listing done")
            return

        # ROS I/O
        self.text_pub = rospy.Publisher("/asr/text_out", String, queue_size=10)

        # Trigger: /asr/enable True => capture + transcribe une fois
        rospy.Subscriber("/asr/enable", Bool, self.cb_trigger)

        # Whisper model
        self.model = WhisperModel(self.model_size, device="cpu", compute_type=self.compute_type)

        # Anti double-trigger
        self.busy = False

        rospy.loginfo("✅ Prêt. Pour tester: rostopic pub /asr/enable std_msgs/Bool \"data: true\" -1")

    def print_devices(self):
        rospy.loginfo("🔎 Devices audio (sounddevice):")
        for i, d in enumerate(sd.query_devices()):
            rospy.loginfo(
                f"  - [{i}] {d.get('name')} | in={d.get('max_input_channels')} out={d.get('max_output_channels')}"
                f" | default_sr={d.get('default_samplerate')}"
            )

    def _resample(self, audio_f32: np.ndarray, in_sr: int, out_sr: int) -> np.ndarray:
        if in_sr == out_sr:
            return audio_f32
        if _HAS_SCIPY:
            from math import gcd
            g = gcd(in_sr, out_sr)
            up = out_sr // g
            down = in_sr // g
            return resample_poly(audio_f32, up, down).astype(np.float32)

        # fallback interp
        n = int(len(audio_f32) * (out_sr / in_sr))
        x_old = np.linspace(0.0, 1.0, num=len(audio_f32), endpoint=False)
        x_new = np.linspace(0.0, 1.0, num=n, endpoint=False)
        return np.interp(x_new, x_old, audio_f32).astype(np.float32)

    # -------------------------
    # Beep helper
    # -------------------------
    def _play_wav(self, wav_path: str):
        if not self.enable_beeps:
            return
        if not wav_path:
            return
        if not os.path.isfile(wav_path):
            rospy.logwarn(f"🔔 Beep introuvable: {wav_path}")
            return
        if self.aplay_path is None:
            rospy.logwarn("🔔 aplay introuvable. Installe: sudo apt install -y alsa-utils")
            return

        # Lecture silencieuse (pas de spam stdout)
        subprocess.run(
            [self.aplay_path, "-q", wav_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False
        )

    def cb_trigger(self, msg):
        if not msg.data:
            return
        if self.busy:
            rospy.logwarn("⏳ ASR déjà en cours, ignore trigger.")
            return

        self.busy = True
        try:
            # Beep start (indique "parle maintenant")
            self._play_wav(self.beep_start_wav)

            rospy.loginfo(f"🎙️ Enregistrement {self.record_sec:.1f}s... (parle maintenant)")

            frames = int(self.record_sec * self.mic_sample_rate)
            audio = sd.rec(
                frames,
                samplerate=self.mic_sample_rate,
                channels=1,
                dtype="float32",
                device=self.device_index,
                blocking=True,
            ).reshape(-1)

            # Beep stop (indique "c'est fini")
            self._play_wav(self.beep_stop_wav)

            rospy.loginfo("✅ Capture terminée. Transcription...")

            audio_asr = self._resample(audio, self.mic_sample_rate, self.asr_sample_rate)

            segments, info = self.model.transcribe(
                audio_asr,
                language=self.language,
                vad_filter=self.vad_filter,
                beam_size=self.beam_size,
                condition_on_previous_text=False,
            )

            parts = []
            for seg in segments:
                t = (seg.text or "").strip()
                if t:
                    parts.append(t)

            text = " ".join(parts).strip()
            if text:
                rospy.loginfo(f"🗣️ ASR → {text}")
                self.text_pub.publish(String(text))
            else:
                rospy.logwarn("🤷 Aucun texte reconnu (audio trop faible / silence).")

        except Exception as e:
            rospy.logerr(f"❌ Erreur ASR: {e}")
        finally:
            self.busy = False

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    node = WhisperPushToTalk()
    node.spin()

