#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
import requests
from std_msgs.msg import String
from collections import deque

"""
ia_node.py
----------
Node ROS qui communique avec un modèle local via Ollama (API /api/chat).
Robuste pour un robot :
- historique court (mémoire de contexte)
- warmup au démarrage (évite le premier timeout)
- timeouts séparés connect/read
- retry 1 fois en cas de timeout
- keep_alive pour éviter le reload du modèle
"""

class IADialogNode:
    def __init__(self):
        rospy.init_node("ia_dialog")
        rospy.loginfo("🤖 IA Dialog Node — Ollama (/api/chat)")

        # -----------------------------
        # Params
        # -----------------------------
        self.model_name = rospy.get_param("~model_name", "llama3.1:8b-instruct-q4_K_M")
        self.ollama_url = rospy.get_param("~ollama_url", "http://127.0.0.1:11434/api/chat")

        # Génération
        self.temperature = float(rospy.get_param("~temperature", 0.2))
        self.top_p = float(rospy.get_param("~top_p", 0.9))
        self.num_predict = int(rospy.get_param("~num_predict", 60))

        # Perf / contexte
        self.num_ctx = int(rospy.get_param("~num_ctx", 768))
        self.num_thread = int(rospy.get_param("~num_thread", 4))

        # Mémoire conversation (nombre de tours user/assistant)
        self.max_turns = int(rospy.get_param("~max_turns", 4))
        self.history = deque(maxlen=self.max_turns * 2)

        # HTTP timeouts
        self.http_connect_timeout = float(rospy.get_param("~http_connect_timeout", 3.0))
        self.http_timeout = float(rospy.get_param("~http_timeout", 120.0))

        # Ollama keep alive (évite reload)
        self.keep_alive = rospy.get_param("~keep_alive", "10m")  # "0" pour désactiver

        # Retry
        self.retry_on_timeout = bool(rospy.get_param("~retry_on_timeout", True))
        self.retry_delay_sec = float(rospy.get_param("~retry_delay_sec", 0.3))

        # -----------------------------
        # Contexte
        # -----------------------------
        self.user_name = "visiteur"
        self.emotion = "neutre"

        # -----------------------------
        # ROS I/O
        # -----------------------------
        rospy.Subscriber("/ia/input_text", String, self.cb_user_text)
        rospy.Subscriber("/ia/user_name", String, self.cb_user_name)
        rospy.Subscriber("/emotion/state", String, self.cb_emotion)

        # Optionnel : reset mémoire
        rospy.Subscriber("/ia/reset", String, self.cb_reset)

        self.pub = rospy.Publisher("/ia/text_out", String, queue_size=10)

        # -----------------------------
        # System prompt
        # -----------------------------
        self.system_prompt = (
            "IDENTITÉ\n"
            "Tu es TIAGO, un robot de service développé par PAL Robotics. "
            "Tu es actuellement utilisé au CESI (école d’ingénieurs), campus de Nanterre, "
            "dans le FabLab situé au 4e étage du bâtiment N2. "
            "Ton code et tes comportements ont été développés par des étudiants de la formation S3E "
            "(Systèmes Électriques et Électronique Embarquée). "
            "Tu es encore en développement.\n\n"
            "RÔLE\n"
            "Tu es un robot réceptionniste : tu accueilles les visiteurs, tu expliques où ils sont, "
            "tu présentes le FabLab et le robot, et tu proposes de guider la conversation. "
            "Tu peux répondre à des questions sur le FabLab, le campus, et ton fonctionnement général.\n\n"
            "RÈGLES IMPORTANTES (à respecter)\n"
            "- Réponds uniquement en français.\n"
            "- Réponses courtes : 1 à 2 phrases maximum.\n"
            "- Si la question est floue (ou que la phrase ressemble à une mauvaise transcription), "
            "demande une clarification en 1 phrase.\n"
            "- Si on te demande quelque chose que tu ne sais pas avec certitude, dis-le clairement "
            "et propose une alternative.\n"
            "- N’invente jamais de faits : ne change pas le lieu, ne change pas l’école, ne change pas l’étage.\n"
            "- Ne dis pas que tu peux te déplacer/faire une visite si tu n’as pas cette capacité réellement. "
            "À la place, propose une présentation orale.\n\n"
            "STYLE\n"
            "Poli, accueillant, professionnel, simple. "
            "Adapte légèrement le ton selon l’émotion : triste → rassurant, heureux → chaleureux, neutre → standard.\n\n"
            "OBJECTIF\n"
            "Ton objectif est que le visiteur se sente accueilli et comprenne : où il est, ce qu’est le FabLab, "
            "et ce que tu peux faire.\n"
        )

        rospy.loginfo(
            "💬 IA prête (model=%s | url=%s | temp=%.2f | top_p=%.2f | predict=%d | ctx=%d | threads=%d | turns=%d | timeout=%.1fs)",
            self.model_name, self.ollama_url, self.temperature, self.top_p,
            self.num_predict, self.num_ctx, self.num_thread, self.max_turns, self.http_timeout
        )

        # Warmup (évite timeout au premier utilisateur)
        self.warmup()

    # -----------------------------
    # Callbacks contexte
    # -----------------------------
    def cb_user_name(self, msg):
        name = msg.data.strip()
        if name:
            self.user_name = name

    def cb_emotion(self, msg):
        emo = msg.data.strip()
        if emo:
            self.emotion = emo

    def cb_reset(self, msg):
        # Peu importe le contenu, on reset
        self.history.clear()
        rospy.loginfo("🧹 Mémoire IA reset.")

    # -----------------------------
    # Warmup
    # -----------------------------
    def warmup(self):
        try:
            rospy.loginfo("🔥 Warmup Ollama…")
            out = self.ask_model("Réponds uniquement : prêt.")
            rospy.loginfo("✅ Warmup terminé (réponse=%s)", (out or "").strip()[:60])
        except Exception as e:
            rospy.logwarn("⚠️ Warmup échoué : %s", e)

    # -----------------------------
    # Texte utilisateur
    # -----------------------------
    def cb_user_text(self, msg):
        user_text = msg.data.strip()
        if not user_text:
            return

        rospy.loginfo("👤 Texte reçu : %s", user_text)

        reply = self.ask_model(user_text)
        reply = (reply or "").strip()

        if not reply:
            reply = "Désolé, je n'ai pas compris."

        # Mémorise le tour
        self.history.append({"role": "user", "content": user_text})
        self.history.append({"role": "assistant", "content": reply})

        rospy.loginfo("🤖 Réponse générée : %s", reply)
        self.pub.publish(reply)

    # -----------------------------
    # Messages chat
    # -----------------------------
    def build_messages(self, user_text):
        user_context = (
            "CONTEXTE UTILISATEUR\n"
            f"- Nom : {self.user_name}\n"
            f"- Émotion : {self.emotion}\n"
        )

        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "system", "content": user_context},
        ]

        # Historique court
        messages.extend(list(self.history))

        # Dernier message user
        messages.append({"role": "user", "content": user_text})

        return messages

    # -----------------------------
    # Appel Ollama /api/chat
    # -----------------------------
    def ask_model(self, user_text):
        payload = {
            "model": self.model_name,
            "stream": False,
            "messages": self.build_messages(user_text),
            "options": {
                "temperature": self.temperature,
                "top_p": self.top_p,
                "num_predict": self.num_predict,
                "num_ctx": self.num_ctx,
                "num_thread": self.num_thread,
            },
            "keep_alive": self.keep_alive,
        }

        def _do_request():
            t0 = time.time()
            r = requests.post(
                self.ollama_url,
                json=payload,
                timeout=(self.http_connect_timeout, self.http_timeout),
            )
            r.raise_for_status()
            data = r.json()
            dt = time.time() - t0

            # Debug léger (1 fois de temps en temps)
            rospy.loginfo("⏱️ Ollama répondu en %.2fs | keys=%s", dt, list(data.keys()))

            msg = data.get("message") or {}
            content = (msg.get("content") or "").strip()
            return content

        try:
            return _do_request()

        except requests.exceptions.ReadTimeout as e:
            rospy.logwarn("⏳ Timeout lecture Ollama : %s", e)
            if self.retry_on_timeout:
                rospy.logwarn("🔁 Retry dans %.1fs…", self.retry_delay_sec)
                time.sleep(self.retry_delay_sec)
                try:
                    return _do_request()
                except Exception as e2:
                    rospy.logerr("❌ Retry échoué : %s", e2)
                    return "Désolé, j'ai un petit souci technique."
            return "Désolé, j'ai un petit souci technique."

        except requests.exceptions.RequestException as e:
            rospy.logerr("❌ Erreur HTTP Ollama : %s", e)
            return "Désolé, j'ai un petit souci technique."

        except ValueError as e:
            rospy.logerr("❌ Réponse Ollama non-JSON : %s", e)
            return "Désolé, j'ai un petit souci technique."

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = IADialogNode()
    node.run()

