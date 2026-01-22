#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool

"""
name_node.py
-------------
Node ROS chargé de demander, écouter et valider le nom de l'utilisateur.
"""

class NameNode:
    def __init__(self):
        rospy.init_node("name_node")

        rospy.loginfo("📛 NameNode initialisé")

        # Mémoire interne
        self.waiting_for_name = False

        # Souscriptions
        rospy.Subscriber("/name/start", Bool, self.cb_start)
        rospy.Subscriber("/asr/text_out", String, self.cb_asr)

        # Publications
        self.tts_pub = rospy.Publisher("/tts/text", String, queue_size=10)
        self.asr_enable_pub = rospy.Publisher("/asr/enable", Bool, queue_size=10)
        self.name_pub = rospy.Publisher("/ia/user_name", String, queue_size=10)
        self.done_pub = rospy.Publisher("/name/done", Bool, queue_size=10)

    # ------------------------------------------------------------------
    #                     START process (ask for the name)
    # ------------------------------------------------------------------

    def cb_start(self, msg):
        if not msg.data:
            return

        rospy.loginfo("📛 Début sequence demande du nom")

        self.waiting_for_name = True

        # Demander le nom via TTS
        self.tts_pub.publish("Comment vous appelez-vous ?")

        # Activer ASR
        rospy.sleep(1.0)
        self.asr_enable_pub.publish(True)


    # ------------------------------------------------------------------
    #                   ASR callback (getting the name)
    # ------------------------------------------------------------------

    def cb_asr(self, msg):

        if not self.waiting_for_name:
            return

        text = msg.data.strip()
        rospy.loginfo(f"🔍 Texte ASR reçu pour extraction du nom : {text}")

        # Extraire un nom (naïf mais efficace)
        name = self.extract_name(text)

        if name is None:
            # Redemander poliment
            self.tts_pub.publish("Je n'ai pas bien compris. Pouvez-vous répéter votre prénom ?")
            return

        # Nom trouvé
        rospy.loginfo(f"📛 Nom extrait : {name}")

        # Désactiver ASR
        self.asr_enable_pub.publish(False)

        # Publier le nom
        self.name_pub.publish(name)

        # Prévenir que c’est terminé
        self.done_pub.publish(True)

        # Fin du process
        self.waiting_for_name = False


    # ------------------------------------------------------------------
    #                 UTILITAIRE : extraction du prénom
    # ------------------------------------------------------------------

    def extract_name(self, text):

        if len(text.split()) == 0:
            return None

        # On prend le premier mot
        raw = text.split()[0]

        # Nettoyage basique
        name = raw.capitalize()

        # Filtrer les phrases invalides
        if len(name) < 2 or len(name) > 20:
            return None

        # Interdire certains mots parasites issus ASR
        parasites = ["bonjour", "salut", "oui", "non", "merci"]
        if name.lower() in parasites:
            return None

        return name


    # ------------------------------------------------------------------
    #                             RUN
    # ------------------------------------------------------------------

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = NameNode()
    node.run()

