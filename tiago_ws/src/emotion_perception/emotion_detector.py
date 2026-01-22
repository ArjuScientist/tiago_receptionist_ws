#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import time
from deepface import DeepFace
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

"""
emotion_detector.py
--------------------
Détection des émotions à partir du visage.
Publie en continu sur /emotion/state
Optimisé pour ROS Noetic + OpenCV + DeepFace (backend léger).
"""

class EmotionDetector:
    def __init__(self):
        rospy.init_node("emotion_detector")

        rospy.loginfo("😊 EmotionDetector initialisé")

        # Bridge ROS -> OpenCV
        self.bridge = CvBridge()

        # Publisher
        self.pub = rospy.Publisher("/emotion/state", String, queue_size=5)

        # Souscrire au flux caméra (adapter si besoin)
        rospy.Subscriber("/xtion/rgb/image_color", Image, self.cb_image)

        # Timing : éviter trop de calcul
        self.last_time = 0
        self.delay = 0.3  # analyse toutes les 300 ms

        # Dernière émotion connue
        self.last_emotion = "neutral"

        rospy.loginfo("📸 En attente d'image...")


    # ----------------------------------------------------------------------
    #                       CALLBACK image
    # ----------------------------------------------------------------------

    def cb_image(self, msg):

        # Limiter fréquence pour éviter surcharge CPU
        if time.time() - self.last_time < self.delay:
            return

        self.last_time = time.time()

        # Convertir image ROS -> OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Erreur conversion image : {e}")
            return

        # Resize pour accélérer
        small = cv2.resize(frame, (320, 240))

        # Détection émotion
        try:
            result = DeepFace.analyze(
                img_path=small,
                actions=['emotion'],
                enforce_detection=False,
                detector_backend='opencv'
            )

            emotion = result["dominant_emotion"]

            # Publier seulement si changement
            if emotion != self.last_emotion:
                self.last_emotion = emotion
                rospy.loginfo(f"😊 Émotion détectée : {emotion}")
                self.pub.publish(emotion)

        except Exception as e:
            rospy.logwarn(f"⚠️ Impossible d'analyser émotion : {e}")


    # ----------------------------------------------------------------------
    #                           MAIN LOOP
    # ----------------------------------------------------------------------

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = EmotionDetector()
    node.run()

