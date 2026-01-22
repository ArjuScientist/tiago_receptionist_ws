#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import time

"""
face_detector.py
----------------
Détecte les visages via OpenCV et publie :
 - /face_event : True si un visage est présent, False sinon.
Optimisé pour Tiago (CPU only, faible latence).
"""

class FaceDetector:
    def __init__(self):
        rospy.init_node("face_detector")

        rospy.loginfo("👀 FaceDetector initialisé")

        # Bridge ROS → CV
        self.bridge = CvBridge()

        # Chargement Haar Cascade (inclus par défaut dans OpenCV)
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )

        # Topic caméra (paramètre pour flexibilité)
        camera_topic = rospy.get_param("~camera_topic", "/xtion/rgb/image_color")

        rospy.loginfo(f"📷 Abonné à : {camera_topic}")

        # Subscribers & Publishers
        rospy.Subscriber(camera_topic, Image, self.cb_image)

        self.pub = rospy.Publisher("/face_event", Bool, queue_size=5)

        # Anti-flutter : éviter ON/OFF trop rapides
        self.last_state = False   # False = pas de visage
        self.last_change_time = 0
        self.stability_delay = 0.3  # 300 ms

    # ----------------------------------------------------------------------
    #                     CALLBACK IMAGE
    # ----------------------------------------------------------------------

    def cb_image(self, msg):
        # Convertir image ROS → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Erreur conversion image : {e}")
            return

        # Resize pour accélérer
        small = cv2.resize(frame, (320, 240))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        # Détection de visage
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(60, 60)
        )

        detected = len(faces) > 0

        # Anti-clignotement : état stable 300 ms
        if detected != self.last_state:
            if time.time() - self.last_change_time > self.stability_delay:
                self.last_state = detected
                self.last_change_time = time.time()

                rospy.loginfo(f"👤 Face detection: {detected}")
                self.pub.publish(detected)
        else:
            # reset timer si l'état reste stable
            self.last_change_time = time.time()

    # ----------------------------------------------------------------------
    #                           MAIN LOOP
    # ----------------------------------------------------------------------

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = FaceDetector()
    node.run()

