from Gyrus import ThreadedGyrus
import logging
import cv2
import mediapipe as mp
import time
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
drawing_styles = mp.solutions.drawing_styles

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class HandTrackerGyrus(ThreadedGyrus):
    def __init__(self,broker):
        super().__init__(broker)
        self.hands=mp_hands.Hands(min_detection_confidence=0.5,min_tracking_confidence=0.5)

    def get_keys(self):
        return ["image"]

    def get_name(self):
        return "HandTrackerGyrus"

    def read_message(self,message):
        if "image" in message and "tracks" not in message:
            image_copy=message["image"].copy()
            results=self.hands.process(message["image"])
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image_copy, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                        drawing_styles.get_default_hand_landmark_style(),
                        drawing_styles.get_default_hand_connection_style())
            self.broker.publish({"timestamp":time.time(),"test_image": image_copy},"test_image")
