from Gyrus import Gyrus
import face_recognition
import numpy as np
import cv2
import time


class FaceRecognizer(Gyrus):
    def __init__(self,display):
        self.display=display
        self.gray_face = face_recognition.load_image_file("gray.jpg")
        self.gray_face_encoding = face_recognition.face_encodings(self.gray_face)[0]
        self.kat_face = face_recognition.load_image_file("kat.jpg")
        self.kat_face_encoding = face_recognition.face_encodings(self.kat_face)[0]
        # Create arrays of known face encodings and their names
        self.known_face_encodings = [
            self.gray_face_encoding,
            self.kat_face_encoding
        ]
        self.known_face_names = [
            "Gray",
            "Kat"
        ]
        self.max_update_period=0.5 #in seconds, don't try to recognize faces more than this often
        self.last_frame_time=time.time()


    def read_message(self,message):
        if "camera_frame" in message:
            #limit framerate on processing
            if message["timestamp"] <self.last_frame_time+self.max_update_period:
                return []
            self.last_frame_time=message["timestamp"]
            frame=message["camera_frame"]
            rgb_small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
            #rgb_small_frame = rgb_small_frame[:, :, ::-1]
            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)



            face_names = []
            for face_encoding in face_encodings:
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                name="Unknown Face"
                if matches[best_match_index]:
                    name = self.known_face_names[best_match_index]
                face_names.append(name)
            self.update_display(frame,face_locations,face_names)
        return []

    def update_display(self,video_frame,face_locations,face_names):

            frame=video_frame.copy()
            for (top, right, bottom, left), name in zip(face_locations, face_names):

                #this is to undo downscaling
                top*=4
                bottom*=4
                left*=4
                right*=4
                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
            self.display.update_image("face_recognizer",frame)
