import cv2 as cv
import queue, threading, time
import logging

root = logging.getLogger()
root.setLevel(logging.INFO)

class GratbotVideoConnectionUV4L:
    def __init__(self, name):
        self.cap = cv.VideoCapture(name)
        ret, frame = self.cap.read()
        self.frame=frame
        self._lock=threading.Lock()
        self.end_called=False
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

  # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while not self.end_called:
            ret, frame = self.cap.read()
            if not ret:
                raise Exception("failed to read from video")
                break
            self._lock.acquire()
            self.frame=frame
            self._lock.release()
           
    def read(self):
        self._lock.acquire()
        toret=self.frame
        self._lock.release()
        return toret

    def stop(self):
        self.end_called=True
        if self.thread.is_alive():
            self.thread.join()
        
    def __exit__(self, exc_type, exc_value, traceback) :
        self.cap.release()
