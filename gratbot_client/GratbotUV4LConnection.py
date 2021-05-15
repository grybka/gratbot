
import cv2 as cv
import threading
import logging
import time

root = logging.getLogger()
root.setLevel(logging.INFO)

class GratbotUV4LConnection:
    def __init__(self,stream_address):
        self.cap = cv.VideoCapture(stream_address)
        self._lock=threading.Lock()
        self.grabbed_frame=None
        self.end_called=False
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

    def get_latest_frame(self):
        #returns status, frame
        #I cannot find documentation on this
        self._lock.acquire()
        theframe=self.grabbed_frame
        self._lock.release()
        return self.cap.retrieve(theframe)


    def _reader(self):
        while not self.end_called:
            grabbed_frame=self.cap.grab()
            self._lock.acquire()
            self.grabbed_frame=grabbed_frame
            self._lock.release()
        logging.info("Video Thread Stopped")
            #self.grabbed_frame=self.cap.grab()
            #ret, frame = self.cap.read()
            #$if not ret:
            #    raise Exception("failed to read from video")
            #    break
            #self.frame=frame
            #self.frame_timestamp=time.time()

    def stop(self):
        self.end_called=True
        self.thread.join()
        self.thread=None
        logging.info("Releasing Video")
        self.cap.release()

    def __exit__(self, exc_type, exc_value, traceback):
        if self.thread is not None:
            self.stop()

if __name__ == "__main__":
    conn=GratbotUV4LConnection("http://10.0.0.4:8080/stream/video.mjpeg")
    cv.namedWindow("preview")
    cv.moveWindow("preview", 0, 0)
    start_time=time.time()
    n_frames=0
    try:
        while True:
            a=time.time()
            status,new_frame=conn.get_latest_frame()
            print("waited {}".format(a-time.time()))
            n_frames+=1
            if n_frames==10:
                print("fps {}".format(10/(time.time()-start_time)))
                n_frames=0
                start_time=time.time()
            #print(status)
            #time.sleep(0.1)
            cv.imshow("preview", new_frame)
            key = cv.waitKey(1)

    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended, exiting")
    print("done")
