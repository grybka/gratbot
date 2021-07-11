#motor gyrus
import threading
import logging
import board
import time
from Gyrus import ThreadedGyrus
from adafruit_motorkit import MotorKit

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class MotorGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.kit=MotorKit(i2c=board.I2C())
        self.motor_lock=threading.Lock()
        self.kit._pca.frequency=1000

        self.left_run_until=0
        self.right_run_until=0
        self.left_motor=self.kit.motor1
        self.right_motor=self.kit.motor2

        self.motor_thread=None
        super().__init__(broker)

    def get_keys(self):
        return ["motor_command"]

    def get_name(self):
        return "MotorGyrus"

    def start_thread_called(self):
        self.motor_thread = threading.Thread(target=self._motor_thread_loop)
        self.motor_thread.daemon = True
        self.motor_thread.start()

    def join_called(self):
        if self.motor_thread is not None:
            return self.motor_thread.join()
        return None

    def _motor_thread_loop(self):
        #this thread stops motors when they are supposed to stop
        while not self.should_quit:
            time.sleep(0.005)
            now=time.time()
            with self.motor_lock:
                if now>=self.left_run_until:
                    self.left_motor.throttle=0
                if now>=self.right_run_until:
                    self.right_motor.throttle=0

    def scale_throttle(self,throttle,duration):
        if abs(throttle)<0.05:
            return 0,0
        if abs(throttle)>=0.4:
            return throttle,duration
        scale=abs(throttle)/0.4
        return throttle/scale,duration*scale

    def read_message(self,message):
        if "motor_command" in message:
            m=message["motor_command"]
            now=time.time()
            left_throttle,left_duration=self.scale_throttle(m["left_throttle"],m["left_duration"])
            right_throttle,right_duration=self.scale_throttle(m["right_throttle"],m["right_duration"])
            #handle low throttles with reduced duration
            self.left_run_until=now+left_duration
            self.right_run_until=now+right_duration
            with self.motor_lock:
                self.left_motor.throttle=left_throttle
                self.right_motor.throttle=right_throttle
            self.broker.publish({"timestamp": time.time(),"motor_response": m},["motor_response"])
