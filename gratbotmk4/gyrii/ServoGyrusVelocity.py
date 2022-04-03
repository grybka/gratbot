
#Servo Gyrus with constant velocity
import threading
import logging
import board
import time
import numpy as np
from Gyrus import ThreadedGyrus
from adafruit_servokit import ServoKit

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class ServoGyrusVelocity(ThreadedGyrus):
    def __init__(self,broker):
        self.servo_count=16
        self.kit = ServoKit(channels=self.servo_count)
        self.max_angle=165
        self.min_angle=60
        self.max_vel=100
        self.velocity_thread=None
        self.thread_sleep_time=0.005
        self.servo_velocities=np.zeros(self.servo_count)
        self.servo_angles=np.zeros(self.servo_count)
        for i in range(len(self.servo_count)):
            self.servo_angles[i]=self.kit.servo[i].angle
        super().__init__(broker)

    def get_keys(self):
        return ["servo_command"]

    def get_name(self):
        return "ServoGyrusVelocity"

    def start_thread_called(self):
        self.velocity_thread = threading.Thread(target=self._velocity_thread_loop)
        self.velocity_thread.daemon = True
        self.velocity_thread.start()

    def join_called(self):
        if self.velocity_thread is not None:
            return self.velocity_thread.join()
        return None

    def _velocity_thread_loop(self):
        while not self.should_quit:
            time.sleep(self.thread_sleep_time)
            self.sevo_angles=np.clip(self.servo_angles+self.servo_velocities*self.thread_sleep_time,self.min_angle,self.max_angle)
            for i in range(self.servo_count):
                if self.servo_velocities[i]!=0:
                    self.kit.servo[i].angle=self.servo_angles[i]

    def set_servo_velocity(self,servo_num,vel):
        vel=np.clip(vel,-self.max_vel,self.max_vel)
        m={"servo_number": servo_num,"velocity": vel}
        self.servo_velocities[i]=vel
        self.broker.publish({"timestamp": time.time(),"servo_response": m},["servo_response"])

#    def set_servo_angle(self,servo_num,angle):
#        m={"servo_number": servo_num,"angle": angle}
#        angle=np.clip(angle,self.min_angle,self.max_angle)
#        self.kit.servo[servo_num].angle=angle
#        self.broker.publish({"timestamp": time.time(),"servo_response": m},["servo_response"])

    def read_message(self,message):
        if "servo_command" in message:
#            logger.debug("servo command received")
            m=message["servo_command"]
            now=time.time()
            servo_num=int(m["servo_number"])
            if "vel" in m:
                vel=m["vel"]
                self.set_servo_angle(servo_num,vel)
            elif "report" in m:
                m={"servo_number": servo_num,"angle": self.kit.servo[servo_num].angle,"vel": self.servo_velocities[sevo_num]}
                self.broker.publish({"timestamp": time.time(),"servo_response": m},["servo_response"])
