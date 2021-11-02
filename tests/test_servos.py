
from adafruit_servokit import ServoKit
import logging

root = logging.getLogger()
root.setLevel(logging.INFO)


if __name__ == "__main__":
    kit = ServoKit(channels=16)
    logging.info("Initiating Script")
    while True:
        snum=int(input("Servo Number: "))
        angle=int(input("position: "))
        kit.servo[snum].angle=angle

