
from adafruit_servokit import ServoKit
import logger

root = logging.getLogger()
root.setLevel(logging.INFO)


if __name__ == "__main__":
    logging.info("Initiating Script")
    while True:
        snum=int(input("Servo Number: "))
        steppos=int(input("position: "))
        self.kit.servo[servo_num].angle=angle
