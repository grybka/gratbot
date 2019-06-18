import sys
import yaml
import time
import logging
import traceback
sys.path.append('../hardware_interface')
import hardware
from hardware import GratbotMotor

root = logging.getLogger()
root.setLevel(logging.INFO)

def read_ir_sensors(robot):
#returns a number between 0 and 7
    return [robot["left_ir_sensor"].get_status(),robot["middle_ir_sensor"].get_status(),robot["right_ir_sensor"].get_status()]

def ir_sensor_to_binary(my_array):
    return my_array[0]+my_array[1]*2+my_array[2]*4


#This is where our main function begins
if __name__ == "__main__":
    logging.info("Initiating Script")
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    robot=hardware.create_hardware(config_data["hardware"])
    #make initial read of ir sensors
    last_ir_read=read_ir_sensors(robot)
    robot["wheel_turn_servo"].setpos_fraction(0)
    time.sleep(0.5)
    max_speed=50
    slow_speed=50
    try:
        while True:
            time.sleep(0.5)
            current_ir_array=read_ir_sensors(robot)
            robot["left_front_led"].set_color(current_ir_array)
            robot["right_front_led"].set_color(current_ir_array)
            current_ir=ir_sensor_to_binary(current_ir_array)
            if current_ir==0b000:
                logging.info("I'm lost, 000")
                robot["wheel_motor"].stop()
                robot["wheel_turn_servo"].setpos_fraction(0)
            if current_ir==0b101:
                logging.info("I'm lost, 101")
                robot["wheel_motor"].stop()
                robot["wheel_turn_servo"].setpos_fraction(0)
            elif current_ir==0b001:
                logging.info("Hard Right!")
                robot["wheel_turn_servo"].setpos_fraction(-1)
                robot["wheel_motor"].go(GratbotMotor.forward,slow_speed)
            elif current_ir==0b100:
                logging.info("Hard Left!")
                robot["wheel_turn_servo"].setpos_fraction(1)
                robot["wheel_motor"].go(GratbotMotor.forward,slow_speed)
            elif current_ir==0b010:
                logging.info("Straight ahead!")
                robot["wheel_turn_servo"].setpos_fraction(0)
                robot["wheel_motor"].go(GratbotMotor.forward,max_speed)
            elif current_ir==0b110:
                logging.info("Gentle Left!")
                robot["wheel_turn_servo"].setpos_fraction(0.5)
                robot["wheel_motor"].go(GratbotMotor.forward,max_speed)
            elif current_ir==0b011:
                logging.info("Gentle Right!")
                robot["wheel_turn_servo"].setpos_fraction(-0.5)
                robot["wheel_motor"].go(GratbotMotor.forward,max_speed)
            elif current_ir==0b111:
                logging.info("Straight ahead slow!")
                robot["wheel_turn_servo"].setpos_fraction(0)
                robot["wheel_motor"].go(GratbotMotor.forward,max_speed)
    except KeyboardInterrupt:
        logging.warning("Keyboard Exception Program Ended, exiting")
        robot["wheel_motor"].stop()
        robot["wheel_turn_servo"].setpos_fraction(0)

    except Exception as e:
        logging.warning("Exception: {}".format(str(e)))
        exc_type, exc_obj, exc_tb = sys.exc_info()
        traceback.print_tb(exc_tb)
        logging.warning("Type: {}".format(exc_type))
        logging.warning("File: {}".format(exc_tb.tb_frame.f_code.co_filename))
        logging.warning("Line: {}".format(exc_tb.tb_lineno))

        logging.warning("Program Ended, exiting")
        robot["wheel_motor"].stop()
        robot["wheel_turn_servo"].setpos_fraction(0)

