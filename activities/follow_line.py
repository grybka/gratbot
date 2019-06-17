import sys
import yaml
import time
sys.path.append('../hardware_interface')
import hardware

def read_ir_sensors(robot):
#returns a number between 0 and 7
    return [robot["left_ir_sensor"].get_status(),robot["middle_ir_sensor"].get_status(),robot["right_ir_sensor"].get_status()]

def ir_sensor_to_binary(my_array):
    return my_array[0]+my_array[1]*2+my_array[2]*4


#This is where our main function begins
if __name__ == "__main__":
    logger.info("Initiating Script")
    #initialize hardware
    config_file=open("../hardware_interface/hardware_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    robot.create_hardware(config_data["hardware"])
    #make initial read of ir sensors
    last_ir_read=read_ir_sensors()
    time.sleep(0.5)
    try:
        while True:
            current_ir_array=read_ir_sensors()
            robot["left_front_led"].set_color(current_ir_array)
            robot["right_front_led"].set_color(current_ir_array)
            current_ir=ir_sensor_to_binary(ir_array):
            if current_ir==0b000:
                logger.info("I'm lost, 000")
                robot["wheel_motor"].stop()
                robot["wheel_turn_servo"].setpos_fraction(0)
            if current_ir==0b101:
                logger.info("I'm lost, 101")
                robot["wheel_motor"].stop()
                robot["wheel_turn_servo"].setpos_fraction(0)
            elif current_ir==0b001:
                logger.info("Hard Right!")
                robot["wheel_turn_servo"].setpos_fraction(-1)
                robot["wheel_motor"].go(robot["wheel_motor"].forward,0.4)
            elif current_ir==0b100:
                logger.info("Hard Left!")
                robot["wheel_turn_servo"].setpos_fraction(1)
                robot["wheel_motor"].go(robot["wheel_motor"].forward,0.4)
            elif current_ir==0b010:
                logger.info("Straight ahead!")
                robot["wheel_turn_servo"].setpos_fraction(0)
                robot["wheel_motor"].go(robot["wheel_motor"].forward,0.6)
            elif current_ir==0b110:
                logger.info("Gentle Left!")
                robot["wheel_turn_servo"].setpos_fraction(0.5)
                robot["wheel_motor"].go(robot["wheel_motor"].forward,0.6)
            elif current_ir==0b011:
                logger.info("Gentle Right!")
                robot["wheel_turn_servo"].setpos_fraction(-0.5)
                robot["wheel_motor"].go(robot["wheel_motor"].forward,0.6)
            elif current_ir==0b111:
                logger.info("Straight ahead slow!")
                robot["wheel_turn_servo"].setpos_fraction(0)
                robot["wheel_motor"].go(robot["wheel_motor"].forward,0.6)

    except:
        logger.warning("Program Ended, exiting")
        robot["wheel_motor"].stop()
        robot["wheel_turn_servo"].setpos_fraction(0)

