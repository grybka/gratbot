import sys
sys.path.append('../hardware_interface')
import hardware
import logging
import yaml

root = logging.getLogger()
root.setLevel(logging.INFO)


if __name__ == "__main__":
    logging.info("Initiating Script")
    config_file=open("../hardware_interface/raspclaws_config.yaml","r")
    config_data=yaml.safe_load(config_file)
    config_file.close()
    robot=hardware.create_hardware(config_data["hardware"])
    while True:
        snum=int(input("Servo Number: "))
        steppos=int(input("position: "))
        hardware=config_data["hardware"]
        success=False

        for x in hardware:
            elem=hardware[x]
            if elem["type"]=="GratbotServo" and elem["servo_number"]==snum:
                sname=x
                print("moving servo {} to {}".format(sname,steppos))
                robot[x].setpos_steps(steppos)
                
                success=True



