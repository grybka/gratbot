import logging
from datetime import datetime

class GratbotBehaviorHunt:
    def __init__(self,comms,eyeballs):
        self.comms=comms
        self.eyeballs=eyeballs
        self.after_someone=False

        self.behavior_log=[]

    def act(self):
        #get info from comms and eyeballs
        detection_array=self.eyeballs.get_detection_array()
        #output decisions into comms
        wheel_yaw_neutral=370
        wheel_yaw_spread=130
        #behavior is move towards first person it sees
        detection_made=False
        wheel_speed=0
        wheel_yaw=0
        target_x=-1
        target_y=-1

        for detection in detection_array:
            if detection[0]=='person':
                detection_made=True
                   
                target_x=detection[1]
                target_y=detection[2]
                #if x is <0.5, turn left, if x>0.5 turn right
                wheel_yaw=wheel_yaw_neutral-wheel_yaw_spread+2*wheel_yaw_spread*target_x
                wheel_speed=80
                self.comms.set_intention(["wheel_turn_servo","position_steps","SET"],wheel_yaw)
                self.comms.set_intention(["wheel_motor","speed","SET"],wheel_speed)
                break
        if detection_made==True:
            if not self.after_someone:
                    logging.info("Target Acquired")
                    self.after_someone=True
        else:
            if self.after_someone:
                    logging.info("Target Lost")
                    self.after_someone=False
            self.comms.set_intention(["wheel_motor","speed","SET"],0)
            self.comms.set_intention(["wheel_turn_servo","position_steps","SET"],wheel_yaw_neutral)
        #always stop for ultrasonic sensor
        self.comms.set_intention(["ultrasonic_sensor","distance","GET"],[]) #remind it to run
        current_dist=self.comms.get_state(["ultrasonic_sensor","distance"])
        if "average_distance" in current_dist:
            if current_dist["average_distance"]<0.5:
                logging.info("Too close, stopping")
                wheel_speed=0
                self.comms.set_intention(["wheel_motor","speed","SET"],0)
        self.behavior_log.append( {"wheel_speed": wheel_speed,"wheel_yaw": wheel_yaw,"target_x": target_x,"target_y":target_y} )
        #so it doesn't run away
        self.comms.set_intention(["wheel_motor","speed","SET"],0)

    def save_state(self):
        f=open("behavior_log_{}.txt".format(datetime.now().isoformat()),"w")
        elems=["wheel_speed","wheel_yaw","target_x","target_y"]
        strout=""
        for x in elems:
            strout=strout+" "+x
        f.write(strout+"\n")
        for entry in self.behavior_log:
            strout=""
            for x in elems:
                strout=strout+" {}".format(entry[x])
            f.write(strout+"\n")
        f.close()



            
