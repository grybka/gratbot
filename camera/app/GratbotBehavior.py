import logging
logger=logging.getLogger("behavior")

class GratbotBehaviorHunt:
    def __init__(self,comms,eyeballs):
        self.comms=comms
        self.eyeballs=eyeballs
        self.after_someone=False


    def act():
        #get info from comms and eyeballs
        detection_array=eyeballs.get_detection_array()
        #output decisions into comms
        wheel_yaw_neutral=370
        wheel_yaw_spread=130
        #behavior is move towards first person it sees
        detection_made=False
        for detection in detection_array:
            if detection[0]=='person':
                detection_made=True
                   
                target_x=detection[1]
                target_y=detection[2]
                #if x is <0.5, turn left, if x>0.5 turn right
                new_wheel_yaw=wheel_yaw_neutral-wheel_yaw_spread+2*wheel_yaw_spread*target_x
                self.comms.set_intent([["wheel_turn_servo","position_steps"],"SET"],new_wheel_yaw)

                self.comms.set_intent([["wheel_motor","speed"],"SET"],int(80))
                break
        if detection_made==True:
            if not self.after_someone:
                    logger.info("Target Acquired")
                    self.after_someone=True
        else:
            if self.after_someone:
                    logger.info("Target Lost")
                    self.after_someone=False
            self.comms.set_intent([["wheel_motor","speed"],"SET"],0)
            self.comms.set_intent([["wheel_turn_servo","position_steps"],"SET"],wheel_yaw_neutral)
         #always stop for ultrasonic sensor
         self.comms.set_intent([["ultrasonic_sensor","distance"],"GET"],[]) #remind it to run
         current_dist=self.comms.get_state(["ultrasonic_sensor","distance"])
         if "average_distance" in current_dist:
            if current_dist["average_distance"]<0.5:
                logger.info("Too close, stopping")
                self.comms.set_intent([["wheel_motor","speed"],"SET"],0)

            
