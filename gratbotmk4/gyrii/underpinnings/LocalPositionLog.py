from underpinnings.BayesianArray import BayesianArray
import numpy as np
from pyquaternion import Quaternion

class LocalPositionLog:
    def __init__(self):
        self.pointing=[1,0,0,0] #the Quaternion that represents my orientation compared to the map
        self.offset=BayesianArray(ndim=3)
        self.offset.covariance*=1e-6
        self.motor_to_meters=1.0
        self.motor_to_meters_unc=0.1

    def handle_motor_update(self,message):
        left_motor=message["left_motor"]
        right_motor=message["right_motor"]
        forward=left_motor+right_motor
        magnitude=forward*self.motor_to_meters
        quat=Quaternion(self.pointing)
        rot=quat.rotation_matrix
        forward_vec=np.array([0,0,1])
        mapvec=rot@forward_vec
        #mapvec[0] is x
        #mapvec[1] is y
        #note that this is the pointing of my head.  my body is on presumably flat floor
        twodpointing=np.array([mapvec[0],mapvec[1]])
        twodpointing=twodpointing/(np.linalg.norm(twodpointing)+1e-6)
        #TODO FIX THIS
        #self.offset+=twodpointing*magnitude

    def read_message(self,message):
        if "packets" in message:
            packet=message["packets"][-1]
            self.pointing=packet["rotation_vector"]
        if "left_motor" in message:
            self.handle_motor_update(message)

    def reset_offset(self):
        self.offset=BayesianArray(ndim=3)
        self.offset.covariance*=1e-6
