import numpy as np

class ServoSimulator
    def __init__(self):
        self.degrees_per_step=1
        self.no_motion_prob=0.2
        self.degree_uncertainty=2

    def apply_motion(desired_steps,old_position):
        new_position=old_position
        if np.random()>self.no_motion_prob:
            new_position+=desired_steps*self.degrees_per_step+self.degree_uncertainty*np.random.normal()
        return new_position





