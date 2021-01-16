
import time
from behaviors.GratbotBehavior import GratbotBehavior
from behaviors.GratbotBehavior import GratbotBehaviorStatus
from behaviors.GratbotBehavior import GratbotBehavior_RecordSensorToMemory
import numpy as np
from scipy.optimize import curve_fit

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

#So this is a behavior that is a series
#First, move around in a circle a bunch while logging the magnetic sensor
#Then, do a fit to
class CalibrateMagsensor(GratbotBehavior):

    def __init__(self):
        super().__init__()
        self.target_data_size=1000
        self.mag_data_x=[]
        self.mag_data_y=[]
        self.mag_data_z=[]


    def show_plot(self,x,y,z):
        fig,ax=plt.subplots()
        ax.plot(x,y,'*')
        ax.plot(y,z,'*')
        ax.plot(z,x,'*')
        ax.set(xlabel="Field",ylabel="Field")
        ax.grid()
        plt.show()

    def act(self,comms,sensors):
        if len(self.mag_data_x)==0:
            print("state {}".format(sensors.gratbot_state["magnetometer/b_field"]))
            print("Move robot around in 3 dimensions")
        if len(self.mag_data_x)<self.target_data_size:
            self.mag_data_x.append(sensors.gratbot_state["magnetometer/b_field"][0])
            self.mag_data_y.append(sensors.gratbot_state["magnetometer/b_field"][1])
            self.mag_data_z.append(sensors.gratbot_state["magnetometer/b_field"][2])
            if len(self.mag_data_x)%10==0:
                print("Magnetic Calibration {} percent".format(100*len(self.mag_data_x)/self.target_data_size))
                print("Current field {}".format(sensors.gratbot_state["magnetometer/b_field"]))
            return GratbotBehaviorStatus.INPROGRESS

        def fit_func(x,bx,by,bz,b0):
            return (x[0]-bx)**2+(x[1]-by)**2+(x[2]-bz)**2-b0**2

        self.show_plot(self.mag_data_x,self.mag_data_y,self.mag_data_z)
        popt,pcov=curve_fit(fit_func,[self.mag_data_x,self.mag_data_y,self.mag_data_z],np.zeros(len(self.mag_data_x)))
        its_inf=False
        for i in range(4):
            print("Parameter {} is {} +- {}".format(i,popt[i],np.sqrt(pcov[i][i])))
            if abs(np.sqrt(pcov[i][i])/abs(popt[i]) > 0.1 ):
                its_inf=True
        if its_inf:
            return GratbotBehaviorStatus.FAILED
        sensors.b_field_correction=[popt[0],popt[1],popt[2]]
        self.show_plot(self.mag_data_x-popt[0],self.mag_data_y-popt[1],self.mag_data_z-popt[2])
        return GratbotBehaviorStatus.COMPLETED

class CalibrateMagsensorPrintField(GratbotBehavior):

    def __init__(self):
        super().__init__()

    def act(self,comms,sensors):
        print("Current field {}".format(sensors.gratbot_state["magnetometer/b_field"]))
        print("corrected field {}".format(sensors.get_corrected_bfield()))
        print("compass heading {}".format(sensors.get_compass_heading()*360/(2*np.pi)))
        return GratbotBehaviorStatus.COMPLETED
