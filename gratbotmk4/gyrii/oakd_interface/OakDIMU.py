import depthai as dai
import logging
import time
import numpy as np
import os
from pathlib import Path
import blobconverter
from oakd_interface.OakDElement import OakDElement

logger=logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class OakDIMU(OakDElement):
    def __init__(self,pipeline,streamname="imu"):
        self.streamname=streamname
        imu = pipeline.createIMU()
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.MAGNETOMETER_CALIBRATED,dai.IMUSensor.ROTATION_VECTOR,dai.IMUSensor.GYROSCOPE_CALIBRATED], 100)
        #imu.setBatchReportThreshold(5)
        imu.setBatchReportThreshold(4)
        #imu.setMaxBatchReports(20)
        imu.setMaxBatchReports(20)
        imu_xlinkOut = pipeline.createXLinkOut()
        imu_xlinkOut.setStreamName(streamname)
        imu.out.link(imu_xlinkOut.input)
        self.imu=imu
        self.last_gyro_Ts=0
        self.local_rotation=np.zeros(3)

    def build_queues(self,device):
        self.imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

    def tryget(self,broker):
        #get accelerometry data
        #return nothing
        imuData = self.imuQueue.tryGet()
        if imuData is not None and len(imuData.packets) != 0:
            dat=[]
            for imuPacket in imuData.packets:
                rVvalues = imuPacket.rotationVector
                acceleroValues = imuPacket.acceleroMeter
                magneticField = imuPacket.magneticField
                gyroscope = imuPacket.gyroscope
                gyroscopeTs = gyroscope.timestamp.get()
                rvTs = rVvalues.timestamp.get()
                acceleroTs = acceleroValues.timestamp.get()
                magneticTs = magneticField.timestamp.get()
                self.local_rotation+=np.array([gyroscope.x,gyroscope.y,gyroscope.z])*(gyroscopeTs.total_seconds()-self.last_gyro_Ts)
                self.last_gyro_Ts=gyroscopeTs.total_seconds()
                #rValues.accuracy didn't work
                dat.append({"rotation_vector": [rVvalues.real,rVvalues.i,rVvalues.j,rVvalues.k,3],
                            "rotation_vector_timestamp": rvTs.total_seconds(),
                            "acceleration": [acceleroValues.x,acceleroValues.y,acceleroValues.z],
                            "acceleration_timestamp": acceleroTs.total_seconds(),
                            "magnetic_field": [magneticField.x,magneticField.y,magneticField.z],
                            "magneticTs": magneticTs.total_seconds(),
                            "local_rotation": self.local_rotation.tolist(),
                            "gyroscope": [gyroscope.x,gyroscope.y,gyroscope.z],
                            "gyroscope_timestamp": gyroscopeTs.total_seconds()})
            my_keys=["rotation_vector","acceleration","magnetic_field","gyroscope"]
            message={"keys": my_keys,
                     "timestamp": time.time(),
                     "packets": dat}
            broker.publish(message,my_keys)
