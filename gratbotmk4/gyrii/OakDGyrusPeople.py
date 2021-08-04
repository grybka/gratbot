
from Gyrus import ThreadedGyrus
import threading
import depthai as dai
import logging
import time
import os
import numpy as np
#import blobconverter
from pathlib import Path
import blobconverter

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class OakDGyrusPeople(ThreadedGyrus):
    def __init__(self,broker):
        self.oak_comm_thread=None
        #self.model="person-detection-retail-0013"
        self.models=[ {"modelname": "person-detection-0200",
                       "streamname": "person_detections",
                       "labels": ["person"]},
                      {"modelname": "face-detection-0200",
                       "streamname": "face_detections",
                       "labels": ["face"]}]
        #self.model1="person-detection-0200"
        #self.model2="face-detection-0200"
        self.local_rotation=np.zeros(3)
        self.last_gyro_Ts=0
        super().__init__(broker)

    def start_thread_called(self):
        logging.debug("starting OakD Comms Thread")
        self.oak_comm_thread = threading.Thread(target=self._oak_comm_thread_loop)
        self.oak_comm_thread.daemon = True
        self.oak_comm_thread.start()

    def join_called(self):
        if self.oak_comm_thread is not None:
            return self.oak_comm_thread.join()
        return None

    def get_keys(self):
        return [] #nothing yet

    def get_name(self):
        return "OakDGyrusPeople"

    def read_message(self,message):
        self.output_queue.put(object_to_json)

    def tryget_imudata(self,imuQueue):
        #get accelerometry data
        imuData = imuQueue.tryGet()
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
                dat.append({"rotation_vector": [rVvalues.real,rVvalues.i,rVvalues.j,rVvalues.k,rVvalues.accuracy],
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
            self.broker.publish(message,my_keys)

    def tryget_image(self,previewQueue):
        inPreview = previewQueue.tryGet()
        if inPreview is not None:
            frame = inPreview.getCvFrame()
            frame_message={"timestamp": time.time()}
            image_timestamp=inPreview.getTimestamp().total_seconds()
            frame_message["image_timestamp"]=image_timestamp
            frame_message["image"]=frame
            frame_message["keys"]=["image"]
            detection_message=[]
            for model in self.models:
                detectionNNQueue=model["queue"]
                inDet = detectionNNQueue.tryGet()
                if inDet is not None:
                    for detection in inDet.detections:
                        bbox_array=[detection.xmin,detection.xmax,detection.ymin,detection.ymax]
                        spatial_array=[detection.spatialCoordinates.x,detection.spatialCoordinates.y,detection.spatialCoordinates.z]
                        label = model["labels"][detection.label]
                        detection_message.append({"label": label,
                                                  "spatial_array": spatial_array,
                                                  "bbox_array": bbox_array,
                                                  "confidence": detection.confidence})
            if len(detection_message)!=0:
                self.broker.publish({"timestamp": time.time(),"image_timestamp": image_timestamp,"detections": detection_message, "keys": ["detections"]},["detections"]) #publish an indepedent detections message
                frame_message["detections"]=detection_message #also append to image
            self.broker.publish(frame_message,frame_message["keys"])

    def tryget_depth(depthQueue):
        inDepth = depthQueue.tryGet()
        if inDepth is not None:
            frame=inDepth.getFrame()
            frame_message={"timestamp": time.time()}
            image_timestamp=inDepth.getTimestamp().total_seconds()
            frame_message["image_timestamp"]=image_timestamp
            frame_message["image"]=frame
            frame_message["keys"]=["depth"]
            self.broker.publish(frame_message,frame_message["keys"])

    def _oak_comm_thread_loop(self):
        self.init_oakd()
        with dai.Device(self.pipeline) as device:
            imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4,blocking=False)

            for model in self.models:
                model["queue"] = device.getOutputQueue(name=model["streamname"], maxSize=4, blocking=False)
            logging.debug("OakD created and queue's gotten")
            while not self.should_quit:
                self.tryget_imudata(imuQueue)
                self.tryget_image(previewQueue)
                self.tryget_depth(depthQueue)
        logging.debug("Exiting OakD thread")

    def init_model(self,model_name,camRgb,stereo,streamname='detections',shaves=6):
        #spatial detection of people
        spatialDetectionNetwork = self.pipeline.createMobileNetSpatialDetectionNetwork()
        spatialDetectionNetwork.setBlobPath(str(blobconverter.from_zoo(name=model_name, shaves=shaves)))
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)
        camRgb.preview.link(spatialDetectionNetwork.input)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        xoutNN = self.pipeline.createXLinkOut()
        xoutNN.setStreamName(streamname)
        spatialDetectionNetwork.out.link(xoutNN.input)

    def init_oakd(self):
        self.pipeline = dai.Pipeline()
        self.pipeline.setOpenVINOVersion(dai.OpenVINO.Version.VERSION_2021_2)

        #setup accelerometer/magnetometere
        logger.info("Creating IMU in pipeline")
        imu = self.pipeline.createIMU()
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.MAGNETOMETER_CALIBRATED,dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR,dai.IMUSensor.GYROSCOPE_CALIBRATED], 400)
        imu.setBatchReportThreshold(5)
        imu.setMaxBatchReports(20)
        imu_xlinkOut = self.pipeline.createXLinkOut()
        imu_xlinkOut.setStreamName("imu")
        imu.out.link(imu_xlinkOut.input)


        #setup camera
        logger.info("Creating RGB Camera in pipeline")
        camRgb = self.pipeline.createColorCamera()
        #camRgb.setPreviewSize(544, 320)
        camRgb.setPreviewSize(256, 256)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        xoutRgb = self.pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        camRgb.preview.link(xoutRgb.input)

        #depth camera
        logger.info("Creating Stereo Camera in pipeline")
        monoLeft = self.pipeline.createMonoCamera()
        monoRight = self.pipeline.createMonoCamera()
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        stereo = self.pipeline.createStereoDepth()
        stereo.setConfidenceThreshold(128) #TODO is this good?
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        depthout=self.pipline.createXLinkOut()
        depthout.setStreamName("depth")
        stereo.depth.link(depthout)

        for model in self.models:
            self.init_model(model["modelname"],camRgb,stereo,streamname=model["streamname"],shaves=6)
        #self.init_model(self.model2,camRgb,stereo,streamname='face_detections',shaves=6)
