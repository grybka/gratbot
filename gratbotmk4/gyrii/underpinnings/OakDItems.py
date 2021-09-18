import depthai as dai
import logging
import time
import numpy as np
import os
from pathlib import Path
import blobconverter



logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


nnBlobPath = os.getcwd()+'/models/tiny-yolo-v4_openvino_2021.2_6shave.blob'

#### Initializing Things
def create_imu(pipeline,name="imu"):
    imu = pipeline.createIMU()
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.MAGNETOMETER_CALIBRATED,dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR,dai.IMUSensor.GYROSCOPE_CALIBRATED], 400)
    imu.setBatchReportThreshold(5)
    imu.setMaxBatchReports(20)
    imu_xlinkOut = pipeline.createXLinkOut()
    imu_xlinkOut.setStreamName(name)
    imu.out.link(imu_xlinkOut.input)


def create_depth(pipeline,name="depth"):
    #depth camera
    logger.info("Creating Stereo Camera in pipeline")

    monoLeft = pipeline.createMonoCamera()
    monoRight = pipeline.createMonoCamera()
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    stereo = pipeline.createStereoDepth()
    stereo.setConfidenceThreshold(255)
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    depthout=pipeline.createXLinkOut()
    depthout.setStreamName(name)
    stereo.disparity.link(depthout.input)
    return stereo

def create_yolo(pipeline,name="detections"):
    logger.info("Creating Spatial Detection Network")
    spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()
    spatialDetectionNetwork.setBlobPath(nnBlobPath)
    spatialDetectionNetwork.setConfidenceThreshold(0.5)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)

    # Yolo specific parameters
    spatialDetectionNetwork.setNumClasses(80)
    spatialDetectionNetwork.setCoordinateSize(4)
    spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
    spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
    spatialDetectionNetwork.setIouThreshold(0.5)

    xoutNN = pipeline.createXLinkOut()
    xoutNN.setStreamName(name)
    spatialDetectionNetwork.out.link(xoutNN.input)
    return spatialDetectionNetwork

#mobilenet models
def init_model(pipeline,model_name,camera,stereo,streamname='detections',shaves=6):
    #spatial detection of people
    spatialDetectionNetwork = pipeline.createMobileNetSpatialDetectionNetwork()
    spatialDetectionNetwork.setBlobPath(str(blobconverter.from_zoo(name=model_name, shaves=shaves)))
    spatialDetectionNetwork.setConfidenceThreshold(0.5)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)
    camera.link(spatialDetectionNetwork.input)
    stereo.depth.link(spatialDetectionNetwork.inputDepth)
    xoutNN = pipeline.createXLinkOut()
    xoutNN.setStreamName(streamname)
    spatialDetectionNetwork.out.link(xoutNN.input)


##### Getting things
last_gyro_Ts=0
local_rotation=np.zeros(3)

def tryget_imudata(imuQueue,broker):
    global local_rotation
    global last_gyro_Ts
    #get accelerometry data
    #return nothing
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
            local_rotation+=np.array([gyroscope.x,gyroscope.y,gyroscope.z])*(gyroscopeTs.total_seconds()-last_gyro_Ts)
            last_gyro_Ts=gyroscopeTs.total_seconds()
            dat.append({"rotation_vector": [rVvalues.real,rVvalues.i,rVvalues.j,rVvalues.k,rVvalues.accuracy],
                        "rotation_vector_timestamp": rvTs.total_seconds(),
                        "acceleration": [acceleroValues.x,acceleroValues.y,acceleroValues.z],
                        "acceleration_timestamp": acceleroTs.total_seconds(),
                        "magnetic_field": [magneticField.x,magneticField.y,magneticField.z],
                        "magneticTs": magneticTs.total_seconds(),
                        "local_rotation": local_rotation.tolist(),
                        "gyroscope": [gyroscope.x,gyroscope.y,gyroscope.z],
                        "gyroscope_timestamp": gyroscopeTs.total_seconds()})
        my_keys=["rotation_vector","acceleration","magnetic_field","gyroscope"]
        message={"keys": my_keys,
                 "timestamp": time.time(),
                 "packets": dat}
        broker.publish(message,my_keys)

def tryget_depth(depthQueue,broker):
    #get depth from the depth queu
    #return the depth image if possible or none
    inDepth = depthQueue.tryGet()
    if inDepth is not None:
        frame=inDepth.getFrame()
        #logger.debug("frame type {}".format(type(frame)))
        #logger.debug("frame shape {}".format(frame.shape))
        #logger.debug("frame dtype {}".format(frame.dtype))
        frame_message={"timestamp": time.time()}
        image_timestamp=inDepth.getTimestamp().total_seconds()
        #logger.debug("got image {}".format(image_timestamp))
        frame_message["image_timestamp"]=image_timestamp
        frame_message["depth_image"]=frame
        frame_message["keys"]=["depth"]
        broker.publish(frame_message,frame_message["keys"])
        return frame
    else:
        return None

def tryget_image(previewQueue,broker):
    #get an image from a image queue, publish
    #return a timestamp,the image
    inPreview = previewQueue.tryGet()
    if inPreview is not None:
        frame = inPreview.getCvFrame()
        frame_message={"timestamp": time.time()}
        image_timestamp=inPreview.getTimestamp().total_seconds()
        logger.debug("image at {}".format(image_timestamp))
        frame_message["image_timestamp"]=image_timestamp
        frame_message["image"]=frame
        frame_message["keys"]=["image"]
        broker.publish(frame_message,frame_message["keys"])
        return image_timestamp,frame
    else:
        return None,None

def tryget_nndetections(detectionNNQueue,broker,image,model_labels):
    #publish detections from a nn
    #no return
    inDet = detectionNNQueue.tryGet()
    if inDet is not None:
        print("indet is a {}".format(type(inDet.getData())))
        print("indet is {}".format(inDet.getData()))
        detection_message=[]
        for detection in inDet.detections:
            det_item={}
            bbox_array=[detection.xmin,detection.xmax,detection.ymin,detection.ymax]
            det_item["bbox_array"]=bbox_array
            det_item["spatial_array"]=[detection.spatialCoordinates.x,detection.spatialCoordinates.y,detection.spatialCoordinates.z]
            det_item["label"] = model_labels[detection.label]
            det_item["confidence"] = detection.confidence
            if image is not None:
                height = image.shape[0]
                width = image.shape[1]
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                det_item["subimage"]=image[y1:y2,x1:x2]
            detection_message.append(det_item)
        if len(detection_message)!=0:
            frame_message={"timestamp": time.time()}
            frame_message["detections"]=detection_message
            broker.publish(frame_message,["detections"])
        return None
    else:
        return None
