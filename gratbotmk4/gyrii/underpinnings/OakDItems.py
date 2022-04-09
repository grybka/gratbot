import depthai as dai
import logging
import time
import numpy as np
import os
from pathlib import Path
import blobconverter

#Rule:
#Detection bounding boxes should be given in terms of fraction of full width or height


logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


nnBlobPath = os.getcwd()+'/models/tiny-yolo-v4_openvino_2021.2_6shave.blob'

#### Initializing Things
def create_imu(pipeline,name="imu"):
    imu = pipeline.createIMU()
    #imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.MAGNETOMETER_CALIBRATED,dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR,dai.IMUSensor.GYROSCOPE_CALIBRATED], 400)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.MAGNETOMETER_CALIBRATED,dai.IMUSensor.ROTATION_VECTOR,dai.IMUSensor.GYROSCOPE_CALIBRATED], 400)
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
    xoutNNpassthru = pipeline.createXLinkOut()
    xoutNNpassthru.setMetadataOnly(True)
    xoutNNpassthru.setStreamName(streamname+"_passthru")
    spatialDetectionNetwork.passthrough.link(xoutNNpassthru.input)


#mobilenet models
def init_model_nopassthru(pipeline,model_name,camera,stereo,streamname='detections',shaves=6):
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

#class agnostic
def init_class_agnostic(pipeline,camera):
    NN_PATH = blobconverter.from_zoo(name="mobile_object_localizer_192x192", zoo_type="depthai")
    NN_WIDTH = 192
    NN_HEIGHT = 192
    PREVIEW_WIDTH = 640
    PREVIEW_HEIGHT = 360

    # Define a neural network that will make predictions based on the source frames
    detection_nn = pipeline.create(dai.node.NeuralNetwork)
    detection_nn.setBlobPath(NN_PATH)
    detection_nn.setNumPoolFrames(4)
    detection_nn.input.setBlocking(False)
    detection_nn.setNumInferenceThreads(2)

    # Create manip
    manip = pipeline.create(dai.node.ImageManip)
    manip.initialConfig.setResize(NN_WIDTH, NN_HEIGHT)
    manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)
    manip.initialConfig.setKeepAspectRatio(False)

    xout_nn = pipeline.create(dai.node.XLinkOut)
    xout_nn.setStreamName("nn")
    xout_nn.input.setBlocking(False)

    camera.preview.link(manip.inputImage)
    manip.out.link(detection_nn.input)
    detection_nn.out.link(xout_nn.input)



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
            #rValues.accuracy didn't work
            dat.append({"rotation_vector": [rVvalues.real,rVvalues.i,rVvalues.j,rVvalues.k,3],
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
        #logger.debug("image at {}".format(image_timestamp))
        frame_message["image_timestamp"]=image_timestamp
        frame_message["image"]=frame
        frame_message["keys"]=["image"]

        sequenceNum=inPreview.getSequenceNum()
        logger.debug("Image squence num {}".format(sequenceNum))


        broker.publish(frame_message,frame_message["keys"])
        return image_timestamp,frame
    else:
        return None,None

def tryget_nndetections(detectionNNQueue,passthruQueue,broker,image,model_labels):
    #publish detections from a nn
    #no return
    inDet = detectionNNQueue.tryGet()
    if inDet is not None:
        #logger.debug("nn deetection got")
        metadata=passthruQueue.get()
        #image_seqnum=metadata.getSequenceNum()
        device_timestamp=metadata.getTimestamp().total_seconds()
        #logger.debug("metadata deetection got timestamp {}".format(device_timestamp))
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
                #  (x-0.5)+w/(2h)
                xmin=(detection.xmin-0.5)*height/width+0.5
                xmax=(detection.xmax-0.5)*height/width+0.5

                bbox_array=[xmin,xmax,detection.ymin,detection.ymax]
                det_item["bbox_array"]=bbox_array
                #so the awkward thing is that I am looking at a square, so the height is the determining factor
                #and the x maps 0.5 -> 0.5,  x -> (x-0.5)*height+width/2
                #logger.debug("{} {} {} {}".format(detection.xmin,detection.xmax,detection.ymin,detection.ymax))
                x1 = int( (detection.xmin -0.5)* height+width/2)
                x2 = int( (detection.xmax -0.5)* height+width/2)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                x1=np.clip(x1,1,min(width-1,x2))
                x2=np.clip(x2,max(1,x1),width-1)
                y1=np.clip(y1,1,min(height-1,y2))
                y2=np.clip(y2,max(1,y1),height-1)

                #logger.debug("image shape {}".format(image.shape))
                #logger.debug("x1 x2 y1 y2 {} {} {} {}".format(x1,x2,y1,y2))
                det_item["subimage"]=image[y1:y2,x1:x2]
            detection_message.append(det_item)
        if len(detection_message)!=0:
            frame_message={"timestamp": time.time(),"image_timestamp": device_timestamp}
            frame_message["detection_name"]="detections_hardware"
            frame_message["detections"]=detection_message
            broker.publish(frame_message,["detections"])
        return None
    else:
        return None


def tryget_nndetections_nopassthru(detectionNNQueue,broker,model_labels):
    image=None
    #publish detections from a nn
    #no return
    #assume its squeezed

    inDet = detectionNNQueue.tryGet()
    if inDet is not None:
        #logger.debug("nn deetection got")
        #image_seqnum=metadata.getSequenceNum()
        device_timestamp=inDet.getTimestamp().total_seconds()
        #logger.debug("metadata deetection got timestamp {}".format(device_timestamp))
        detection_message=[]
        sequenceNum=inDet.getSequenceNum()
        logger.debug("Image squence num {}".format(sequenceNum))
        for detection in inDet.detections:
            det_item={}
            bbox_array=[detection.xmin,detection.xmax,detection.ymin,detection.ymax]
            det_item["bbox_array"]=bbox_array
            det_item["spatial_array"]=[detection.spatialCoordinates.x,detection.spatialCoordinates.y,detection.spatialCoordinates.z]
            det_item["label"] = model_labels[detection.label]
            det_item["confidence"] = detection.confidence
            det_item["subimage"] = 0
            detection_message.append(det_item)
        if len(detection_message)!=0:
            frame_message={"timestamp": time.time(),"image_timestamp": device_timestamp}
            frame_message["detection_name"]="detections_hardware"
            frame_message["detections"]=detection_message
            broker.publish(frame_message,["detections"])
        return None
    else:
        return None


def tryget_classagnostic(q_nn,broker,image):
    in_nn = q_nn.get()
    THRESHOLD=0.2
    if in_nn is not None:
        # get outputs
        device_timestamp=in_nn.getTimestamp().total_seconds()
        detection_boxes = np.array(in_nn.getLayerFp16("ExpandDims")).reshape((100, 4))
        detection_scores = np.array(in_nn.getLayerFp16("ExpandDims_2"))
        preview_xshape=192
        preview_yshape=192
        xshape=640
        yshape=360

        # keep boxes bigger than threshold
        mask = detection_scores >= THRESHOLD
        boxes = detection_boxes[mask]
        #colors = colors_full[mask]
        scores = detection_scores[mask]
        detection_message=[]
        for i in range(boxes.shape[0]):
            box = boxes[i]
            y1 = box[0]
            y2= box[2]
            x1=box[1]
            x2=box[3]
            #y1 = (yshape * box[0]).astype(int)
            #y2 = (yshape * box[2]).astype(int)
            #x1 = (xshape * box[1]).astype(int)
            #x2 = (xshape * box[3]).astype(int)
            #if x2-x1 > 640*0.75 or y2-y1>360*0.75:
            #    continue #objects that use up 75$ of the area are too close
            preview_y1 = (preview_yshape * box[0]).astype(int)
            preview_y2 = (preview_yshape * box[2]).astype(int)
            preview_x1 = (preview_xshape * box[1]).astype(int)
            preview_x2 = (preview_xshape * box[3]).astype(int)


            bbox_array=[x1,x2,y1,y2]
            det_item={}
            det_item["bbox_array"]=bbox_array
            #det_item["spatial_array"]=[detection.spatialCoordinates.x,detection.spatialCoordinates.y,detection.spatialCoordinates.z]
            #det_item["label"] = model_labels[detection.label]
            det_item["label"] = "unknown"
            det_item["confidence"] = detection_scores[i]
            if image is not None:
                det_item["subimage"]=image[preview_y1:preview_y2,preview_x1:preview_x2]
            else:
                det_item["subimage"]=0
            detection_message.append(det_item)
        if len(detection_message)!=0:
            frame_message={"timestamp": time.time(),"image_timestamp": device_timestamp}
            frame_message["detection_name"]="detections_hardware"
            frame_message["detections"]=detection_message
            broker.publish(frame_message,["detections"])
    return None
