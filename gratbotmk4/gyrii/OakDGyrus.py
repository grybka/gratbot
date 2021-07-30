from Gyrus import ThreadedGyrus
import threading
import depthai as dai
import logging
import time
import os
import numpy as np
#import blobconverter
from pathlib import Path

#nnBlobPath = str((Path(__file__).parent / Path('models/tiny-yolo-v4_openvino_2021.2_6shave.blob')).resolve().absolute())
nnBlobPath = os.getcwd()+'/models/tiny-yolo-v4_openvino_2021.2_6shave.blob'
faceBlobPath = os.getcwd()+'/models/face-detection-retail-0004_openvino_2021.2_4shave.blob'

logger=logging.getLogger(__name__)
#logger.setLevel(logging.WARNING)
logger.setLevel(logging.INFO)


# Tiny yolo v3/4 label texts
labelMap = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]

def frame_norm(frame, bbox):
    norm_vals = np.full(len(bbox), frame.shape[0])
    norm_vals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * norm_vals).astype(int)

class OakDGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.do_detection=True
        self.watch_faces=False
        self.do_imu=True
        self.oak_comm_thread=None
        #for keeping track of rotation
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
        return "OakDGyrus"

    def read_message(self,message):
        self.output_queue.put(object_to_json)

    def _oak_comm_thread_loop(self):
        self.init_oakd()
        with dai.Device(self.pipeline) as device:
            imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            if self.do_detection:
                detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            if self.watch_faces:
                face_nn = device.getOutputQueue("face_nn")

            logging.debug("OakD created and queue's gotten")
            while not self.should_quit:
                #get detections data
                #get image data
                inPreview = previewQueue.tryGet()
                if inPreview is not None:
                    frame = inPreview.getCvFrame()
                    #TODO turn frame into message
                    frame_message={"timestamp": time.time()}
                    image_timestamp=inPreview.getTimestamp().total_seconds()
                    frame_message["image_timestamp"]=image_timestamp
                    frame_message["image"]=frame
                    frame_message["keys"]=["image"]
                    detection_message=[]
                    if self.do_detection:
                        inDet = detectionNNQueue.get()
                        detections = inDet.detections
                        if len(detections)!=0:
                            for detection in detections:
                                bbox_array=[detection.xmin,detection.xmax,detection.ymin,detection.ymax]
                                spatial_array=[detection.spatialCoordinates.x,detection.spatialCoordinates.y,detection.spatialCoordinates.z]
                                try:
                                    label = labelMap[detection.label]
                                except:
                                    label = detection.label
                                detection_message.append({"label": label,
                                                          "spatial_array": spatial_array,
                                                          "bbox_array": bbox_array,
                                                          "confidence": detection.confidence})
                    if self.watch_faces:
                        bboxes = np.array(face_nn.get().getFirstLayerFp16())
                        bboxes = bboxes.reshape((bboxes.size // 7, 7))
                        #cut_bboxes = bboxes[bboxes[:, 2] > 0.7][:, 3:7]
                        bboxes = bboxes[bboxes[:,2]>0.7][:,:]
                        for raw_bbox in bboxes:
                            b=raw_bbox[3:7]
                            detection_message.append({"label": "face",
                                                      "bbox_array": [b[0],b[2],b[1],b[3]],
                                                      "confidence": raw_bbox[2]})
                    if len(detection_message)!=0:
                        self.broker.publish({"timestamp": time.time(),"image_timestamp": image_timestamp,"detections": detection_message, "keys": ["detections"]},["detections"]) #publish an indepedent detections message
                        frame_message["detections"]=detection_message #also append to image

                    self.broker.publish(frame_message,frame_message["keys"])

                #get accelerometry data
                if self.do_imu:
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

            logging.debug("Exiting OakD thread")

    def init_oakd(self):
        self.pipeline = dai.Pipeline()
        self.pipeline.setOpenVINOVersion(dai.OpenVINO.Version.VERSION_2021_2)

        #setup accelerometer/magnetometere
        logger.info("Creating IMU in pipeline")
        imu = self.pipeline.createIMU()
        #imu.enableIMUSensor([dai.IMUSensor.LINEAR_ACCELERATION, dai.IMUSensor.MAGNETOMETER_CALIBRATED,dai.IMUSensor.ARVR_STABILIZED_GAME_ROTATION_VECTOR,dai.IMUSensor.GYROSCOPE_CALIBRATED], 400)
        imu.enableIMUSensor([dai.IMUSensor.LINEAR_ACCELERATION, dai.IMUSensor.MAGNETOMETER_CALIBRATED,dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR,dai.IMUSensor.GYROSCOPE_CALIBRATED], 400)
        imu.setBatchReportThreshold(5)
        imu.setMaxBatchReports(20)

        #rgb camera
        logger.info("Creating RGB Camera in pipeline")
        camRgb = self.pipeline.createColorCamera()
        camRgb.setPreviewSize(416, 416)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        #camRgb.setPreviewKeepAspectRatio(False)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        #depth camera
        logger.info("Creating Stereo Camera in pipeline")
        monoLeft = self.pipeline.createMonoCamera()
        monoRight = self.pipeline.createMonoCamera()
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        stereo = self.pipeline.createStereoDepth()
        stereo.setConfidenceThreshold(255)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        #detection
        if self.do_detection:
            logger.info("Creating Spatial Detection Network")
            spatialDetectionNetwork = self.pipeline.createYoloSpatialDetectionNetwork()
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
            stereo.depth.link(spatialDetectionNetwork.inputDepth)
            camRgb.preview.link(spatialDetectionNetwork.input)

        #faces
        if self.watch_faces:
            face_manip=self.pipeline.createImageManip()
            face_manip.initialConfig.setResize(300,300)

            face_nn = self.pipeline.createNeuralNetwork()
            face_nn.setBlobPath(faceBlobPath)

            face_nn_xout = self.pipeline.createXLinkOut()
            face_nn_xout.setStreamName("face_nn")

            camRgb.preview.link(face_manip.inputImage)
            face_manip.out.link(face_nn.input)
            face_nn.out.link(face_nn_xout.input)

        #outputs
        #RGB Camera (after detections)

        logger.info("Making connections")
        xoutRgb = self.pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        if self.do_detection:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)



        #IMU output
        imu_xlinkOut = self.pipeline.createXLinkOut()
        imu_xlinkOut.setStreamName("imu")
        imu.out.link(imu_xlinkOut.input)

        #Detection bounding boxes
        if self.do_detection:
            xoutNN = self.pipeline.createXLinkOut()
            xoutNN.setStreamName("detections")
            spatialDetectionNetwork.out.link(xoutNN.input)
            #xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()
            #spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)
