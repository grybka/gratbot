from Gyrus import ThreadedGyrus
import threading
import depthai as dai
import logging
import time
import os
import numpy as np

#nnBlobPath = str((Path(__file__).parent / Path('models/tiny-yolo-v4_openvino_2021.2_6shave.blob')).resolve().absolute())
nnBlobPath = os.getcwd()+'/models/tiny-yolo-v4_openvino_2021.2_6shave.blob'

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

class OakDGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.do_detection=True
        self.do_imu=True
        self.oak_comm_thread=None
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

            logging.debug("OakD created and queue's gotten")
            while not self.should_quit:
                #get detections data
                #get image data
                inPreview = previewQueue.get()
                frame = inPreview.getCvFrame()
                #TODO turn frame into message
                frame_message={"timestamp": time.time()}
                frame_message["image"]=frame
                frame_message["keys"]=["image"]
                if self.do_detection:
                    inDet = detectionNNQueue.get()
                    detections = inDet.detections
                    if len(detections)!=0:
                        detection_message=[]
                        for detection in detections:
                            bbox_array=[detection.xmin,detection.xmax,detection.ymin,detection.ymax]
                            spatial_array=[detection.spatialCoordinates.x,detection.spatialCoordinates.y,detection.spatialCoordinates.z]
                            try:
                                label = self.labelMap[detection.label]
                            except:
                                label = detection.label
                            detection_message.append({"label": label,
                                                      "spatial_array": spatial_array,
                                                      "bbox_array": bbox_array,
                                                      "confidence": detection.confidence})
                        self.broker.publish({"timestamp": time.time(),"detections": detection_message},["detections"]) #publish an indepedent detections message
                        frame_message["detections"]=detection_message #also append to image
                self.broker.publish(frame_message,frame_message["keys"])

                #get accelerometry data
                if self.do_imu:
                    imuData = imuQueue.get()
                    if len(imuData.packets) != 0:
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
                            dat.append({"rotation_vector": [rVvalues.real,rVvalues.i,rVvalues.j,rVvalues.k,rVvalues.accuracy],
                                        "rotation_vector_timestamp": rvTs.total_seconds(),
                                        "acceleration": [acceleroValues.x,acceleroValues.y,acceleroValues.z],
                                        "acceleration_timestamp": acceleroTs.total_seconds(),
                                        "magnetic_field": [magneticField.x,magneticField.y,magneticField.z],
                                        "magneticTs": magneticTs.total_seconds(),
                                        "gyroscope": [gyroscope.x,gyroscope.y,gyroscope.z],
                                        "gyroscope_timestamp": gyroscopeTs.total_seconds()})
                        my_keys=["rotation_vector","acceleration","magnetic_field"]
                        message={"keys": my_keys,
                                 "timestamp": time.time()
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
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(20)

        #rgb camera
        logger.info("Creating RGB Camera in pipeline")
        camRgb = self.pipeline.createColorCamera()
        camRgb.setPreviewSize(416, 416)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
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
