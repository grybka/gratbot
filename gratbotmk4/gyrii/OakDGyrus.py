from Gyrus import ThreadedGyrus
import threading
import depthai as dai
import logging
import time

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
        self.oak_comm_thread = threading.Thread(target=self._oak_comm_thread_loop)
        self.oak_comm_thread.daemon = True
        super().__init__(broker)

    def start_thread_called(self):
        logging.debug("starting OakD Comms Thread")
        self.oak_comm_thread.start()

    def join_called(self):
        return self.oak_comm_thread.join()

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
            detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

            logging.debug("OakD created and queue's gotten")
            while not self.should_quit:
                #get detections data
                #get image data
                inPreview = previewQueue.get()
                inDet = detectionNNQueue.get()
                frame = inPreview.getCvFrame()
                detections = inDet.detections
                #TODO turn frame into message
                frame_message={}
                frame_message["image"]=frame
                frame_message["keys"]=["image"]
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
                    frame_message["detections"]=detection_message
                    frame_message["keys"].append("detections")
                self.broker.publish(frame_message,frame_message["keys"])

                #get accelerometry data
                imuData = imuQueue.get()
                if len(imuData.packets) != 0:
                    dat=[]
                    for imuPacket in imuData.packets:
                        rVvalues = imuPacket.rotationVector
                        acceleroValues = imuPacket.acceleroMeter
                        magneticField = imuPacket.magneticField
                        rvTs = rVvalues.timestamp.get()
                        acceleroTs = acceleroValues.timestamp.get()
                        magneticTs = magneticField.timestamp.get()
                        dat.append({"rotation_vector": [rVvalues.i,rVvalues.j,rVvalues.k,rVvalues.real,rVvalues.accuracy],
                                    "rotation_vector_timestamp": rvTs.total_seconds(),
                                    "acceleration": [acceleroValues.x,acceleroValues.y,acceleroValues.z],
                                    "acceleration_timestamp": acceleroTs.total_seconds(),
                                    "magnetic_field": [magneticField.x,magneticField.y,magneticField.z],
                                    "magneticTs": magneticTs.total_seconds(),
                                    "timestamp": time.time()})
                    my_keys=["rotation_vector","acceleration","magnetic_field"]
                    message={"keys": my_keys,
                             "packets": dat}
                    self.broker.publish(message,my_keys)

            logging.debug("Exiting OakD thread")

    def init_oakd(self):
        self.pipeline = dai.Pipeline()
        self.pipeline.setOpenVINOVersion(dai.OpenVINO.Version.VERSION_2021_2)

        #setup accelerometer/magnetometere
        imu = self.pipeline.createIMU()
        imu.enableIMUSensor([dai.IMUSensor.LINEAR_ACCELERATION, dai.IMUSensor.MAGNETOMETER_CALIBRATED,dai.IMUSensor.ARVR_STABILIZED_GAME_ROTATION_VECTOR], 400)
        imu.setBatchReportThreshold(10)
        imu.setMaxBatchReports(20)

        #rgb camera
        camRgb = self.pipeline.createColorCamera()
        camRgb.setPreviewSize(416, 416)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        #depth camera
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
        detection=False
        if dectection:
            spatialDetectionNetwork.setBlobPath(nnBlobPath)
            spatialDetectionNetwork.setConfidenceThreshold(0.5)
            spatialDetectionNetwork.input.setBlocking(False)
            spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
            spatialDetectionNetwork.setDepthLowerThreshold(100)
            spatialDetectionNetwork.setDepthUpperThreshold(5000)

            # Yolo specific parameters
            spatialDetectionNetwork = self.pipeline.createYoloSpatialDetectionNetwork()
            spatialDetectionNetwork.setNumClasses(80)
            spatialDetectionNetwork.setCoordinateSize(4)
            spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
            spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
            spatialDetectionNetwork.setIouThreshold(0.5)
            stereo.depth.link(spatialDetectionNetwork.inputDepth)
            camRgb.preview.link(spatialDetectionNetwork.input)

        #outputs
        #RGB Camera (after detections)
        xoutRgb = self.pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        if detection:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)



        #IMU output
        imu_xlinkOut = self.pipeline.createXLinkOut()
        imu_xlinkOut.setStreamName("imu")
        imu.out.link(imu_xlinkOut.input)

        #Detection bounding boxes
        xoutNN = pipeline.createXLinkOut()
        xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()
        spatialDetectionNetwork.out.link(xoutNN.input)
        spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)
