from Gyrus import ThreadedGyrus
import threading
import depthai as dai
import logging
import time

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
            logging.debug("OakD created and queue's gotten")
            while not self.should_quit:
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
        imu_xlinkOut = self.pipeline.createXLinkOut()
        imu_xlinkOut.setStreamName("imu")
        imu.setBatchReportThreshold(10)
        imu.setMaxBatchReports(20)
        imu.out.link(imu_xlinkOut.input)
