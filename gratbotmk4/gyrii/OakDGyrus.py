from Gyrus import ThreadedGyrus
import threading
import depthai as dai

class OakDGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.oak_comm_thread = threading.Thread(target=self._oak_comm_thread_loop)
        self.oak_comm_thread.daemon = True
        self.super().__init__()

    def start_thread_called(self):
        self.oak_comm_thread.start_thread()

    def get_keys(self):
        return [] #nothing yet

    def get_name(self):
        return "OakDGyrus"

    def read_message(self,message):
        self.output_queue.put(object_to_json)

    def _oak_comm_thread_loop(self):
        self.init_oakd()
        with dai.Device(pipeline) as device:
            imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
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
                        dat.append({"rotation_vector": rVvalues,
                                    "rotation_vector_timestamp": rvTs,
                                    "acceleration": acceleroValues,
                                    "acceleration_timestamp": acceleroTs,
                                    "magnetic_field": magnetiField,
                                    "magneticTs": magneticTs})
                    message={"keys": ["rotation_vector","acceleration","magnetic_field"],
                             "packets": dat}

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
