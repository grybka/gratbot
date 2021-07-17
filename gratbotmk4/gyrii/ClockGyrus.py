#send out clock pulses, used to trigger behaviors so far

from Gyrus import ThreadedGyrus
import threading,queue
import time

class ClockGyrus(ThreadedGyrus):
    def __init__(self,broker):
        self.broker=broker
        #self.clock_pulse_period=0.05
        self.clock_pulse_period=0.1
        self._clock_thread = threading.Thread(target=self._run_clock)
        self._clock_thread.daemon = True
        self.should_quit=False
        super().__init__(broker)

    def get_keys(self):
        return []

    def get_name(self):
        return "ClockGyrus"

    def start_thread_called(self):
        self._clock_thread.start()

    def join_called(self):
        self.should_quit=True
        return self._clock_thread.join()

    def _run_clock(self):
        while not self.should_quit:
            time.sleep(self.clock_pulse_period)
            self.broker.publish({"timestamp": time.time(),"clock_pulse": self.clock_pulse_period},"clock_pulse")

    def read_message(self,message):
        ...
