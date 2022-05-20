from Gyrus import ThreadedGyrus
import numpy as np
import cv2
import threading,queue
import logging
from EncodeMessageToSend import encode_message_to_send,decode_message_received

logger=logging.getLogger(__name__)
#logger.setLevel(logging.DEBUG)
logger.setLevel(logging.INFO)
#listen for requests.

#messages must have
#"keys"


class SocketGyrusLink(ThreadedGyrus):
    def __init__(self,broker,incoming_queue,outgoing_queue,keys=[]):
        self.keys=keys
        self.incoming_queue=incoming_queue
        self.outgoing_queue=outgoing_queue
        self.receive_thread = threading.Thread(target=self._receive_thread_loop)
        self.receive_thread.daemon = True
        super().__init__(broker)

    def start_thread_called(self):
        self.receive_thread.start()

    def join_called(self):
        return self.receive_thread.join()

    def get_keys(self):
        return self.keys

    def get_name(self):
        return "SocketGyrusLinkServer"

    def read_message(self,message):
        if self.outgoing_queue.full():
            to_toss=self.outgoing_queue.get(block=False) #throw away packets if they aren't getting sent
            logger.debug("Throwing away a packet with keys {}".format(to_toss["keys"]))
            #logger.warning("Throwing away a packet with keys {}".format(to_toss["keys"]))
        self.outgoing_queue.put(encode_message_to_send(message),timeout=0.1)

    def _receive_thread_loop(self):
        while not self.should_quit:
            try:
                broadcast_message=self.incoming_queue.get(block=True,timeout=1)
                object_to_broadcast=decode_message_received(broadcast_message)
                if "keys" in object_to_broadcast:
                    self.broker.publish(object_to_broadcast,object_to_broadcast["keys"])
                else:
                    logger.error(" keys not in socketgyruslink message! {}".format(broadcast_message))
            except queue.Empty:
                logger.debug("queue empty")
                ... #no big deal, just try again
        logger.debug("SocketGyrusLink receive_thread quitting")