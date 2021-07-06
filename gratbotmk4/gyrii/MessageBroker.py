#A message broker class that I intend to be like a pared down AMQP
#passing python messages between threads
import queue

#a queue that approximately respects the size limit and starts throwing out old data if it hits max size
class BrokerQueue(queue.Queue):
    def __init__(self,name,approx_max_size):
        super().__init__()
        self.name=name
        self.approx_max_size=approx_max_size

    def aggressive_put(self,message):
        self.put(message)
        if self.qsize()>self.approx_max_size:
            self.get()


class MessageBroker:
    def __init__(self):
        #a mapping from message routing key to queue
        self.key_to_queue_map={}
        self.queues={}
        self.queue_sizes={}

    def subscribe(self,keys,queue_name,queue_size=200):
        if queue_name in self.queues:
            raise Exception("Queue already exists")
        self.queues[queue_name]=BrokerQueue(queue_name,queue_size)
        for key in keys:
            if key in self.key_to_queue_map:
                self.key_to_queue_map[key].append(self.queues[queue_name])
            else:
                self.key_to_queue_map[key]=[ self.queues[queue_name] ]

    def publish(self,message,keys):
        if isinstance(keys, dict):
            raise Exception("You messed up your message and keys order")
        if isinstance(keys, str):
            keys=[ keys ]
        queues_to_deliver_to=set()
        for key in keys:
            if key not in self.key_to_queue_map:
                continue
            for q in self.key_to_queue_map[key]:
                queues_to_deliver_to.add(q)
        for q in queues_to_deliver_to:
            q.aggressive_put(message)

    def receive(self,queue_name,timeout=0.1,block=True):
        return self.queues[queue_name].get(block=block,timeout=timeout)

    #TODO have a way to unsubscribe
