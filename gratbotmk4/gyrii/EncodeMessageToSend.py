
import numpy as np
import cv2
import logging
import base64
import uuid
import collections

def encode_message_to_send(object):
    if type(object)==dict or type(object)==collections.OrderedDict: #walk through dicts
        ret={}
        for key in object:
            ret[key]=encode_message_to_send(object[key])
        return ret
    if type(object)==list: #walk through lists
        ret=[]
        for elem in object:
            ret.append(encode_message_to_send(elem))
        return ret
    if type(object)==np.ndarray:
        if object.dtype==np.uint8:
            _,encoded=cv2.imencode('.jpg',object)
            encoded=encoded.tobytes()
            return {"_packed_type":"jpegimage","data":x.decode('utf-8')}
        else:
            return object
    return object

def decode_message_received(object):
    if type(object)==dict: #walk through dicts
        if "_packed_type" in object:
            if object["_packed_type"]=="jpegimage":
                x=bytes(object["data"],encoding='utf-8')
                x=np.frombuffer(x,dtype=np.uint8)
                return cv2.imdecode(x,cv2.IMREAD_COLOR)
            raise Exception("json_to_message doesn't understand type {}".format(object["_packed_type"]))
        ret={}
        for key in object:
            ret[key]=json_to_message(object[key])
        return ret
    if type(object)==list: #walk through lists
        ret=[]
        for elem in object:
            ret.append(decode_message_received(elem))
        return ret
    return object
