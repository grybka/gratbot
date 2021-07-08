
import numpy as np
import cv2
import logging
import base64

def message_to_json(object):
    if type(object)==dict: #walk through dicts
        ret={}
        for key in object:
            ret[key]=message_to_json(object[key])
        return ret
    if type(object)==list: #walk through lists
        ret=[]
        for elem in object:
            ret.append(message_to_json(elem))
        return ret
    if type(object)==np.ndarray:
        if object.dtype==np.uint8:
            #images are special
            _,encoded=cv2.imencode('.jpg',object)
            encoded=encoded.tobytes()
            x=base64.b64encode(encoded)
            return {"_packed_type":"jpegimage","data":x.decode('utf-8')}
        else:
            return {"_packed_type":"ndarray","data": message_to_json(object.tolist())}
    #presume anything else is OK.  I could have some more checks here
    if type(object) in [str,int,float,complex,bool]:
        return object
    raise Exception("message_to_json can't handle type: {}".format(type(object)))

def json_to_message(object):
    if type(object)==dict: #walk through dicts
        if "_packed_type" in object:
            if object["_packed_type"]=="jpegimage":
                x=bytes(object["data"],encoding='utf-8')
                x=base64.b64decode(x)
                x=np.frombuffer(x,dtype=np.uint8)
                return cv.imdecode(x,cv.IMREAD_COLOR)
            if object["_packed_type"]=="ndarray":
                return np.array(object["data"])
            else:
                raise Exception("json_to_message doesn't understand type {}".format(object["_packed_type"]))
        ret={}
        for key in object:
            ret[key]=json_to_message(object[key])
        return ret
    if type(object)==list: #walk through lists
        ret=[]
        for elem in object:
            ret.append(json_to_message(elem))
    return object


