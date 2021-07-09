import json

def dict_array_upend(mydict,key,elem):
    if key not in mydict:
        mydict[key]=[]
    mydict[key.append(elem)]

def load_sensor_log_file(fname):
    response=[]
    first_timestamp=0
    f=open(fname,'r')
    for line in f.readlines():
        dat=json.loads(line)
        if "timestamp" in dat:
            if first_timestamp==0:
                first_timestamp=dat["timestamp"]
            #timestamp=dat["timestamp"]-first_timestamp
        response.append(dat)
    return response,first_timestamp
