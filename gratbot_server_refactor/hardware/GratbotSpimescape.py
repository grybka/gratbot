import logging

_all_gratbot_spimescapes={}

class GratbotSpimescape:
    def set(self,endpoint,value):
        raise Exception("set unhandled")
    def get(self,endpoint):
        raise Exception("get unhandled")
    def expose_endpoints(self,endpoint):
        #return a list of viable get and set endpoints
        return [ [],[] ]
    def get_update(self,last_time): #request changes since time as dictionary
        return {}
    def shut_down(self):
        return


def create_hardware(datastruct):
    hardware_dat={}
    for x in datastruct.keys():
        logging.info("hardware creating {}".format(x))
        if datastruct[x]["type"] in _all_gratbot_spimescapes:
            hardware_dat[x]=_all_gratbot_spimescapes[datastruct[x]["type"]](datastruct[x],hardware_dat)
            hardware_dat[x].type=datastruct[x]["type"]
        else:
            logging.warning("Unrecognized hardware {}".format(x))
        #hardware_dat[x]=create_hardware_item(datastruct[x])
    return hardware_dat
