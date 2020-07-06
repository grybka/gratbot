import sys
import numpy as np
import cv2 as cv
import logging
import cvlib
from cvlib.object_detection import draw_bbox

from GratbotVideoConnectionUV4L import GratbotVideoConnectionUV4L
sys.path.append('../gratbot_client')
from GratbotComms import GratbotComms
from GratbotClient import GratbotClient
from GratbotBehaviors import WaveAtFace
from GratbotBehaviors import HeadTrackFace
from GratbotBehaviors import MoveAndTrackObjects
from GratbotBehaviors import JustSaveObjectPos

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.INFO)
#    level=logging.DEBUG)

# connect to bot controls
logging.info("Connecting to Gratbot")
gratbot_comms = GratbotComms("10.0.0.5", 9999)


# connect to camera
cv.namedWindow("preview")
cv.moveWindow("preview", 0, 0)
logging.info("Connecting to Gratbot")
video = GratbotVideoConnectionUV4L("http://10.0.0.5:8080/stream/video.mjpeg")
frame_width = video.cap.get(cv.CAP_PROP_FRAME_WIDTH)
frame_height = video.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
print("video resolution: {} x {} ".format(frame_width, frame_height))

forward_speed=0.0
#on_behavior= WaveAtFace(gratbot_comms)
#on_behavior = HeadTrackFace(gratbot_comms)
gratbot_comms.set_intention( ["camera_x","position","SET" ], 0 )
gratbot_comms.set_intention( ["camera_y","position","SET" ], 10 )
#on_behavior = MoveAndTrackObjects(gratbot_comms)
on_behavior = JustSaveObjectPos(gratbot_comms)
#on_behavior = None

# Video Display loop here
try:
    cycle_counter = 0
    while True:
        cycle_counter += 1
        myframe, mytime = video.read()
        #myframe = cv.resize(myframe, None, fx=4, fy=4)
        #objframe = imagefinder.get_processed_frame()
        faces,confidences=cvlib.detect_face(myframe)
        video_objects=[]
        if len(faces)>0: #if you see a face, that's the most interesting
            #TODO probably rerank faces by confidence
            for i in range(len(faces)):
                face=faces[i]
                video_objects.append({
                    "confidence": confidences[i],
                    "label": "face",
                    "startx": face[0],
                    "starty": face[1],
                    "endx": face[2],
                    "endy": face[3]
                })
                confidence=confidences[i]
                (startx,starty)=face[0],face[1]
                (endx,endy)=face[2],face[3]
                cv.rectangle(myframe,(startx,starty),(endx,endy),(0,255,0),2)
                text = "Face {:.2f}%".format(confidence * 100)
                Y = starty - 10 if starty - 10 > 10 else starty + 10
                # write confidence percentage on top of face rectangle
                cv.putText(myframe, text, (startx,Y), cv.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0), 2)
        else: #otherwise look for objects
            bbox,label,conf=cvlib.detect_common_objects(myframe,confidence=0.4,model='yolov3-tiny')
            draw_bbox(myframe,bbox,label,conf,write_conf=True)
            for i in range(len(label)):
                box=bbox[i]
                video_objects.append({
                    "confidence": conf[i],
                    "label": label[i],
                    "startx": box[0],
                    "starty": box[1],
                    "endx": box[2],
                    "endy": box[3]
                })
        cv.imshow("preview", myframe)
#        objframe = []
#        if len(objframe) > 0:
#            objframe = cv.resize(objframe, None, fx=4, fy=4)
#cv.putText(objframe,"Dist: {0:.3g} cm".format(gratbot_comms.ultra),(320,200),cv.FONT_HERSHEY_SIMPLEX,1.5,(0,255,0))
#            cv.imshow("object_finder", objframe)
        key = cv.waitKey(10)
        if on_behavior is not None:
            on_behavior.act(video_objects)
        if key!=-1:
            print("{} pressed".format(key))
            if key==97: #a Key
                gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 1)
                gratbot_comms.set_intention( [ "leg_controller","left_speed", "SET" ], 0)
                gratbot_comms.set_intention( [ "leg_controller","right_speed", "SET" ], 0)
            if key==44: #w Key in dvorak
                forward_speed=np.clip(forward_speed+0.1,-1,1)
                print("Speed {}".format(forward_speed))
                gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 1)
                gratbot_comms.set_intention( [ "leg_controller","left_speed", "SET" ], forward_speed)
                gratbot_comms.set_intention( [ "leg_controller","right_speed", "SET" ], forward_speed)
            if key==111: #s Key idvorake
                forward_speed=np.clip(forward_speed-0.1,-1,1)
                print("Speed {}".format(forward_speed))
                gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 1)
                gratbot_comms.set_intention( [ "leg_controller","left_speed", "SET" ], forward_speed)
                gratbot_comms.set_intention( [ "leg_controller","right_speed", "SET" ], forward_speed)
            if key==39: #q Key idvorake
                gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 0)
                forward_speed=0
                gratbot_comms.set_intention( [ "leg_controller","left_speed", "SET" ], forward_speed)
                gratbot_comms.set_intention( [ "leg_controller","right_speed", "SET" ], forward_speed)
# if cycle_counter%30==0:
#print("object fps {}".format(imagefinder.get_fps()))
        # send commands (could be different thread)

except KeyboardInterrupt:
    gratbot_comms.set_intention( [ "leg_controller","on_off", "SET" ], 0)
    logging.warning("Keyboard Exception Program Ended, exiting")
    behavior_thread_should_quit = True
finally:
    video.stop()
    #controller.close()
    gratbot_comms.stop()
