#omjer memeroy
from underpinnings.id_to_name import id_to_name
import uuid
import logging
import face_recognition
import os,pathlib
import numpy as np
import yaml
from underpinnings.FeatureEncoder import image_label_to_vector

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def load_object_info_file(fname="config/object_knowledge.yaml"):
    known_objects=[]
    with open(fname,'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        for elem in data:
            #give everything a unique id
            elem["id"]=uuid.uuid1()
    return data


class WeightedList: #a pairing of key, number
    def __init__(self):
        self.data={}

    def add_value(self,key,value):
        if key in self.data:
            self.data[key]+=value
        else:
            self.data[key]=value

    def to_list(self):
        ret=[]
        for key in self.data:
            ret.append( (self.data[key],key) )
        return sorted(ret)

class ObjectMemory:
    def __init__(self):
        self.objects={}

        #face recognition only
        self.known_face_encodings=[]
        self.known_face_encoding_ids=[]

    def load_objects(self,data):
        #load from data structure presumably stored in yaml
        for elem in data:
            logger.debug("loading face encoding for {}".format(elem["proper_name"]))
            object={"id": elem["id"]}
            face = face_recognition.load_image_file(elem["face_file"])
            the_encoding = face_recognition.face_encodings(face)[0]
            object["face_encoding"]=the_encoding
            object["proper_name"]=elem["proper_name"]
            object["image_label_vector"]=image_label_to_vector(elem["image_label"])
            self.objects[elem["id"]]=object
            self.known_face_encodings.append(the_encoding)
            self.known_face_encoding_ids.append(id)


    def load_face_encodings(self):
        facepath=os.path.join(os.getcwd(),"config","faces")
        face_files=os.listdir(facepath)
        #TODO define face_files
        for f in face_files:
            face = face_recognition.load_image_file(os.path.join(facepath,f))
            logger.debug("loading face file {}".format(f))
            #locations=face_recognition.face_locations(face)
            #logger.debug("face locations {}".format(locations))
            the_encoding = face_recognition.face_encodings(face)[0]
            #TODO tie name in here
            name=f.split(".")[0]
            id=self.instantiate_new_object({"face_encoding": the_encoding, "proper_name": name})
            self.known_face_encodings.append(the_encoding)
            self.known_face_encoding_ids.append(id)

    def instantiate_new_object(self,features):
        #if score is worse than some precondiction, call this
        #to create a new object in memory
        id=uuid.uuid1()
        self.objects[id]=features
        return id

    def get_object_features(self,object_id):
        ...

    ###
    def get_object_match_scores(self,features):
        scores=[]
        ids=[]
        for id,obj in self.objects.items():
            the_score=self.score_object_from_feature(features,obj)
            scores.append(the_score)
            ids.append(id)
        return ids,scores

    def score_object_from_feature(self,features,object):
        score=0
        n_elems=0
        #does this object tend to get this image label?
        if "image_label_vector" in features and "image_label_vector" in object:
            overlap=np.dot(features["image_label_vector"],object["image_label_vector"])
            score+=(2*(1-overlap)+0.1)**2
            #score+=1/(overlap+0.1)-1
            #dist=np.linalg.norm(features["image_label_vector"]-object["image_label_vector"])
            #score=score+1/(0.4+dist) #TODO should I make it more real log like
            n_elems=n_elems+1
        #for face specific things
        if "face_encoding" in features and "face_encoding" in object:
            dist= np.linalg.norm(features["face_encoding"] - object["face_encoding"])
            #score=score+1/(0.4+dist)
            score+=dist*dist/(0.6*0.6)
            n_elems=n_elems+1
        if "xywh" in features and "xywh" in object: #bounding box in frame
            a=features["xywh"]
            b=object["xywh"]
            score+=((a[0]-b[0])/(a[2]+b[2]))**2
            score+=((a[1]-b[1])/(a[3]+b[3]))**2
            n_elems+=2

        return score/n_elems

    def update_object_from_feature(self,features,object):
        #update image label
        if "image_label_vector" in features:
            image_label_persist=0.9
            if "image_label_vector" in object:
                vec=features["image_label_vector"]
                object["image_label_vector"]=object["image_label_vector"]*image_label_persist+(1-image_label_persist)*vec
            else:
                object["image_label_vector"]=vec
        #special for faces
        if "face_encoding" in features:
            logger.debug("updating face encoding")
            face_label_persist=0.99
            if "face_encoding" in object:
                object["face_encoding"]=object["face_encoding"]*face_label_persist+(1-face_label_persist)*features["face_encoding"]
            else:
                object["face_encoding"]=features["face_encoding"]
        if "xywh" in features: #bounding box in frame
            object["xywh"]=features["xywh"]


    def track_lost(self,object_id):
        obj=self.objects[object_id]
