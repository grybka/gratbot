#omjer memeroy
from underpinnings.id_to_name import id_to_name
import uuid
import logging
import face_recognition
import os,pathlib
import numpy as np

logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

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
        #face specific
        if "image_label" in features and features["image_label"]=="face" and "image" in features:
            subimage=features["image"]
            bbox=(0,0,subimage.shape[0],subimage.shape[1])
            face_encoding = face_recognition.face_encodings(subimage,[bbox])[0]
            self.known_face_encodings.append(face_encoding)
            self.known_face_encoding_ids.append(id)
        return id

    def get_object_features(self,object_id):
        ...

    def identify_object_from_features(self,features):
        #get scores of various objects based on features in dictionary
        #current plan:  the 'score' is a number with 0 being unlike, so i should use something like inverse chi-square of a fit

        #return ranked objects
        scores=WeightedList()
        if "position" in features:
            ...
        if "image_label" in features:
            ...
        if "image" in features:
            if "image_label" in features and features["image_label"]=="face":
                score_matches=self.get_score_image_face_only(features["image"])
                for key in score_matches:
                    scores.add_value(key,1./(0.4+score_matches[key]))
            else:
                ...
        return scores.to_list()


    def get_score_position(self,object,relative_position):
        #relative position is a vector [frontness,backness,leftness,rightness]
        ...

    def get_score_image_label(self,object,label):
        ...

    def get_score_image(self,object,subimage):
        ...

    def get_score_image_face_only(self,subimage):
        if len(self.known_face_encodings)==0:
            return {}
        bbox=(0,0,subimage.shape[0],subimage.shape[1])
        face_encodings = face_recognition.face_encodings(subimage,[bbox])
        #logger.debug("face encoding is {}".format(face_encodings))
        face_distances = face_recognition.face_distance(np.stack(self.known_face_encodings,axis=0), face_encodings)
        ret={}
        for i in range(len(self.known_face_encoding_ids)):
            ret[self.known_face_encoding_ids[i]]=face_distances[i]
            #Note, a typical cutoff is 0.6 here
        return ret
