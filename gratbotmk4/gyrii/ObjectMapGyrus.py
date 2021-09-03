
from Gyrus import ThreadedGyrus
import logging
import uuid
from uncertainties import ufloat,umath
from underpinnings.id_to_name import id_to_name
logger=logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

#records where objects are relative to me

class ObjectMapGyrusObject:
    def __init__(self,position=np.zeros(3),track_id=None):
        self.position=position #BayesianArray of x,y,z
        self.id=uuid.uuid1() #or tie to tracker id?

    def update_position(self,position):
        self.position=self.position.updated(position)

    def update_linear_motion(self,my_motion):
        #my_motion should be a BayesianArray
        self.position-=my_motion

    def update_turn(self,rotmat):
        xyz=self.position.get_as_ufloat(self)
        new_xyz=np.matmul(rot_mat,xyz)
        self.position=BayesianArray.from_ufloats(new_xyz)


class ObjectMapGyrus(ThreadedGyrus):
    def __init__(self,broker,display=None):
        super().__init__(broker)
        self.display=display

        #the objects in my map
        self.objects=[]

        #self.focal_length=640/(2*np.tan(np.pi*73.5/2/180)) #in pixels
        self.focal_length_x=640/(2*np.tan(np.pi*73.5/2/180)) #in fraction of image
        self.focal_length_y=640/(2*np.tan(np.pi*73.5/2/180)) #in fraction of image
        #tracking pose offsets
        self.last_used_heading=np.array([0,0,0])
        self.z_gyro_index=0
        self.y_gyro_index=2
        self.x_gyro_index=1

        self.score_cutoff=6.0

    def get_keys(self):
        return ["tracks","packets","clock_pulse"]

    def get_name(self):
        return "TrackerGyrus"

    def update_with_turn(self,turn,turn_unc):
        turn=ufloat(-turn_mag,turn_unc)
        rot_mat=np.array([ [umath.cos(turn),umath.sin(turn),0],[-umath.sin(turn),umath.cos(turn),0],[0,0,1]])
        for obj in objects:
            obj.update_turn(rot_mat)

    def update_from_track(self,track):
        #figure out if this track is already an object I know or not
        #   - keep a dict of trackid -> object id mapping
        #   - failing that, if its in roughtly the same location with the same label, assign it
        #   - failing that, we have to decide if it is new or spurious
        #Also more importantly; how to deal with an object not where I think it should be?
        #TODO I should hash this somehow, otherwise it could be slow
        position=self.convert_track_pos_to_xyz(track)
        best_score=np.inf
        best_obj=None
        for obj in self.objects:
            score=self.is_this_that(track,obj)
            if score<best_score:
                best_score=score
                best_obj=obj
        if best_score<self.score_cutoff:
            ... #old object
            obj.update_position(position)
        else:
            #new object
            self.objects.append(ObjectMapGyrusObject(position=position,track_id=track["id"],label=track["label"]))

    def convert_track_pos_to_xyz(self,track):
        bbox=track["bbox_array"]
        x_extent=(bbox[1]-bbox[0])/400
        y_extent=(bbox[3]-bbox[2])/400
        track_center=track["center"]
        track_center_unc=track["center_uncertainty"]
        #TODO do I include extent here?
        angle_x=ufloat((track_center[0]-0.5)/self.focal_length_x,(x_extent+track_center_unc[0])/self.focal_length_x)
        angle_y=ufloat((track_center[1]-0.5)/self.focal_length_1,(y_extent+track_center_unc[1])/self.focal_length_1)
        dist=ufloat(track_center[2],track_center_unc[2]
        #convert to xyz
        x=dist*angle_x
        y=dist*angle_y
        z=dist*np.cos(angle_x)*np.cos(angle_y)
        return BayesianArray.from_ufloats([x,y,z])

    def is_this_that(self,track,obj):
        #returns the likelyhood that an track actually corresponds to an object in my memory
        #location part
        center=self.convert_track_pos_to_xyz(track)
        chisq=obj.position.chi_square_from_pose(center)
        return chisq

    def read_message(self,message):
        #I could ask it where to find something?
        if 'clock_pulse' in message:
            self.draw_object_map()
        if 'tracks' in message:
            for track in message["tracks"]:
                self.update_from_track(track)
        if 'packets' in message: #rotation etc
            #I'm really only interested in the last measurement
            packet=message['packets'][-1]
            next_heading=np.array(packet["local_rotation"])
            accel=np.array(packet["acceleration"])
            delta_heading=next_heading-self.last_used_heading
            #I'm assuming it only turns
            mag_accel=np.linalg.norm(accel)
            cos_angle=accel[1]/mag_accel
            sin_angle=accel[2]/mag_accel
            turn_mag=delta_heading[self.z_gyro_index]*cos_angle-delta_heading[self.y_gyro_index]*sin_angle #in radians
            #inteperet this as a pose offset
            self.update_with_turn(turn,turn_unc*turn)
            self.last_used_heading=next_heading
