sys.path.append('../yolov5_tool')
from yolov5_tool import yolov5_tool

class Theo_Chaser_Object_Tagger():
    def init(self):
        self.model_file="C:/Users/grybk/projects/yolov5/yolov5/runs/exp28/weights/last.pt"
        self.video_width=640
        self.video_height=480

    def tag_objects(self,video_frame):
        if self.yv5model==None:
            self.yv5model=yolov5_tool()
            self.yv5model.initialize(self.model_file)
        video_objects=self.yv5model.detect(video_frame)
        return video_objects

    def draw_bboxes(self,video_frame,video_objects):
        self.yv5model.draw_object_bboxes(video_frame,video_objects)
        return video_frame

    def get_obj_loc_width(self,face):
        #converts the start,stop notation into an array of points, center and wh
        centerx=(0.5*(face["startx"]+face["endx"])-self.video_width/2)/self.video_width
        centery=(0.5*(face["starty"]+face["endy"])-self.video_height/2)/self.video_height
        width=(-face["startx"]+face["endx"])/self.video_width
        height=(-face["starty"]+face["endy"])/self.video_height
        return np.array([centerx,centery]),np.array([width,height])
