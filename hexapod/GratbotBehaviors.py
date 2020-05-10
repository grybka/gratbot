# Behaviors for hexapod


class GratbotBehavior:
    def __init__(self, comms):
        self.comms = comms
    def act(self,video_objects):
        return

class WaveAtFace(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
        self.waving=False
    def act(self,video_objects):
        wave_controller="wave_paw"
        idle_controller="legs_idle"
        if self.waving==False and "faces" in video_objects:
            self.waving=True
            self.comms.set_intention( [idle_controller,"on_off","SET" ], 0 )
            self.comms.set_intention( [wave_controller,"on_off","SET" ], 1 )
            self.comms.set_intention( [wave_controller,"left_speed","SET" ], 1 )
            self.comms.set_intention( [wave_controller,"right_speed","SET" ], 1 )
        elif self.waving==True and "faces" not in video_objects:
            self.waving=False
            self.comms.set_intention( [idle_controller,"on_off","SET" ], 1 )
            self.comms.set_intention( [wave_controller,"on_off","SET" ], 0 )


class HeadTrackFace(GratbotBehavior):
    def __init__(self, comms):
        super().__init__(comms)
    def act(self, video_objects):
        video_height=480
        video_width=640
        pixel_to_servo=1.0
        if "faces" in video_objects:
            face=video_objects["faces"][0]
            centerx=face["startx"]+face["endx"]
            centery=face["starty"]+face["endy"]
            dx=video_width-centerx #how does it know the size
            dy=video_height-centery
            dx_servo=dx*pixel_to_servo
            dy_servo=dy*pixel_to_servo
            self.comms.set_intention( ["camera_x","position_delta","SET" ], dx_servo )
            self.comms.set_intention( ["camera_y","position_delta","SET" ], dy_servo )
        else
            self.comms.set_intention( ["camera_x","position","SET" ], 0 )
            self.comms.set_intention( ["camera_y","position","SET" ], 0 )
