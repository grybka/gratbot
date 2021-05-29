from gyrii.behaviors.Behavior import *
from gyrii.behaviors.MotionBehaviors import *
from gyrii.behaviors.SensorPrintBehaviors import *
from gyrii.behaviors.ExploreBehavior import *
from gyrii.behaviors.TextCommandBehavior import *
from gyrii.behaviors.CalibrateMotionBehavior import *

def get_behavior():
    #myloop=GratbotBehavior_Series([PrintState("magnetometer/b_field"),GratbotBehavior_Wait(1.0)])
    #myloop=GratbotBehavior_Series([PrintState("lidar/lidar_scan"),GratbotBehavior_Wait(1.0)])
    #myloop=GratbotBehavior_Series([PrintState("latest_pose"),GratbotBehavior_Wait(1.0)])
    #myloop=GratbotBehavior_Series([GratbotBehavior_Wait(1.0)])
    #myloop.should_loop=True
    #myloop=GratbotBehavior_Series([GratbotBehavior_Wait(2.0),MotorFixedAmount(-0.4,'ahead')])
    #myloop.should_loop=True
    #myloop=GratbotBehavior_Series([GratbotBehavior_Wait(1.0),SaveFrame(),MoveAhead(-0.2)])
    #myloop2=GratbotBehavior_Series([GratbotBehavior_Wait(2.0),Turn(0.4)])
    #myloop2.should_loop=True
    #myloop=GratbotBehavior_Series([GratbotBehavior_Wait(10.0),myloop2])
    #myloop=ExploreBehavior()
    myloop=TextCommandBehavior()
    #myloop=RandomMotionTrackVisual()

    #MotorFixedAmount(-0.4,'ahead')])
    return myloop
