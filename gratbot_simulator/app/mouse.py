from MouseModel import MouseWorld
import cv2
import numpy as np

world=MouseWorld()
world.add_boundary(3.0,3.0)
world.add_random_cheese()

#control_vector=np.array([0,0])
control_vector=[0,0]
#main loop
nsteps=200

#for step in range(nsteps):
while True:
#observation,reward=world.step([0.1,1.0])
    observation,reward=world.step(control_vector)
    world.render()
    key=cv2.waitKey(50)
#key=cv2.waitKey(0)
    #82 = forward
    #84 = backward
    #81 = left
    #83 = right
    if key==-1:
        x=1
        #noop
    elif key==82:
        control_vector[0]+=0.1
        if control_vector[0]>1:
            control_vector[0]=1
        print(control_vector)
    elif key==84:
        control_vector[0]-=0.1
        if control_vector[0]<-1:
            control_vector[0]=-1
        print(control_vector)
    elif key==81:
        control_vector[1]+=0.1
        if control_vector[1]>1:
            control_vector[1]=1
        print(control_vector)
    elif key==83:
        control_vector[1]-=0.1
        if control_vector[1]<-1:
            control_vector[1]=-1
        print(control_vector)
    elif key==113:
        print("Quit pressed")
        break
    else:
        print("key {}".format(key))
        break
    
#print("key {}".format(key))
#print("{} {}".format(world.mouse_pos[0],world.mouse_pos[1]))
