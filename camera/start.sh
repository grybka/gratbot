#starts a docker image that will be able to display to an osx screen
#you'll need to run "workon workon OpenCV-master-py3" after starting the container to access opencv
#####THIS IS FOR APPLE
#IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
#xhost +$IP
#docker run --name gratbot_camera --rm -v /Users/grybka/projects/gratbot/camera/app:/app -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$IP:0 -p 5000:5000 -p 8888:8888 -it  grat_opencv:latest /bin/bash
#docker run --name gratbot_camera --rm -v $myapp:/app -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$IP:0 -p 5000:5000 -p 8888:8888 --privileged -v /dev/input/js0:/dev/input/js0 -it  grat_opencv:latest /bin/bash
#docker run --name gratbot_camera --rm -v $myapp:/app -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$IP:0 -p 5000:5000 -p 8888:8888 --device=/dev/input/js0 -v /sys/class/leds/xpad0/brightness:/sys/class/leds/xpad0/brightness  -it  grat_opencv:latest /bin/bash #THIS IS FOR THE MAC
####THIS IS FOR THE PC
xhost +local:root
myapp=$PWD/app
docker run --name gratbot_camera --rm -v $myapp:/app -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --device=/dev/input/js0 -v /sys/class/leds/xpad0/brightness:/sys/class/leds/xpad0/brightness --ipc=host  -it  grat_opencv:latest /bin/bash



#docker run --name gratbot_camera --rm -v $myapp:/app -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$IP:0 -p 5000:5000 -p 8888:8888 -it  grat_opencv:latest /bin/bash

#docker run --name gratbot_camera --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v /Users/grybka/projects/gratbot/camera/app:/app -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$IP:0 -p 5000:5000 -p 8888:8888 -it spmallick/opencv-docker:opencv /bin/bash
