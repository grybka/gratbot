#starts a docker image that will be able to display to an osx screen
#you'll need to run "workon workon OpenCV-master-py3" after starting the container to access opencv
#####THIS IS FOR APPLE
IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
xhost +$IP
local_dir=`pwd`/app
echo $local_dir
docker run --name grat_learning --rm -v $local_dir:/app -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$IP:0 -p 5000:5000 -p 8888:8888 -it  grat_learning:latest /bin/bash
