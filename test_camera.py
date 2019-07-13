import picamera
import numpy as np
import cv2

camera = picamera.PiCamera()              #Camera initialization
#xres=640
xres=320
#yres=480
yres=240
camera.resolution = (xres, yres)
camera.framerate = 6
image=np.empty((xres*yres*3,),dtype=np.uint8)
camera.capture(image,'bgr')
image=image.reshape((yres,xres,3))
#rawCapture = PiRGBArray(camera, size=(xres, yres))
cv2.imwrite("{}.png".format(time.time()),image)

