#!/usr/bin/python3

import cv2

from picamera2 import Picamera2
from libcamera import Transform

# Grab images as numpy arrays and leave everything else to OpenCV.

cv2.startWindowThread()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (800,600)},transform=Transform(hflip=True,vflip=True)))
#picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (800,600)}))
#picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)}))
#picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (4608,2592)}))
picam2.start()


print(dir(picam2))
#print(picam2.__dict__)
#print(picam2.camera_config)
#print(type(picam2.camera_config))
#print(picam2.camera_config["main"])
#print(picam2.camera_config["main"]["size"])
#print(picam2.camera_config["main"]["size"][0])
#print(picam2.camera_config["main"]["size"][1])

while True:
    im = picam2.capture_array()
    cv2.imshow("Camera", im)
    cv2.waitKey(1)
