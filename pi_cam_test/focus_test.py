#!/usr/bin/python3

import os

import cv2

from picamera2 import Picamera2
from libcamera import Transform
from libcamera import controls

# Setting HDR to OFF
os.system("v4l2-ctl --set-ctrl wide_dynamic_range=0 -d /dev/v4l-subdev0")

cv2.startWindowThread()

picam2 = Picamera2()

picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (800,600)},transform=Transform(hflip=True,vflip=True)))

#picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)},transform=Transform(hflip=True,vflip=True)))

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

def keyboard_command(wait_key_in):
    picam2.stop()
    if wait_key_in == ord('1'):
        print("Continuous Focus")
        picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        #print(picam2.camera_config)
        print(picam2.__dict__)
    if wait_key_in == ord('2'):
        print("Manual Infinty Focus")
        picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 0.0})
        print(picam2.camera_config)
    if wait_key_in == ord('3'):
        print("Continuous Fast Focus")
        picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous, "AfSpeed": controls.AfSpeedEnum.Fast})
        print(picam2.camera_config)
    if wait_key_in == ord('4'):
        print("Setting HDR to ON")
        os.system("v4l2-ctl --set-ctrl wide_dynamic_range=1 -d /dev/v4l-subdev0")
    if wait_key_in == ord('5'):
        print("Setting HDR to OFF")
        os.system("v4l2-ctl --set-ctrl wide_dynamic_range=0 -d /dev/v4l-subdev0")
    picam2.start()

while True:
    im = picam2.capture_array()
    cv2.imshow("Camera", im)
    # Read keyboard
    wait_key_in = cv2.waitKey(10) & 0xFF
    if keyboard_command(wait_key_in) == 0:
        break;






