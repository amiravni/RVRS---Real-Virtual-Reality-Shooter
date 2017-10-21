# -*- coding: utf-8 -*-
"""
Created on Mon Aug 29 19:55:48 2016

@author: PC-BRO

dictionary:
    theta0: 1950 --> 90deg , 21.6666 --> 1deg
    theta1: 1450 --> 90deg , 16.1111 --> 1deg
    theta2: -640 --> 40deg , -16 --> 1deg

"""

import numpy as np
import time
#import serial
import struct
import time
from ovrsdk import *
from Quaternion import Quat
import binascii

import cv2  ## to insall--> pip install opencv-python

#ser = serial.Serial('COM8',baudrate=115200)
#ser.timeout = 0.00

ovr_Initialize()
hmd = ovrHmd_Create(0)
hmdDesc = ovrHmdDesc()
ovrHmd_GetDesc(hmd, byref(hmdDesc))
print hmdDesc.ProductName
ovrHmd_StartSensor( \
  hmd,
  ovrSensorCap_Orientation |
  ovrSensorCap_YawCorrection,
  0
)

cam = cv2.VideoCapture(1)
yaw_lastStep = 0
pitch_lastStep = 0
roll_lastStep = 0
while True:

	ret,img = cam.read()
	if ret:
		cv2.imshow('img',img)
		cv2.waitKey(1)
	ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())
	pose = ss.Predicted.Pose
        #print pose.Orientation.w
        #print pose.Orientation.x
        #print pose.Orientation.y
        #print pose.Orientation.z

	q = Quat([pose.Orientation.w,pose.Orientation.x,pose.Orientation.y,pose.Orientation.z])   # q.ra --> Pitch , q.dec --> Yaw , q.roll --> Roll
#	print q.ra, q.dec, q.roll


	# this part is true only for "pitch" of -90 to 90 degrees (The half dome infront of a person )
	yaw_newStep = ((q.dec/180)*100)
 	if yaw_newStep > 100:
		yaw_newStep = 100
	if yaw_newStep < -100:
		yaw_newStep = -100	
	yaw_steps = int(round(yaw_newStep - yaw_lastStep))
	yaw_lastStep = yaw_newStep
	print yaw_newStep
#	if yaw.steps != 0:
#		print(q.dec,steps)
		#ser.write(struct.pack('B',yaw.steps + 128))
 	time.sleep(0.0005)
#	recv = ser.read(1024).lstrip().rstrip()
#	if len(recv) > 0:
#		print recv
