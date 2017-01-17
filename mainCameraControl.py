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
import serial
import struct
import time
from ovrsdk import *
from Quaternion import Quat
import binascii

import cv2

ser = serial.Serial('COM8',baudrate=115200)
ser.timeout = 0.00

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
lastStep = 0
while True:

	ret,img = cam.read()
	if ret:
		cv2.imshow('img',img)
		cv2.waitKey(1)
	ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())
	pose = ss.Predicted.Pose
	q = Quat([pose.Orientation.w,pose.Orientation.x,pose.Orientation.y,pose.Orientation.z])
#	print q.ra, q.dec, q.roll

#	encoderCommand = [int(q.dec*theta2enc[0]),0*theta2enc[1],0*theta2enc[2],0,0,0]
#	print encoderCommand
	newStep = int((q.dec/180)*100)
	steps = newStep - lastStep
	lastStep = newStep
 	if steps > 100:
		steps = 100
	if steps < -100:
		steps = -100
	if steps != 0:
#		print(q.dec,steps)
		ser.write(struct.pack('B',steps + 128))
 	time.sleep(0.0005)
#	recv = ser.read(1024).lstrip().rstrip()
#	if len(recv) > 0:
#		print recv