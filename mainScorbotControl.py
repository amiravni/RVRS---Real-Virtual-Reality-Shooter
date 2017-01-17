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
from platform_communication_handler import platformCommunicationHandler as pch
import time
from ovrsdk import *
from Quaternion import Quat

import cv2



draw_skip = 1
d1 = 0.22
d2 = 0.22
#theta2enc = [21.6666, 16.1111, -16, 0 , 0, 0]
theta2enc = [21.6666, 16.6666, -16.666, 0 , 0, 0]
l_gripper_and_pen = [0.25,0]
l_initY = 0
l_initZ = 0
l_org_to_screen =  0.51
counter_contour = 0
def xyz2angles(x,y,z):
    global d1,d2,theta2enc,l_gripper_and_pen
    theta0 = np.arctan2(z,x)
    theta0_deg = np.rad2deg(theta0)
    x_corr = x/np.cos(theta0)   
    x0 = float(x_corr - l_gripper_and_pen[0])
    y0 = float(y - l_gripper_and_pen[1])
    L = float(np.sqrt(x0*x0 + y0*y0))    
    gamma = np.arccos( -( (L*L - d1*d1 - d2*d2)/(2.0*d1*d2) ) ) 
    alpha = np.arcsin( (d2/L)*np.sin(gamma) )
    theta1 = alpha + np.arctan2(y0,x0)     
    theta1_deg = 90 - np.rad2deg(theta1)
    x1 = d1*np.cos(theta1)
    y1 = d1*np.sin(theta1)
    theta2 = np.arctan2((y0-y1),(x0-x1))
    theta2_deg = -np.rad2deg(theta2)
    #print x,y,z,x_corr,theta0_deg,theta1_deg,theta2_deg
    #print x,y,x0,y0,L,gamma,alpha,theta1,theta1_deg,theta2,theta2_deg,x1,y1
    return [theta0_deg*theta2enc[0],theta1_deg*theta2enc[1],theta2_deg*theta2enc[2],0,0,0]


#def plotLine()

encoder_tmp = [0]*6
encoder_abs_pos_array = [0]*6
encoder_init_positions_array = [0]*6
encoder_poly_array = [0]*37
theta_array = np.zeros([101,6])
colorPitch = [203,380]
colorNow = 1

platform_controller = pch('COM7')
time.sleep(3)
print "Init Done!"
# {false, false, true, false, true, true}

encoder_init_positions_array[0] = 1
encoder_init_positions_array[2] = 1
encoder_init_positions_array[4] = 1
encoder_init_positions_array[5] = 1  

 
platform_controller.give_commands_go_home(encoder_init_positions_array)
platform_controller.read_from_serial_blocking(1024)

time.sleep(5)
encoder_abs_pos_array[3] = colorPitch[colorNow]
platform_controller.give_commands(encoder_abs_pos_array)

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

while True:

	ret,img = cam.read()
	if ret:
		cv2.imshow('img',img)
		cv2.waitKey(1)
	ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())
	pose = ss.Predicted.Pose
	q = Quat([pose.Orientation.w,pose.Orientation.x,pose.Orientation.y,pose.Orientation.z])
	print q.ra, q.dec, q.roll

	encoderCommand = [int(q.dec*theta2enc[0]),0*theta2enc[1],0*theta2enc[2],0,0,0]
	print encoderCommand
	platform_controller.give_commands(encoderCommand)
 	time.sleep(0.25)
