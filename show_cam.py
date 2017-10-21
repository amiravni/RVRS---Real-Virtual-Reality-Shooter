import cv2
import sys
import os
import time
import numpy as np
import serial   ### pip install serial, pip install pyserial
import struct
from ovrsdk import *  ### pip install python-ovrsdk
from Quaternion import Quat
import binascii
import cv2  #### DONOT: pip install opencv-python DO: sudo apt-get install python-opencv

serialExist = True
if serialExist:
  ser = serial.Serial('/dev/ttyACM0',baudrate=115200)
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

cam_left = cv2.VideoCapture(1)
cam_right = cv2.VideoCapture(0)
#cam_right = cam_left

cv2.namedWindow("left", cv2.WND_PROP_FULLSCREEN)
cv2.namedWindow("right", cv2.WND_PROP_FULLSCREEN)
#cv2.setWindowProperty("right", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
#cv2.setWindowProperty("left", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
send_screens = False
frameIdx = 0
left_canvas = np.zeros((1080,960,3)).astype('uint8')
right_canvas = np.zeros((1080,960,3)).astype('uint8')

yaw_lastStep = 0
pitch_lastStep = 0
roll_lastStep = 0
yaw_steps = 0

while True:
  ret1,left_img = cam_left.read()
  ret2,right_img = cam_right.read()
  #img = cv2.fliplr(img)
  if ret1 or ret2:
    resized_left_img = cv2.resize(left_img,(960,720))
    resized_right_img = cv2.resize(right_img,(960,720))

    left_canvas[180:900,:] = resized_left_img.astype('uint8')
    right_canvas[180:900,:] = resized_right_img.astype('uint8')
    cv2.imshow('left',left_canvas)
    cv2.imshow('right',right_canvas)
    cv2.waitKey(1)
    frameIdx += 1
    if frameIdx > 10 and  not send_screens:
      os.system('bash send-app-windows.sh')
      send_screens=True 

  ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())
  pose = ss.Predicted.Pose
  q = Quat([pose.Orientation.w,pose.Orientation.x,pose.Orientation.y,pose.Orientation.z])   # 	q.ra --> Pitch , q.dec --> Yaw , q.roll --> Roll
  #print q.ra, q.dec, q.roll

  # this part is true only for "pitch" of -90 to 90 degrees (The half dome infront of a person )
  steps_per_rev = 127
  yaw_newStep = -((q.dec/180)*steps_per_rev)
  if yaw_newStep > steps_per_rev:
    yaw_newStep = steps_per_rev
  if yaw_newStep < -steps_per_rev:
    yaw_newStep = -steps_per_rev
  #yaw_steps =#int(round(yaw_newStep - yaw_lastStep))
  yaw_lastStep  = yaw_steps
  yaw_steps = int(round(yaw_newStep))
  #yaw_lastStep = yaw_newStep

  ra_corrected = q.ra
  if q.ra > 180.0:
    ra_corrected = q.ra - 360.0

  pitch_newStep = (((ra_corrected)/180)*100)
  if pitch_newStep > 100:
    pitch_newStep = 100
  if pitch_newStep < -100:
    pitch_newStep = -100
  pitch_steps = int(round(pitch_newStep - pitch_lastStep))
  pitch_lastStep = pitch_newStep

  if yaw_steps != yaw_lastStep: # or pitch_steps != 0:
    print(q.dec,q.ra,yaw_steps,pitch_steps)
    #ser.write(struct.pack(2*'B',yaw_steps + 128,pitch_steps + 128))
    if serialExist:
		ser.write(struct.pack('BB',yaw_steps+128,10))
  time.sleep(0.0005)
  if (serialExist):
    recv = ser.read(1024).lstrip().rstrip()
    if len(recv) > 0:
      print recv


