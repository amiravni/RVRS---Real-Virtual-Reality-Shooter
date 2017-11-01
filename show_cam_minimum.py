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
import face_recognition # https://github.com/ageitgey/face_recognition

serialExist = False
if serialExist:
  ser = serial.Serial('/dev/ttyACM0',baudrate=115200)
  ser.timeout = 0.00

'''
res = ovr_Initialize()
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
'''

amir_image = face_recognition.load_image_file("./examples/amir.jpg")
amir_face_encoding = face_recognition.face_encodings(amir_image)[0]
alon_image = face_recognition.load_image_file("./examples/alon.jpg")
alon_face_encoding = face_recognition.face_encodings(alon_image)[0]
aviv_image = face_recognition.load_image_file("./examples/aviv.jpg")
aviv_face_encoding = face_recognition.face_encodings(aviv_image)[0]

# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True


cam_left = cv2.VideoCapture(2)
cam_right = cv2.VideoCapture(1)
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
scaling_factor = 1
time_now = time.time()
time2process = 1

while True:
	ret1,left_img = cam_left.read()
	ret2,right_img = cam_right.read()
	#img = cv2.fliplr(img)
	if ret1 or ret2:
		resized_left_img = cv2.resize(left_img,(960,720))
		resized_right_img = cv2.resize(right_img,(960,720))

		small_left_img = cv2.resize(resized_left_img, (0, 0), fx=1/scaling_factor, fy=1/scaling_factor)
		#small_right_img = cv2.resize(resized_right_img, (0, 0), fx=1, fy=1)
		time_passed = time.time() - time_now
		process_this_frame = False
		if time_passed > time2process:
			process_this_frame = True
			time_now = time.time()
	
		if process_this_frame:
			# Find all the faces and face encodings in the current frame of video
			face_locations = face_recognition.face_locations(small_left_img)
			face_encodings = face_recognition.face_encodings(small_left_img, face_locations)

			face_names = []
			for face_encoding in face_encodings:
					# See if the face is a match for the known face(s)
					match = face_recognition.compare_faces([alon_face_encoding ,amir_face_encoding,aviv_face_encoding], face_encoding)
					name = "Unknown"

					if match[0]:
						name = "alon"
					if match[1]:
						name = "amir"
					if match[2]:
						name = "aviv"
					face_names.append(name)

		# Display the results
		for (top, right, bottom, left), name in zip(face_locations, face_names):
			# Scale back up face locations since the frame we detected in was scaled to 1/4 size
			top *= scaling_factor
			right *= scaling_factor
			bottom *= scaling_factor
			left *= scaling_factor

			# Draw a box around the face
			cv2.rectangle(resized_left_img, (left, top), (right, bottom), (0, 0, 255), 2)

			# Draw a label with a name below the face
			cv2.rectangle(resized_left_img, (left, bottom - 35), (right, bottom), (0, 0, 255), -1)
			font = cv2.FONT_HERSHEY_DUPLEX
			cv2.putText(resized_left_img, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)



		left_canvas[180:900,:] = resized_left_img.astype('uint8')
		right_canvas[180:900,:] = resized_right_img.astype('uint8')
		cv2.imshow('left',left_canvas)
		cv2.imshow('right',right_canvas)
		cv2.waitKey(1)
		frameIdx += 1
		if frameIdx > 10 and  not send_screens:
		  #os.system('bash send-app-windows.sh')
		  send_screens=True 
'''
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

'''
