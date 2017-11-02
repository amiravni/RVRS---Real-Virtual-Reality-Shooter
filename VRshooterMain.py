
import sys
import os
import time
import numpy as np
import serial   ### pip install serial, pip install pyserial
import struct
import dlib
#Threads
from thread import start_new_thread
from threading import Lock

from ovrsdk import *  ### pip install python-ovrsdk
from Quaternion import Quat
import binascii
import cv2  #### DONOT: pip install opencv-python DO: sudo apt-get install python-opencv
import face_recognition # https://github.com/ageitgey/face_recognition

left_camera_image = -1
right_camera_image = []
face_locations_global = []
face_names_global = []
grab_data = True
MAX_FACES = 10
serialExist = False

frame_lock = Lock()

def cameras_handle():
	global left_camera_image
	global face_locations_global
	global face_names_global
	global grab_data
	cam_left = cv2.VideoCapture(0)
	cam_right = cv2.VideoCapture(1)

	cv2.namedWindow("left", cv2.WND_PROP_FULLSCREEN)
	cv2.namedWindow("right", cv2.WND_PROP_FULLSCREEN)

	left_canvas = np.zeros((1080,960,3)).astype('uint8')
	right_canvas = np.zeros((1080,960,3)).astype('uint8')
	scaling_factor = 1
	send_screens = False
	frameIdx = 0
	tracker = []
	# Initial co-ordinates of the object to be tracked 
	# Create the tracker object
	for fff in range(0,MAX_FACES):
		tracker.append(dlib.correlation_tracker())
	
	while True:
		ret1,left_img = cam_left.read()
		ret2,right_img = cam_right.read()
		if ret1 or ret2:
			resized_left_img = cv2.resize(left_img,(960,720))
			resized_right_img = cv2.resize(right_img,(960,720))
			downscale = 1/float(scaling_factor)
			small_left_img = cv2.resize(resized_left_img, (0, 0), fx=downscale, fy=downscale)
			frameIdx = frameIdx + 1
			frame_lock.acquire()
			try:
				left_camera_image = small_left_img		
			finally:
				frame_lock.release()				
			
			if grab_data == True:
				frame_lock.acquire()
				try:
					left_camera_image = small_left_img

					faces_positions_local = face_locations_global
					faces_names_local = face_names_global	
					grab_data = False										
				finally:
					frame_lock.release()
				# Provide the tracker the initial position of the object
				if len(faces_positions_local) > 0:
					for fff in range(0,len(faces_positions_local)):
						tracker[fff].start_track(small_left_img, dlib.rectangle(top = faces_positions_local[fff][0],right = faces_positions_local[fff][1],bottom= faces_positions_local[fff][2],left = faces_positions_local[fff][3]))

			# Update the tracker  
			if len(faces_positions_local) > 0:
				for fff in range(0,len(faces_positions_local)):
					tracker[fff].update(small_left_img)
					rect = tracker[fff].get_position()
					faces_positions_local[fff] = [int(rect.top()),int(rect.right()),int(rect.bottom()),int(rect.left())]
			#pt1 = (int(rect.left()), int(rect.top()))
			#pt2 = (int(rect.right()), int(rect.bottom()))
			#cv2.rectangle(img, pt1, pt2, (255, 255, 255), 3)       	 	
			
			for (top, right, bottom, left), name in zip(faces_positions_local, faces_names_local):
				# Scale back up face locations since the frame we detected in was scaled to 1/4 size
				top *= scaling_factor
				right *= scaling_factor
				bottom *= scaling_factor
				left *= scaling_factor

				# Draw a box around the face
				cv2.rectangle(resized_left_img, (left, top), (right, bottom), (0, 0, 255), 2)

				# Draw a label with a name below the face
				cv2.rectangle(resized_left_img, (left, bottom - 0), (right, bottom), (0, 0, 255), -1)
				font = cv2.FONT_HERSHEY_DUPLEX
				cv2.putText(resized_left_img, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)


			left_canvas[180:900,:] = resized_left_img.astype('uint8')
			right_canvas[180:900,:] = resized_right_img.astype('uint8')
			cv2.imshow('left',left_canvas)
			cv2.imshow('right',right_canvas)
			cv2.waitKey(1)
			if frameIdx > 10 and  not send_screens:
				os.system('bash send-app-windows.sh')
				send_screens=True 	
		

def face_recognition_handle():
	global left_camera_image
	global face_locations_global
	global face_names_global
	global grab_data
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

	while True:		
		#tic = time.time()    
		time.sleep(0.95)
		frame_lock.acquire()
		try:
			frame_rec = left_camera_image
		finally:
			frame_lock.release()		
		if type(frame_rec) is int:
			continue
		else:			
			
			# Find all the faces and face encodings in the current frame of video
			'''
			time1 = time.time()
			face_locations = face_recognition.face_locations(frame_rec, number_of_times_to_upsample=0, model="hog")
			print "1) "+str(time.time() - time1)
			time1 = time.time()
			face_encodings = face_recognition.face_encodings(frame_rec, face_locations)
			print "2) "+str(time.time() - time1)
			'''
			face_locations = face_recognition.face_locations(frame_rec, number_of_times_to_upsample=0, model="hog")
			face_encodings = face_recognition.face_encodings(frame_rec, face_locations)
			
			face_names = []
			faces_cnt = 0 
			for face_encoding in face_encodings:
					# See if the face is a match for the known face(s)
					match = face_recognition.compare_faces([alon_face_encoding ,amir_face_encoding,aviv_face_encoding], face_encoding)
					faces_cnt = faces_cnt+ 1

					#faces_identifications[faces_cnt] = -1
					name = "Unknown"
					if match[0]:
					#	faces_identifications[faces_cnt] = 0
						name = "alon"
					if match[1]:
					#	faces_identifications[faces_cnt] = 1
						name = "amir"
					if match[2]:
					#	faces_identifications[faces_cnt] = 2
						name = "aviv"
					face_names.append(name)

			frame_lock.acquire()
			try:
				face_names_global = face_names
				face_locations_global = face_locations
				grab_data = True
			finally:
				frame_lock.release()

def oculus_handle():
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
		
	yaw_lastStep = 0
	pitch_lastStep = 0
	roll_lastStep = 0
	yaw_steps = 0

	while True:
		ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())
		pose = ss.Predicted.Pose
		q = Quat([pose.Orientation.w,pose.Orientation.x,pose.Orientation.y,pose.Orientation.z])   # 	q.ra --> Pitch , q.dec --> Yaw , q.roll --> Roll
		print q.ra, q.dec, q.roll

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


if __name__ == '__main__':
    
	start_new_thread(face_recognition_handle,())
	start_new_thread(cameras_handle,())
	start_new_thread(oculus_handle,())

	while True:
		time.sleep(0.01)
