
import sys
import os
import time
import numpy as np
import serial   ### pip install serial, pip install pyserial
import struct
import dlib
import copy

#Threads
from thread import start_new_thread
from threading import Lock

#from matplotlib import pyplot as plt

from ovrsdk import *  ### pip install python-ovrsdk
from Quaternion import Quat
import binascii
import cv2  #### DONOT: pip install opencv-python DO: sudo apt-get install python-opencv
import face_recognition # https://github.com/ageitgey/face_recognition

NUMOFSCREENS = 2
MAX_FACES = 10
serialExist = True

wanted_dir = "./examples/wanted/"
boring_dir = "./examples/boring/"
camera_L = 1
camera_R = 0
ArduinoCOM = '/dev/ttyACM0'
factor_LR = 0.6
factor_UD = 0.1
factor_line = 0.2
DRAW_RIGHT_EYE = True
cy_sub = 540
cx_sub = 960
dx_sub = 150
dy_sub = 100
tmp_dy_sub = 80
tmp_dx_sub = 0

left_camera_image = -1
right_camera_image = -1
face_locations_global = []
face_names_global = []
face_interest_global = []
face_landmarks_list_global = []
wanted_list_global = []
tracking_frame_global = []
wanted_sz = 100
	
grab_data = True
track_data = False
dx = 0
dy = 0
frame_lock = Lock()

def cameras_handle():
	global wanted_sz
	global left_camera_image
	global right_camera_image	
	global face_locations_global
	global face_names_global
	global face_landmarks_list_global
	global face_interest_global
	global wanted_list_global
	global tracking_frame_global
	global grab_data
	global track_data
	global dx
	global dy
	cam_left = cv2.VideoCapture(camera_L)
	cam_right = cv2.VideoCapture(camera_R)
	
	cam_left.set(3,1920)
	cam_left.set(4,1080)
	cam_right.set(3,1920)
	cam_right.set(4,1080)	
	
	cv2.namedWindow("left", cv2.WND_PROP_FULLSCREEN)
	cv2.namedWindow("right", cv2.WND_PROP_FULLSCREEN)
	cv2.namedWindow("left_comp", cv2.WND_PROP_FULLSCREEN)
	cv2.namedWindow("right_comp", cv2.WND_PROP_FULLSCREEN)

	left_canvas = np.zeros((1080,960,3)).astype('uint8')
	right_canvas = np.zeros((1080,960,3)).astype('uint8')
	scaling_factor = 1
	scaling_factor_tracker = 1	
	send_screens = False
	frameIdx = 0
	cameras_alligened = False
	tracker = []
	wanted_list_local = []
	time1 = time.time()
	time_g = time.time()
	crop = [270,1080-270,480,1920-480] #top bottom left right
	
	
	while True:
		colorLine = (255, 0, 0)
		ret1,left_img = cam_left.read()
		ret2,right_img = cam_right.read()
		if ret1 and ret2:
			#left_img = left_img[crop[0]:crop[1], crop[2]:crop[3]]
			#right_img = right_img[crop[0]:crop[1], crop[2]:crop[3]]
			#tim2 = time.time() - time1
			#if tim2 > 0.1:
			#	print( (time.time() - time_g)+" ---> "+(tim2))
			#time1 = time.time()
			resized_left_img = left_img#cv2.resize(left_img,(960,720))
			resized_right_img = right_img#cv2.resize(right_img,(960,720))
			downscale = 1/float(scaling_factor)
			downscale_tracker = 1/float(scaling_factor_tracker)
			small_left_img = cv2.resize(resized_left_img, (0, 0), fx=downscale, fy=downscale)
			small_right_img = cv2.resize(resized_right_img, (0, 0), fx=downscale, fy=downscale)
			small_left_img_tracker = cv2.resize(resized_left_img, (0, 0), fx=downscale_tracker, fy=downscale_tracker)
			small_right_img_tracker = cv2.resize(resized_right_img, (0, 0), fx=downscale_tracker, fy=downscale_tracker)			
			frameIdx = frameIdx + 1
			frame_lock.acquire()
			try:
				left_camera_image = copy.deepcopy(small_left_img)
				right_camera_image = copy.deepcopy(small_right_img)
			finally:
				frame_lock.release()				
			
			if DRAW_RIGHT_EYE:
				tracking_frame = small_right_img_tracker
			else:
				tracking_frame = small_left_img_tracker
			
			if grab_data == True:
				frame_lock.acquire()
				try:
					#left_camera_image = small_left_img
					face_landmarks_list_local = copy.deepcopy(face_landmarks_list_global)
					faces_positions_local = copy.deepcopy(face_locations_global)
					faces_names_local = copy.deepcopy(face_names_global)
					faces_interest_local = copy.deepcopy(face_interest_global)
					tracking_frame_global = copy.deepcopy(tracking_frame)
					grab_data = False	
				finally:
					frame_lock.release()

			for (top, right, bottom, left), name,intrst in zip(faces_positions_local, faces_names_local,faces_interest_local):
				# Scale back up face locations since the frame we detected in was scaled to 1/4 size
				top = top*scaling_factor + cy_sub - dy_sub + tmp_dy_sub*2
				right = right*scaling_factor + cx_sub - dx_sub
				bottom = bottom*scaling_factor + cy_sub - dy_sub + tmp_dy_sub*2
				left = left*scaling_factor + cx_sub - dx_sub

				
				if intrst:
					colorLine = (0, 255, 0)
				else:
					colorLine = (0, 0, 255)
				
				if DRAW_RIGHT_EYE:
					drawingFrame = resized_right_img
					otherFrame = resized_left_img
				else:
					drawingFrame = resized_left_img
					otherFrame = resized_right_img					
					
				# Draw a box around the face
				#cv2.rectangle(resized_left_img, (left, top), (right, bottom), (0, 0, 255), 2)
				center_Y = int(top + float(bottom - top)/2)
				center_X = int(left + float(right - left)/2)
				factorLine = int(factor_line*float(right - left))
				#cv2.line(drawingFrame,(center_X - factorLine,center_Y),(center_X + factorLine,center_Y), colorLine,4)
				#cv2.line(drawingFrame,(center_X,center_Y - factorLine),(center_X,center_Y + factorLine), colorLine,4)
				# Draw a label with a name below the face
				cv2.rectangle(drawingFrame, (left, bottom - 0), (right, bottom), colorLine, -1)
				font = cv2.FONT_HERSHEY_DUPLEX
				cv2.putText(drawingFrame, name, (left + 6, bottom - 6), font, 2.0, colorLine, 1)				
			

			#left_canvas[180:900,:] = resized_left_img.astype('uint8')#((resized_left_img.astype('uint8') + resized_right_img.astype('uint8')) / 2).astype('uint8')
			#right_canvas[180:900,:] = resized_right_img.astype('uint8')
			
			resized_left_img2 = cv2.resize(resized_left_img,(960,720))
			resized_left_img2_no_marker = copy.deepcopy(resized_left_img2)
			
			resized_right_img2 = cv2.resize(resized_right_img,(960,720))		
			resized_right_img2_no_marker = copy.deepcopy(resized_right_img2)
			
			delta_x = 0
			if len(wanted_list_local) > 0:
				for ind, wntd in enumerate(wanted_list_local):
					resized_right_img2[(ind)*wanted_sz+delta_x:(ind+1)*wanted_sz+delta_x,0:wanted_sz] = wntd
					resized_left_img2[(ind)*wanted_sz:(ind+1)*wanted_sz,0:wanted_sz] = wntd
			else:
				frame_lock.acquire()
				try:
					wanted_list_local = copy.deepcopy(wanted_list_global)
				finally:
					frame_lock.release()			
			
			cv2.circle(resized_left_img2,(480,360+tmp_dy_sub), 40, colorLine, 2)
			#cv2.circle(resized_left_img2,(480,360+tmp_dy), 30, (0,0,255), 2)
			cv2.circle(resized_left_img2,(480,360+tmp_dy_sub), 20, colorLine, 2)
			cv2.line(resized_left_img2,(430,360+tmp_dy_sub), (530,360+tmp_dy_sub), colorLine, 2)
			cv2.line(resized_left_img2,(480,310+tmp_dy_sub), (480,410+tmp_dy_sub), colorLine, 2)

			cv2.circle(resized_right_img2,(480,360+tmp_dy_sub), 40, colorLine, 2)
			#cv2.circle(resized_right_img2,(480,360+tmp_dy), 30, (0,0,255), 2)
			cv2.circle(resized_right_img2,(480,360+tmp_dy_sub), 20,colorLine, 2)
			cv2.line(resized_right_img2,(430,360+tmp_dy_sub), (530,360+tmp_dy_sub), colorLine, 2)
			cv2.line(resized_right_img2,(480,310+tmp_dy_sub), (480,410+tmp_dy_sub), colorLine, 2)
			
			cv2.imshow('left',resized_left_img2_no_marker)
			cv2.imshow('right',resized_right_img2)
			#cv2.imshow('left_comp',resized_left_img2_no_marker)
			cv2.imshow('right_comp',resized_right_img2)
			
			cv2.waitKey(1)
			if frameIdx > 10 and  not send_screens:
				os.system('bash send-app-windows'+str(NUMOFSCREENS) +'.sh')
				send_screens=True 	
		
		
def rec_face(frame_rec):
	face_locations = face_recognition.face_locations(frame_rec, number_of_times_to_upsample=0, model="hog")
	return face_locations

def face_recognition_handle():
	global wanted_sz
	global left_camera_image
	global right_camera_image
	global face_locations_global
	global face_names_global
	global face_interest_global
	global face_landmarks_list_global
	global grab_data
	global wanted_list_global
		
	print(os.system('ls /dev/video*'))
	list_names = []
	list_encoding = []
	list_interest = []
	wanted_list_local =[]
	
	tracker = []
	# Initial co-ordinates of the object to be tracked 
	# Create the tracker object
	for fff in range(0,MAX_FACES):
		tracker.append(dlib.correlation_tracker())
		
	for filename in os.listdir(wanted_dir):
		if filename.endswith(".jpg") or filename.endswith(".png"): 
			image = face_recognition.load_image_file(os.path.join(wanted_dir, filename))
			face_encoding = face_recognition.face_encodings(image)[0]
			name = os.path.splitext(filename)[0]			
			list_names.append(name)
			list_encoding.append(face_encoding)
			list_interest.append(True)	
			
			#add wanted pictures to list
			wanted_resized = cv2.resize(image, (wanted_sz,wanted_sz))
			wanted_resized = cv2.cvtColor(wanted_resized, cv2.COLOR_BGR2RGB)
			wanted_resized = cv2.rectangle(wanted_resized, (0, 0), (wanted_sz, wanted_sz), (0, 0, 255),5)
			wanted_list_local.append(wanted_resized)				
			
			frame_lock.acquire()
			try:
				wanted_list_global = wanted_list_local
			finally:
				frame_lock.release()	
		else:
			continue

	for filename in os.listdir(boring_dir):
		#print os.path.join(boring_dir, filename)
		if filename.endswith(".jpg") or filename.endswith(".png"): 
			print os.path.join(boring_dir, filename)
			image = face_recognition.load_image_file(os.path.join(boring_dir, filename))
			face_encoding = face_recognition.face_encodings(image)[0]
			name = os.path.splitext(filename)[0]			
			list_names.append(name)
			list_encoding.append(face_encoding)
			list_interest.append(False)
		else:
			continue
	print list_names
	print list_interest
					
	#print len(list_encoding)
	# Initialize some variables
	face_locations = []
	face_encodings = []
	face_names = []
	face_interest = []
	scaling_factor_tracker = 1
	tim1 = time.time()
	last_time = time.time()
	
	new_faces_pos = []
	
	while True:		
		frame_lock.acquire()
		try:
			if DRAW_RIGHT_EYE:
				frame_rec = copy.deepcopy(right_camera_image)
			else:
				frame_rec = copy.deepcopy(left_camera_image)
		finally:
			frame_lock.release()	
		
 
		if type(frame_rec) is int:
			continue
		else:		
			# Find all the faces and face encodings in the current frame of video
			if time.time() - last_time > 0.95:
				
				# run face recognition only on the center of the image

				sub_frame = copy.deepcopy(frame_rec[cy_sub - dx_sub + tmp_dy_sub :cy_sub+dx_sub + tmp_dy_sub,cx_sub - dy_sub :cx_sub+dy_sub])
				#cv2.imshow('sub_frame',sub_frame)
				#cv2.waitKey(1)
				new_faces_pos = []
						
				face_locations = rec_face(sub_frame);
				
								
				#print face_locations
				#face_locations = rec_face(frame_rec);				
				
				#face_encodings = face_recognition.face_encodings(frame_rec, face_locations)
				
				face_encodings = face_recognition.face_encodings(sub_frame, face_locations)
				
				last_time = time.time()
				face_names = []
				face_interest = []
				faces_cnt = 0 
				for face_encoding in face_encodings:
					# See if the face is a match for the known face(s)
					match = face_recognition.compare_faces(list_encoding, face_encoding)
					faces_cnt = faces_cnt+ 1

					if any(match):
						for m , name,intrs in zip(match,list_names,list_interest):
							if m == True:
								face_names.append(name)
								face_interest.append(intrs)
								break
						else:
							face_names.append("Uninvolved")
							face_interest.append(False)
					'''
					print face_locations
					new_faces_pos.append([face_locations[0][0]+cx,face_locations[0][1]+cy,face_locations[0][2]+cx,face_locations[0][3]+cy])
					print new_faces_pos
					
					face_locations[0][0] = face_locations[0][0]+ cx
					face_locations[0][1] = face_locations[0][1] + cy
					face_locations[0][2] = face_locations[0][2] + cx
					face_locations[0][3] = face_locations[0][3] + cy
					
					face_locations = new_faces_pos
					'''
					frame_lock.acquire()
					try:
						face_names_global = copy.deepcopy(face_names)
						face_locations_global = copy.deepcopy(face_locations)
						face_interest_global = copy.deepcopy(face_interest)
						grab_data = True
					finally:
						frame_lock.release()
					
				if len(face_locations) > 0:
					for fff in range(0,len(face_locations)):
						tracker[fff].start_track(frame_rec, dlib.rectangle(top = int(face_locations[fff][0] / float(scaling_factor_tracker) ),right = int(face_locations[fff][1] / float(scaling_factor_tracker)),bottom= int(face_locations[fff][2] / float(scaling_factor_tracker)),left = int(face_locations[fff][3] / float(scaling_factor_tracker))))	
				

			else:
				k=7
				# Update the tracker 
				if len(face_locations) > 0:
					for fff in range(0,len(face_locations)):
						tracker[fff].update(frame_rec)
						rect = tracker[fff].get_position()
						face_locations[fff] = ([int(rect.top()* scaling_factor_tracker ),int(rect.right()* scaling_factor_tracker),int(rect.bottom()* scaling_factor_tracker),int(rect.left()* scaling_factor_tracker)]) 
						'''
						dx = 0
						dy = 0
						stamp = frame_rec[ int(face_locations[fff][0] / float(scaling_factor_tracker))-dy : int(face_locations[fff][2] / float(scaling_factor_tracker))+dy , int(face_locations[fff][3] / float(scaling_factor_tracker))-dx : int(face_locations[fff][1] / float(scaling_factor_tracker)) +dx ] 
						face_locations_stamp = face_recognition.face_locations(stamp, number_of_times_to_upsample=0, model="hog")
						#print len(face_locations_stamp)
						if len(face_locations_stamp) > 0:
							face_locations[fff] = ([int(0),int(0),int(0),int(0)]) 
						else:									
							face_locations[fff] = ([int(rect.top()* scaling_factor_tracker ),int(rect.right()* scaling_factor_tracker),int(rect.bottom()* scaling_factor_tracker),int(rect.left()* scaling_factor_tracker)]) 
						'''
				time.sleep(0.1)
				frame_lock.acquire()
				try:
					face_names_global = copy.deepcopy(face_names)
					face_locations_global = copy.deepcopy(face_locations)
					face_interest_global = copy.deepcopy(face_interest)
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
	pitch_steps = 0

	while True:
		ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())
		pose = ss.Predicted.Pose
		q = Quat([pose.Orientation.w,pose.Orientation.x,pose.Orientation.y,pose.Orientation.z])   # 	q.ra --> Pitch , q.dec --> Yaw , q.roll --> Roll
		#print q.ra, q.dec, q.roll

		# this part is true only for "pitch" of -90 to 90 degrees (The half dome infront of a person )
		
		if q.dec > 45:
			decLim = 45.0
		elif q.dec < -45:
			decLim = -45.0
		else:
			decLim = q.dec
		steps_per_rev = 127
		yaw_newStep = ((decLim/180)*steps_per_rev)
		yaw_lastStep  = yaw_steps
		yaw_steps = int(round(yaw_newStep))

		'''
		if q.ra > 45 and q.ra < 90:
			raLim = 45.0
		elif q.ra > 90 and q.ra < 135:
			raLim = 135.0
		elif q.ra > 225 and q.ra < 270:
			raLim = 225.0		
		elif q.ra > 270 and q.ra < 315:
			raLim = 315.0		
		else:
			raLim = q.ra
		'''
		if q.ra > 23 and q.ra < 180:
			raLim = 23.0
		elif q.ra > 180 and q.ra < 338:
			raLim = 338.0		
		else:
			raLim = q.ra
			
		if raLim <= 90 or raLim >= 270 :
			raLim = np.mod(raLim + 180,360)			
			
		
		pitch_newStep = (((raLim)/180)*steps_per_rev)
		pitch_lastStep  = pitch_steps
		pitch_steps = int(round(pitch_newStep))		

		#print q.ra,raLim,pitch_steps
		if yaw_steps != yaw_lastStep or pitch_steps != pitch_lastStep:
			print(q.dec,q.ra,yaw_steps,pitch_steps,raLim)
		#ser.write(struct.pack(2*'B',yaw_steps + 128,pitch_steps + 128))
		if serialExist:
			ser.write(struct.pack('BBB',yaw_steps+128,pitch_steps,10))
		time.sleep(0.0005)
		if (serialExist):
			recv = ser.read(1024).lstrip().rstrip()
			if len(recv) > 0:
				print recv


if __name__ == '__main__':
	#serialExist = False
	if serialExist:
		ser = serial.Serial(ArduinoCOM,baudrate=115200)
		ser.timeout = 0.00	
	
	start_new_thread(face_recognition_handle,())
	start_new_thread(cameras_handle,())
	start_new_thread(oculus_handle,())
	#start_new_thread(track_handle,())	

	while True:
		time.sleep(0.01)
