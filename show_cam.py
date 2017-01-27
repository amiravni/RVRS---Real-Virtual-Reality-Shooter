import cv2
import sys
import os
import time
import numpy as np

cam_left = cv2.VideoCapture(0)
cam_right = cv2.VideoCapture(1)

cv2.namedWindow("left", cv2.WND_PROP_FULLSCREEN)          
cv2.namedWindow("right", cv2.WND_PROP_FULLSCREEN)          
#cv2.setWindowProperty("right", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
#cv2.setWindowProperty("left", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
send_screens = False
frameIdx = 0
left_canvas = np.zeros((1080,960,3)).astype('uint8')
right_canvas = np.zeros((1080,960,3)).astype('uint8')

while True:
    ret,left_img = cam_left.read()
    ret,right_img = cam_right.read()
    #img = cv2.fliplr(img)
    if ret:
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



