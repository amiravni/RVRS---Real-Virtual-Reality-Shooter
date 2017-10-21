'''
Simply display the contents of the webcam with optional mirroring using OpenCV 
via the new Pythonic cv2 interface.  Press <esc> to quit.
'''

import cv2

def show_webcam(mirror=False):
  cam = cv2.VideoCapture(0)
  print cam.isOpened()
  while True:
    ret_val, img = cam.read()
    if ret_val == True:
      cv2.imshow('my webcam', img)
      if cv2.waitKey(1) == 27: 
        break  # esc to quit
  cv2.destroyAllWindows()

def main():
  show_webcam(mirror=False)

if __name__ == '__main__':
  main()

'''
from SimpleCV import Image, Camera

cam = Camera()
img = cam.getImage()
img.save("filename.jpg")
'''

#sudo ls -lah /proc/*/fd | grep video0