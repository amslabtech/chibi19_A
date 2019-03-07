#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv_image2 = cv2.GaussianBlur(gray_image,(3,3),0)
    cv_image2 = cv2.medianBlur(cv_image2,5)
    ret, thresh = cv2.threshold(cv_image2,200,255,0)
   # th2 = cv2.adaptiveThreshold(cv_image2,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,21,20)
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    cv_image = cv2.drawContours(cv_image,contours,-1,(0,255,0),3)
    rects = list(map(cv2.minAreaRect, contours))
    boxs = list(map(cv2.boxPoints, rects))
    boxs = list(map(np.int0, boxs))
    areas = list(map(cv2.contourArea, boxs))
    cv_image = cv2.drawContours(cv_image,boxs,-1,(255,0,0),2)
 #   print(areas)
    for i in range(len(contours)):
#        rect = cv2.minAreaRect(contours[i])
        w, h = rects[i][1]
#        box = cv2.boxPoints(rect)
#        box = np.int0(box)
#        area = cv2.contourArea(box)
        if areas[i] != 0 and w != 0 and h != 0:
            if ((float(w) / h) < 0.1) and areas[i] > 1000:
              cv_image = cv2.drawContours(cv_image,[boxs[i]],0,(0,0,255),2)

 #   cv_image = cv2.drawContours(cv_image,boxs,-1,(0,0,255),2)


   # cv_image = cv2.drawContours(cv_image,contours,-1,(0,255,0),3)
   # cv_image3 = cv2.Canny(gray_image, 50, 150.0)
   # cv_image3 = cv2.Laplacian(cv_image2, cv2.CV_32F)
    #cv2.imshow("Image window", cv_image2)
    cv2.imshow("Image window1", thresh)
    cv2.imshow("Image window2", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

