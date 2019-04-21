#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image",Image)
    self.detection_pub1 = rospy.Publisher("line_detection",String)
    self.detection_pub2 = rospy.Publisher("detection",Bool)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    detection = False

    #gray scale, histogram equalization
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl_image = clahe.apply(gray_image)

    #blur
    cv_image2 = cv2.GaussianBlur(cl_image,(3,3),0)
    cv_image2 = cv2.medianBlur(cv_image2,3)

    #opening
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    open_image = cv2.morphologyEx(cv_image2, cv2.MORPH_OPEN,kernel, iterations = 5)

    #background subtraction
    ret,back = cv2.threshold(cv_image2, 140, 255, cv2.THRESH_TOZERO_INV)
    img_diff = cv2.absdiff(back, open_image)

    #binarize
    ret, thresh = cv2.threshold(img_diff,160,255,0)

    #find contours
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    cv_image = cv2.drawContours(cv_image,contours,-1,(0,255,0),3)

    #bounding rectangle
    rects = list(map(cv2.minAreaRect, contours))
    boxs = list(map(cv2.boxPoints, rects))
    boxs = list(map(np.int0, boxs))
    areas = list(map(cv2.contourArea, boxs))
    cv_image = cv2.drawContours(cv_image,boxs,-1,(255,0,0),2)
    
    #detection
    for i in range(len(contours)):
        w, h = rects[i][1]
        if areas[i] != 0 and w != 0 and h != 0:
            if (((float(w) / h) < 0.25) or ((float(h) / w) < 0.25))and areas[i] > 25000:
                cv_image = cv2.drawContours(cv_image,[boxs[i]],0,(0,0,255),2)
                #print(areas[i])
                #print(float(w) / h)
                
                self.detection_pub1.publish("White Line")
                detection = True
    
    self.detection_pub2.publish(detection)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

    #cv2.imshow("back", back)
    #cv2.imshow("diff", img_diff)
    #cv2.imshow("open_img", open_image)
    cv2.imshow("thresh", thresh)
    cv2.imshow("detection", cv_image)
    #cv2.imshow("gray", gray_image)
    #cv2.imshow("clahe",cl_image)
    cv2.waitKey(3)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
#  video.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

