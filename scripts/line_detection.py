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

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv_image2 = cv2.GaussianBlur(gray_image,(7,7),0)
    for i in range(10):
        cv_image2 = cv2.GaussianBlur(cv_image2,(7,7),0)
        cv_image2 = cv2.medianBlur(cv_image2,5)

    ret, thresh = cv2.threshold(cv_image2,160,255,0)
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    
    cv_image = cv2.drawContours(cv_image,contours,-1,(0,255,0),3)

    rects = list(map(cv2.minAreaRect, contours))
    boxs = list(map(cv2.boxPoints, rects))
    boxs = list(map(np.int0, boxs))
    areas = list(map(cv2.contourArea, boxs))
    cv_image = cv2.drawContours(cv_image,boxs,-1,(255,0,0),2)
    for i in range(len(contours)):
        w, h = rects[i][1]
        if areas[i] != 0 and w != 0 and h != 0:
            if (((float(w) / h) < 0.4) or ((float(h) / w) < 0.4))and areas[i] > 40000:
                cv_image = cv2.drawContours(cv_image,[boxs[i]],0,(0,0,255),2)
                #print(areas[i])
                #print(float(w) / h)
                
                self.detection_pub1.publish("White Line")
                detection = True

    
    
    self.detection_pub2.publish(detection)

    cv2.imshow("Image window1", thresh)
    cv2.imshow("Image window2", cv_image)
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

