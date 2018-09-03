#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import sys

class ip_camera:
    def __init__(self):
        self.image_pub = rospy.Publisher('/image/raw', Image, queue_size=1)
        self.cap = cv2.VideoCapture('rtsp://motoman:motoman@168.176.36.42:88/videoMain') # cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def get_camera_image(self):
        while not rospy.is_shutdown():
            # capture frame
            ret, cv_image = self.cap.read()

            # display image
            # cv2.imshow("Image", cv_image)

            # publish image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)
            cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('ip_camera')
    rospy.loginfo('Running')
    cam = ip_camera()
    cam.get_camera_image()    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()
