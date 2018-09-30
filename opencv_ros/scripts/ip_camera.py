#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import sys

class ip_camera:
    def __init__(self):
        self.image_pub = rospy.Publisher('/image_raw', Image, queue_size = 1)
        self.compressed_pub = rospy.Publisher('/compressed_img', CompressedImage, queue_size = 1)
        self.cap = cv2.VideoCapture('rtsp://motoman:motoman@168.176.27.81:88/videoMain')
        self.bridge = CvBridge()
        self.msg = CompressedImage()
        self.msg.format = 'jpeg'

    def get_camera_image(self):
        while not rospy.is_shutdown():
            # capture frame
            ret, cv_image = self.cap.read()

            # display image
            # cv2.imshow("Image", cv_image)

            # publish image
            self.msg.header.stamp = rospy.Time.now()
            self.msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)
            
            self.compressed_pub.publish(self.msg)
            cv2.waitKey(1)


def main():
    rospy.init_node('ip_camera', anonymous = True)
    rospy.loginfo('Running')
    cam = ip_camera()
    cam.get_camera_image()    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
