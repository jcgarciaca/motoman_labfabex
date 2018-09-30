#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8
import os
import cv2

import random

CATKIN_WS = '/home/juliocesar/catkin_ws/src'
IMAGE_FOLDER = os.path.join(CATKIN_WS, 'motoman_labfabex/tensorflow_object_detector/data/test_images/')

class send_image:
    def __init__(self):
        self.image_pub = rospy.Publisher('/image_raw', Image, queue_size = 1)
        self.take_img = rospy.Subscriber('/send_image', Int8, self.publish_image, queue_size = 1)
        self.brigde = CvBridge()
        
    
    def publish_image(self, data):
        images = os.listdir(IMAGE_FOLDER)
        image_file = os.path.join(IMAGE_FOLDER, images[random.randint(0, len(images) - 1)])
        cv_image = cv2.imread(image_file)

        try:
            self.image_pub.publish(self.brigde.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main():
    rospy.init_node('publis_image', anonymous = True)
    rospy.loginfo('Node started')
    pub_img = send_image()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()