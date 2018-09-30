#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int8


class send_image:
    def __init__(self):
        self.image_pub = rospy.Publisher('/image_raw', Image, queue_size = 1)
        self.take_img = rospy.Subscriber()
        self.brigde = CvBridge()
    
    def publish_image(self)