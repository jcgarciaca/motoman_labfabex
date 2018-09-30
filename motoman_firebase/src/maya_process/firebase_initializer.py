#!/usr/bin/env python

import json
import rospy
import requests
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Point


beginPub = rospy.Publisher('task_begin', String, queue_size=10)
updatePub = rospy.Publisher('task_update', String, queue_size=10)

moveTopic = rospy.Publisher('move_reference_pose', Int8, queue_size=10)

def init():
    rospy.init_node('firebase_listener',
                    log_level=rospy.DEBUG, anonymous=True)
    rospy.Subscriber('task_step', String, step_received)
    rospy.Subscriber('task_receive', String, task_received)

    # requests.post("http://localhost:3000/connect")
    rospy.spin()


def task_received(msg):
    jsn = json.loads(msg.data)
    rospy.logdebug('task decoded to %s' % jsn)
    beginPub.publish(jsn["key"])


def step_received(msg):
    moveTopic.publish(Int8())
    

if __name__ == '__main__':
    init()
