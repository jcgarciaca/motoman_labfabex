#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Int8

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

class robot:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander('manipulator')
        # self.group.set_planner_id('RRTConnectkConfigDefault')
        self.start_pose = self.group.get_current_pose()

        # ========================== robot poses for task ==========================
        # reference pose is where photo is taken
        self.reference_pose = geometry_msgs.msg.Pose()
        self.reference_pose.position.x = 0.3890
        self.reference_pose.position.y = -0.0480
        self.reference_pose.position.z = 0.1702
        self.reference_pose.orientation.x = 0.0
        self.reference_pose.orientation.y = 0.9018
        self.reference_pose.orientation.z = 0.0
        self.reference_pose.orientation.w = 0.4320
        # -------------------------------------------------------------------------
        # place pose is where robot places the piece
        self.place_pose = geometry_msgs.msg.Pose()
        self.place_pose.position.x = 0.5325
        self.place_pose.position.y = -0.0480
        self.place_pose.position.z = 0.4218
        self.place_pose.orientation.x = 0.0
        self.place_pose.orientation.y = 0.6452
        self.place_pose.orientation.z = 0.0
        self.place_pose.orientation.w = 0.7639
        # =========================================================================

        # subscribers
        self.move_ref_pose_sub = rospy.Subscriber('/move_reference_pose', Int8, self.move_take_img_pose_cb, queue_size = 1)
        # center point is target center in image
        self.target_point_sub = rospy.Subscriber('/center_point', geometry_msgs.msg.Point, self.target_point_cb, queue_size = 1)
        self.place_piece_sub = rospy.Subscriber('/pick_up_completed', Int8, self.place_piece_cb, queue_size = 1)
        self.start_pose_cmd_sub = rospy.Subscriber('/place_completed', Int8, self.move_start_cb, queue_size = 1)

        # publishers
        self.move_ref_pose_pub = rospy.Publisher('/reference_pose_reached', Int8, queue_size = 1)
        self.pick_up_complete_pub = rospy.Publisher('/pick_up_completed', Int8, queue_size = 1)
        self.place_complete_pub = rospy.Publisher('/place_completed', Int8, queue_size = 1)
        self.task_complete_pub = rospy.Publisher('/task_completed', Int8, queue_size = 1)


    def move_start_cb(self, data):
        self.group.set_pose_target(self.start_pose)

        # planning trajectory
        plan = self.group.plan()

        # move robot
        rospy.loginfo('Moving the robot to start pose')
        self.group.go()
        

        task_complete = Int8()
        task_complete.data = 1
        self.task_complete_pub.publish(task_complete)


    def place_piece_cb(self, data):
        # aux_place_pose = self.place_pose
        # aux_place_pose.position.z += 0.05
        self.group.set_pose_target(self.place_pose)

        # planning trajectory
        plan = self.group.plan()

        # move robot
        rospy.loginfo('Moving the robot to place pose')
        self.group.go()
        

        place_piece_complete = Int8()
        place_piece_complete.data = 1
        self.place_complete_pub.publish(place_piece_complete)


    def target_point_cb(self, data):
        # define pick up pose
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = 0.3305
        pose_target.position.y = -0.0480
        pose_target.position.z = 0.5495
        pose_target.orientation.x = 0.0
        pose_target.orientation.y = 0.5626
        pose_target.orientation.z = 0.0
        pose_target.orientation.w = 0.8296
        
        self.group.set_pose_target(pose_target)

        # planning trajectory
        plan = self.group.plan()

        # move robot
        rospy.loginfo('Moving the robot to pick up pose')
        self.group.go()

        # enable output
        rospy.sleep(2)

        # move to reference point
        self.group.set_pose_target(self.reference_pose)

        # planning trajectory
        plan = self.group.plan()

        # move robot
        rospy.loginfo('Moving back the robot to reference pose')
        self.group.go()
        

        pick_up_complete = Int8()
        pick_up_complete.data = 1
        self.pick_up_complete_pub.publish(pick_up_complete)


    def move_take_img_pose_cb(self, data):
        self.group.set_pose_target(self.reference_pose)

        # planning trajectory
        plan = self.group.plan()

        # move robot
        rospy.loginfo('Moving the robot to reference pose')
        self.group.go()
        

        reference_reached = Int8()
        reference_reached.data = 1
        self.move_ref_pose_pub.publish(reference_reached)


    def get_ik(self, pose):
        rospy.loginfo('Waiting for service')
        rospy.wait_for_service('compute_ik')
        
        ik_request = PositionIKRequest()
        ik_request.group_name = self.group.get_name()
        ik_request.robot_state = self.robot.get_current_state()
        ik_request.ik_link_name = self.group.get_end_effector_link()
        
        pose_stmp = self.start_pose
        pose_stmp.pose = pose
        ik_request.pose_stamped = pose_stmp
        ik_request.timeout.secs = 10
        ik_request.attempts = 0
        try:
            ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)
            resp = ik_service(ik_request)
            print(resp.solution.joint_state)
            return resp
        except rospy.ServiceException, e:
            rospy.logerr('Error with IK service')
        
        

    def print_pose(self, pose):
        print("============ Start pose ============")
        print("Position: ", pose.pose.position.x, 
            pose.pose.position.y, 
            pose.pose.position.z)
        print("Orientation: ", pose.pose.orientation.x, 
            pose.pose.orientation.y, 
            pose.pose.orientation.z, 
            pose.pose.orientation.w)
        print("====================================")
        

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous = True)
    rospy.loginfo("Starting node")
    my_robot = robot()
    my_robot.print_pose(my_robot.start_pose)
    my_robot.get_ik(my_robot.start_pose.pose)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
