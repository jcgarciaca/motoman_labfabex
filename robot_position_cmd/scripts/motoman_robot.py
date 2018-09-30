#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

from motoman_msgs.srv import WriteSingleIO

class robot:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander('arm_on_rail')
        self.group.set_planner_id('RRTConnectkConfigDefault')
        self.group_move = moveit_commander.MoveGroupCommander('mh6')
        self.group_move.set_planner_id('RRTConnectkConfigDefault')
        self.start_pose = self.group.get_current_pose('arm_link_tool0')
        self.start_pose_modificable = self.group.get_current_pose('arm_link_tool0')
        self.home_position = [0, 0, 0, 0, 0, 0, 0, 0]
        self.aux_position = [-1.0, 0, 0, 0, 0, 0, 0, 0]

        # ========================== robot poses for task ==========================
        # reference pose is where photo is taken
        self.reference_pose = Pose()
        self.reference_pose.position.x = 1.7356
        self.reference_pose.position.y = -0.9611
        self.reference_pose.position.z = 1.3975
        self.reference_pose.orientation.x = 0.706583
        self.reference_pose.orientation.y = -0.707629
        self.reference_pose.orientation.z = 0.000545
        self.reference_pose.orientation.w = 0.000545
        # -------------------------------------------------------------------------
        # pick_up pose is where robot takes the piece
        self.pick_up_pose = Pose()
        self.pick_up_pose.position.x = 1.8024
        self.pick_up_pose.position.y = -0.9818
        self.pick_up_pose.position.z = 0.3595
        self.pick_up_pose.orientation.x = 0.706636
        self.pick_up_pose.orientation.y = -0.707576
        self.pick_up_pose.orientation.z = 0.000473
        self.pick_up_pose.orientation.w = 0.000485        
        # -------------------------------------------------------------------------
        # place pose is where robot places the piece
        self.place_pose = Pose()
        self.place_pose.position.x = 4.5289
        self.place_pose.position.y = -0.1248
        self.place_pose.position.z = 0.7400
        self.place_pose.orientation.x = 0.999999
        self.place_pose.orientation.y = 0.000783
        self.place_pose.orientation.z = 0.000807
        self.place_pose.orientation.w = 0.0
        # =========================================================================

        # subscribers
        self.move_ref_pose_sub = rospy.Subscriber('/move_reference_pose', Int8, self.move_take_img_pose_cb, queue_size = 1)
        # center point is target center in image
        self.target_point_sub = rospy.Subscriber('/reference_pose_reached', Int8, self.target_point_cb, queue_size = 1)
        self.place_piece_sub = rospy.Subscriber('/pick_up_completed', Int8, self.place_piece_cb, queue_size = 1)
        self.start_pose_cmd_sub = rospy.Subscriber('/place_completed', Int8, self.move_start_cb, queue_size = 1)

        # publishers
        self.move_ref_pose_pub = rospy.Publisher('/reference_pose_reached', Int8, queue_size = 1)
        self.pick_up_complete_pub = rospy.Publisher('/pick_up_completed', Int8, queue_size = 1)
        self.place_complete_pub = rospy.Publisher('/place_completed', Int8, queue_size = 1)
        self.task_complete_pub = rospy.Publisher('/task_completed', Int8, queue_size = 1)


    def move_start_cb(self, data):
        rospy.loginfo('Moving the robot to start pose')
        print('self.start_pose.pose', self.start_pose.pose)
        self.move_robot_joint(self.home_position)
        task_complete = Int8()
        task_complete.data = 1
        self.task_complete_pub.publish(task_complete)


    def place_piece_cb(self, data):
        rospy.loginfo('Moving the robot to place pose')
        place_sup_pose = Pose(position = self.place_pose.position, orientation = self.place_pose.orientation)
        z_aux = self.place_pose.position.z
        place_sup_pose.position.z += 0.30
        self.move_robot(place_sup_pose)

        place_sup_pose.position.z -= 0.20
        self.move_robot(place_sup_pose)

        place_sup_pose.position.z -= 0.10        
        self.move_robot(self.place_pose)
        # vaccum OFF
        rospy.sleep(2)
        self.set_vaccum(1, 0)
        rospy.sleep(2)

        self.move_robot(place_sup_pose)

        place_sup_pose.position.z += 0.20
        self.move_robot(place_sup_pose)

        self.place_pose.position.z = z_aux

        place_piece_complete = Int8()
        place_piece_complete.data = 1
        self.place_complete_pub.publish(place_piece_complete)


    def target_point_cb(self, data):
        rospy.loginfo('Moving the robot to pick up pose')
        pick_up_sup_pose = Pose(position = self.pick_up_pose.position, orientation = self.pick_up_pose.orientation)
        z_aux = self.pick_up_pose.position.z
        pick_up_sup_pose.position.z += 0.10
        self.move_robot(pick_up_sup_pose)

        self.pick_up_pose.position.z = z_aux

        self.move_robot(self.pick_up_pose)
        print('=========================== pick_up_pose ==========================')
        print(self.pick_up_pose)
        print('===================================================================')
        # vaccum ON
        rospy.sleep(2)
        self.set_vaccum(0, 1)
        rospy.sleep(2)

        self.move_robot(pick_up_sup_pose)

        rospy.loginfo('Moving back the robot to reference pose')
        self.move_robot(self.reference_pose)

        self.move_robot_joint(self.aux_position)
        self.move_robot_joint(self.home_position)

        self.pick_up_pose.position.z = z_aux

        pick_up_complete = Int8()
        pick_up_complete.data = 1
        self.pick_up_complete_pub.publish(pick_up_complete)


    def move_take_img_pose_cb(self, data):
        rospy.loginfo('Moving the robot to reference pose')
        self.move_robot_joint(self.aux_position)

        self.move_robot(self.reference_pose)
        
        reference_reached = Int8()
        reference_reached.data = 1
        self.move_ref_pose_pub.publish(reference_reached)


    def move_robot(self, target_pose):
        ik = self.get_ik(target_pose)
        if(ik.error_code.val == 1):
            joints = []
            print(len(ik.solution.joint_state.position))
            for i in range(len(ik.solution.joint_state.position)):
                joints.append(ik.solution.joint_state.position[i])
            self.move_robot_joint(joints)
        else:
            rospy.loginfo('IK not found')


    def move_robot_joint(self, target_joints):
        self.group_move.set_joint_value_target(target_joints)
        self.group_move.go()
        

    def get_ik(self, pose):
        rospy.loginfo('Waiting for IK service')
        rospy.wait_for_service('compute_ik')
        
        ik_request = PositionIKRequest()
        ik_request.group_name = self.group.get_name()
        ik_request.robot_state = self.robot.get_current_state()
        ik_request.ik_link_name = self.group.get_end_effector_link()
        print('ik_request.ik_link_name: ', ik_request.ik_link_name)
        
        pose_stmp = self.start_pose_modificable
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


    def set_vaccum(self, status1, status2):
        rospy.loginfo('Waiting for IO service')
        # rospy.wait_for_service('write_single_io')
        try:
            io_service = rospy.ServiceProxy('write_single_io', WriteSingleIO)
            io_service(10032, status1)
            io_service(10033, status2)
        except rospy.ServiceException, e:
            rospy.logerr('Error with IO service')


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
    rospy.init_node('motoman_robot', anonymous = True)
    rospy.loginfo("Starting node")
    my_robot = robot()
    print("================ Ready ================")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
