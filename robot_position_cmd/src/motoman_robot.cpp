#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPositionIK.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include <motoman_msgs/WriteSingleIO.h>

void move_take_img_pose_cb(const std_msgs::Int8ConstPtr msg);
void target_point_cb(const std_msgs::Int8ConstPtr msg);
void place_piece_cb(const std_msgs::Int8ConstPtr msg);
void move_start_cb(const std_msgs::Int8ConstPtr msg);
int getIK(moveit_msgs::GetPositionIK & server, ros::ServiceClient & client, 
    moveit::planning_interface::MoveGroupInterface & group, 
    geometry_msgs::Pose & target_pose, std::vector<double> & joint_group_positions);
void move_robot_joints(moveit::planning_interface::MoveGroupInterface & move_group, 
    std::vector<double> & target_joints);
void define_task_positions();
void move_robot_to_pose(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group, geometry_msgs::Pose & pose);
void move_to_take_img_pose(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group);
void move_to_target_point(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group);
void move_to_place_piece(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group);
void move_to_start(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group);
void set_vaccum(int status1, int status2);

// publishers
ros::Publisher move_ref_pose_pub;
ros::Publisher pick_up_complete_pub;
ros::Publisher place_complete_pub;
ros::Publisher task_complete_pub;


static const std::string PLANNING_GROUP = "arm_on_rail";
static const std::string MOVE_GROUP = "mh6";

geometry_msgs::PoseStamped start_pose_stmp;
std::vector<double> joint_move_group_positions;
sensor_msgs::JointState joint_state;
std::vector<double> home_position, aux_position;

bool move_to_reference_pose = false, move_to_pick_up = false, 
    move_to_place = false, move_start_pos = false;

// task positions
geometry_msgs::Pose reference_pose, pick_up_pose, place_pose;

// servers
moveit_msgs::GetPositionIK ik_server;
motoman_msgs::WriteSingleIO io_server;
// clients
ros::ServiceClient ik_client;
ros::ServiceClient io_client;


int main(int argc, char * argv[]){
    ros::init(argc, argv, "motoman_robot");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // planning group    
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    group.setPlannerId("RRTConnectkConfigDefault");
    moveit::core::RobotStatePtr current_state_group = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> joint_group_positions;
    current_state_group->copyJointGroupPositions(joint_model_group, joint_group_positions);
    start_pose_stmp = group.getCurrentPose(group.getEndEffectorLink());


    // move group
    moveit::planning_interface::MoveGroupInterface move_group(MOVE_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    moveit::core::RobotStatePtr current_state_move_group = move_group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_move_group = move_group.getCurrentState()->getJointModelGroup(MOVE_GROUP);
    
    current_state_group->copyJointGroupPositions(joint_model_move_group, joint_move_group_positions);

    define_task_positions();

    // subscribers
    ros::Subscriber move_ref_pose_sub = nh.subscribe("/move_reference_pose", 1, move_take_img_pose_cb);
    ros::Subscriber target_point_sub = nh.subscribe("/reference_pose_reached", 1, target_point_cb);
    ros::Subscriber place_piece_sub = nh.subscribe("/pick_up_completed", 1, place_piece_cb);
    ros::Subscriber start_pose_cmd_sub = nh.subscribe("/place_completed", 1, move_start_cb);

    // publishers
    move_ref_pose_pub = nh.advertise<std_msgs::Int8>("/reference_pose_reached", 1);
    pick_up_complete_pub = nh.advertise<std_msgs::Int8>("/pick_up_completed", 1);
    place_complete_pub = nh.advertise<std_msgs::Int8>("/place_completed", 1);
    task_complete_pub = nh.advertise<std_msgs::Int8>("/task_completed", 1);

    
    // clients
    ik_client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    io_client = nh.serviceClient<motoman_msgs::WriteSingleIO>("write_single_io");

    while(ros::ok()){
        if(move_to_reference_pose){
            move_to_reference_pose = false;
            move_to_take_img_pose(group, move_group);
            ros::Duration(0.2).sleep();
            move_to_target_point(group, move_group);
            ros::Duration(0.2).sleep();
            move_to_place_piece(group, move_group);
            ros::Duration(0.2).sleep();
            move_to_start(group, move_group);
        }
/*
        if(move_to_reference_pose){
            move_to_reference_pose = false;
            move_to_take_img_pose(group, move_group);
        }else if(move_to_pick_up){
            move_to_pick_up = false;
            move_to_target_point(group, move_group);
        }else if(move_to_place){
            move_to_place = false;
            move_to_place_piece(group, move_group);
        }else if(move_start_pos){
            move_start_pos = false;
            move_to_start(group, move_group);
        }
*/
        ros::spinOnce();
    }

    // ros::spin();
    ros::shutdown();
    return 0;
}


void move_to_take_img_pose(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group){

    ROS_INFO("Moving the robot to reference pose");
    move_robot_joints(move_group, aux_position);

    move_robot_to_pose(group, move_group, reference_pose);

    std_msgs::Int8 reference_reached;
    reference_reached.data = 1;
    ros::Duration(0.2).sleep();
    // move_ref_pose_pub.publish(reference_reached);
}

void move_take_img_pose_cb(const std_msgs::Int8ConstPtr msg){
    move_to_reference_pose = true;    
}


void move_to_target_point(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group){

    ROS_INFO("Moving the robot to pick up pose");
    geometry_msgs::Pose pick_up_sup_pose = pick_up_pose;
    pick_up_sup_pose.position.z += 0.1;
    move_robot_to_pose(group, move_group, pick_up_sup_pose);

    move_robot_to_pose(group, move_group, pick_up_pose);
    
    // vaccum ON
    ros::Duration(1.0).sleep();
    set_vaccum(0, 1);
    ros::Duration(1.0).sleep();
    
    move_robot_to_pose(group, move_group, pick_up_sup_pose);
    ROS_INFO("Moving back the robot to reference pose");
    move_robot_to_pose(group, move_group, reference_pose);

    move_robot_joints(move_group, aux_position);
    move_robot_joints(move_group, home_position);

    std_msgs::Int8 pick_up_complete;
    pick_up_complete.data = 1;
    ros::Duration(0.2).sleep();
    // pick_up_complete_pub.publish(pick_up_complete);
}

void target_point_cb(const std_msgs::Int8ConstPtr msg){
    move_to_pick_up = true;
}


void move_to_place_piece(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group){
    ROS_INFO("Moving the robot to place pose");
    geometry_msgs::Pose place_sup_pose = place_pose;
    place_sup_pose.position.z += 0.3;
    move_robot_to_pose(group, move_group, place_sup_pose);

    place_sup_pose.position.z -= 0.2;
    move_robot_to_pose(group, move_group, place_sup_pose);

    move_robot_to_pose(group, move_group, place_pose);
    // vaccum OFF
    ros::Duration(1.0).sleep();
    set_vaccum(1, 0);
    ros::Duration(1.0).sleep();
    
    ROS_INFO("Moving back the robot to reference pose");
    move_robot_to_pose(group, move_group, place_sup_pose);

    place_sup_pose.position.z += 0.2;
    move_robot_to_pose(group, move_group, place_sup_pose);

    std_msgs::Int8 place_piece_complete;
    place_piece_complete.data = 1;
    ros::Duration(0.2).sleep();
    // place_complete_pub.publish(place_piece_complete);
}

void place_piece_cb(const std_msgs::Int8ConstPtr msg){
    move_to_place = true;
}


void move_to_start(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group){
    ROS_INFO("Moving the robot to start pose");
    move_robot_joints(move_group, home_position);

    std_msgs::Int8 task_complete;
    task_complete.data = 1;
    ros::Duration(0.2).sleep();
    // task_complete_pub.publish(task_complete);
}


void move_start_cb(const std_msgs::Int8ConstPtr msg){
    move_start_pos = true;
}


int getIK(moveit_msgs::GetPositionIK & server, ros::ServiceClient & client, 
    moveit::planning_interface::MoveGroupInterface & group, 
    geometry_msgs::Pose & target_pose, std::vector<double> & joint_group_positions){
    
    // calculate IK
    server.request.ik_request.group_name = group.getName();
    server.request.ik_request.ik_link_name = group.getEndEffectorLink();
    
    // set desired target pose stamped
    geometry_msgs::PoseStamped target_pose_stmp = start_pose_stmp;
    target_pose_stmp.pose = target_pose;
    server.request.ik_request.pose_stamped = target_pose_stmp;

    server.request.ik_request.timeout = ros::Duration(10.0);
    server.request.ik_request.attempts = 0;

    client.call(server);
    if(server.response.error_code.val == 1){
        joint_state = server.response.solution.joint_state;
        for(int i = 0; i < joint_state.position.size(); i++){
            joint_group_positions[i] = joint_state.position[i];
        }
        return 1;
    }else{
        ROS_INFO("IK solution not found");
        return 0;
    }
}


void move_robot_to_pose(moveit::planning_interface::MoveGroupInterface & group, 
    moveit::planning_interface::MoveGroupInterface & move_group, geometry_msgs::Pose & pose){
    if(getIK(ik_server, ik_client, group, pose, joint_move_group_positions)){
        joint_move_group_positions[7] = 0.0;
        move_robot_joints(move_group, joint_move_group_positions);
    }
}


void move_robot_joints(moveit::planning_interface::MoveGroupInterface & move_group, 
    std::vector<double> & target_joints){
    move_group.setJointValueTarget(target_joints);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success) move_group.move();
}


void set_vaccum(int status1, int status2){
	
	io_server.request.address = 10032;
	io_server.request.value = status1;
	if(io_client.call(io_server)){
        // std::cout << "Reset 10032" << std::endl;
    }else{
        ROS_ERROR("Failed to call service write_single_io");
	}

    io_server.request.address = 10033;
    io_server.request.value = status2;
    if(io_client.call(io_server)){
        // std::cout << "Set 10033" << std::endl;
    }else{
        ROS_ERROR("Failed to call service write_single_io");
    }	
}


void define_task_positions() {
    reference_pose.position.x = 1.7356;
    reference_pose.position.y = -0.9611;
    reference_pose.position.z = 1.3875;
    reference_pose.orientation.x = 0.706583;
    reference_pose.orientation.y = -0.707629;
    reference_pose.orientation.z = 0.000545;
    reference_pose.orientation.w = 0.000545;

    pick_up_pose.position.x = 1.8024;
    pick_up_pose.position.y = -0.9818;
    pick_up_pose.position.z = 0.3405;
    pick_up_pose.orientation.x = 0.706636;
    pick_up_pose.orientation.y = -0.707576;
    pick_up_pose.orientation.z = 0.000473;
    pick_up_pose.orientation.w = 0.000485;

    place_pose.position.x = 4.5289;
    place_pose.position.y = -0.1248;
    place_pose.position.z = 0.7300;
    place_pose.orientation.x = 0.999999;
    place_pose.orientation.y = 0.000783;
    place_pose.orientation.z = 0.000807;
    place_pose.orientation.w = 0.0;

    for(int i = 0; i < 8; i++){
        home_position.push_back(0.0);
        aux_position.push_back(0.0);
    }
    aux_position[0] = -1.0;
}
