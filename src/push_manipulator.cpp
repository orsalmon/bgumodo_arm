#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_listener.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "bgumodo_arm/ThreePoints.h"

#include "helperfunctions.h"

#define FINGER  0.6

std_msgs::String 												                   status;
std_msgs::String                                          _buttonPoseCmdStr;
ros::Publisher 													                   status_pub;
ros::Publisher                                             left_finger_pub;
ros::Publisher                                             right_finger_pub;
ros::Publisher                                            _getButtonPoseArmCmd_pub;
moveit::planning_interface::PlanningSceneInterface 				*planning_scene_interface;
moveit::planning_interface::MoveGroup 						      	*group;
moveit::planning_interface::MoveGroup::Plan 					    *my_plan;
geometry_msgs::PointStamped                                buttonPosition;
geometry_msgs::PointStamped                                buttonPositionBaseLinkFrame;

void buttonPoseArm(const bgumodo_arm::ThreePoints::ConstPtr& res)
{
  buttonPosition.point           = res->object;
  buttonPosition.header.frame_id = "wrist_link";
  buttonPosition.header.stamp    = ros::Time(0);
}

void pushingManipulator(const geometry_msgs::Point::ConstPtr& req)
{
  tf::TransformListener listener;
  ros::Duration(1.0).sleep();
  _buttonPoseCmdStr.data = "Start";
  status.data = "Waiting";
  status_pub.publish(status);
  if(!(req->x == 0.0 && req->y == 0.0 && req->z == 0.0))
  {
  	  status.data = "In progress";
  	  status_pub.publish(status);
	  // desired pos goal
	  geometry_msgs::Pose desired_pose;

	  desired_pose.position.x = req->x - 0.25;
	  desired_pose.position.y = req->y;
	  desired_pose.position.z = req->z;
	  desired_pose.orientation.w = 1.0;

	  ROS_INFO("here 1");

	  // set the joints target
	  std::vector<double> group_variable_values = IK(desired_pose);

    std_msgs::Float64 left_finger_value;
    std_msgs::Float64 right_finger_value;
    left_finger_value.data   = FINGER;
    right_finger_value.data  = -FINGER;
    left_finger_pub.publish(left_finger_value);
    right_finger_pub.publish(right_finger_value);

	  group->setJointValueTarget(group_variable_values);

	  bool success = group->plan(*my_plan);

	  if (success) {
		  group->execute(*my_plan);
		  ros::Duration(3.0).sleep();
	  }

    _getButtonPoseArmCmd_pub.publish(_buttonPoseCmdStr);
    ros::Duration(2.0).sleep();
    ROS_INFO("frame=wrist_link x=%f y=%f z=%f",buttonPosition.point.x,buttonPosition.point.y,buttonPosition.point.z);
    if (buttonPosition.header.frame_id == "") {
      ROS_ERROR("cant detect cup from arm camera!");
      return;
    }
    listener.transformPoint("base_link",buttonPosition,buttonPositionBaseLinkFrame);
    ROS_INFO("frame=base_link x=%f y=%f z=%f",buttonPositionBaseLinkFrame.point.x,buttonPositionBaseLinkFrame.point.y,buttonPositionBaseLinkFrame.point.z);

    desired_pose.position       = buttonPositionBaseLinkFrame.point;
    desired_pose.orientation.w  = 1.0;
    group_variable_values = IK(desired_pose);

    group->setJointValueTarget(group_variable_values);

    success = group->plan(*my_plan);

    if (success) {
      group->execute(*my_plan);
      ros::Duration(3.0).sleep();
    }

    group_variable_values[0] = -1.3;
    group_variable_values[1] = 1.4;
    group_variable_values[2] = 1.5;
    group_variable_values[3] = 1.5;
    group_variable_values[4] = 0.0;

    group->setJointValueTarget(group_variable_values);

    success = group->plan(*my_plan);

    if (success) {
      group->execute(*my_plan);
      ros::Duration(3.0).sleep();
    }

    status.data = "Success";
    status_pub.publish(status);
   }
  return;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "komodo_pushing_manipulator");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  planning_scene_interface 		     = new moveit::planning_interface::PlanningSceneInterface;
  group 						               = new moveit::planning_interface::MoveGroup("arm");
  my_plan 						             = new moveit::planning_interface::MoveGroup::Plan;

  left_finger_pub                  = node_handle.advertise<std_msgs::Float64>("left_finger_controller/command", 100);
  right_finger_pub                 = node_handle.advertise<std_msgs::Float64>("right_finger_controller/command", 100);

  _getButtonPoseArmCmd_pub         = node_handle.advertise<std_msgs::String>("detector/observe_cup_arm_cmd", 100);

  status_pub 					             = node_handle.advertise<std_msgs::String>("pushing_manipulator/status", 100);
  ros::Subscriber sub 			       = node_handle.subscribe("pushing_manipulator/command", 1, pushingManipulator);
  ros::Subscriber _CupPoseArm_sub  = node_handle.subscribe("detector/observe_cup_arm_res", 1, buttonPoseArm);

  ros::Duration(1.0).sleep(); //time to construct ros subscribers and publishers

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group->getEndEffectorLink().c_str());

  ROS_INFO("Ready to get button position...");
  
  ros::spin();
  return 0;
}
