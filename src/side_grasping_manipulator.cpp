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

#define PRE_FINGER_OPEN     1.5
#define GRASPING_FINGER     0.15

std_msgs::String 												                   status;
std_msgs::String                                          _cupPoseCmdStr;
ros::Publisher 													                   status_pub;
ros::Publisher                                             left_finger_pub;
ros::Publisher                                             right_finger_pub;
ros::Publisher                                            _getCupPoseArmCmd_pub;
moveit::planning_interface::PlanningSceneInterface 				*planning_scene_interface;
moveit::planning_interface::MoveGroup 						      	*group;
moveit::planning_interface::MoveGroup::Plan 					    *my_plan;
geometry_msgs::PointStamped                                cupPosition;
geometry_msgs::PointStamped                                cupPositionBaseLinkFrame;

void cupPoseArm(const bgumodo_arm::ThreePoints::ConstPtr& res)
{
  cupPosition.point           = res->object;
  cupPosition.header.frame_id = "wrist_link";
  cupPosition.header.stamp    = ros::Time(0);
}

void sideGraspingManipulator(const geometry_msgs::Point::ConstPtr& req)
{
  tf::TransformListener listener;
  ros::Duration(1.0).sleep();
  _cupPoseCmdStr.data = "Start";
  status.data = "Waiting";
  status_pub.publish(status);
  if(!(req->x == 0.0 && req->y == 0.0 && req->z == 0.0))
  {
  	  status.data = "In progress";
  	  status_pub.publish(status);
	  // desired pos goal
	  geometry_msgs::Pose desired_pose;

	  desired_pose.position.x = req->x;
	  desired_pose.position.y = req->y;
	  desired_pose.position.z = req->z;
	  desired_pose.orientation.w = 1.0;

	  ROS_INFO("here 1");

	  // set the joints target
	  std::vector<double> group_variable_values = IK(desired_pose);

    std_msgs::Float64 left_finger_value;
    std_msgs::Float64 right_finger_value;
    left_finger_value.data   = -PRE_FINGER_OPEN;
    right_finger_value.data  = PRE_FINGER_OPEN;
    left_finger_pub.publish(left_finger_value);
    right_finger_pub.publish(right_finger_value);

	  group->setJointValueTarget(group_variable_values);

	  bool success = group->plan(*my_plan);

	  if (success) {
		  group->execute(*my_plan);
		  ros::Duration(3.0).sleep();
	  }

    left_finger_value.data  = GRASPING_FINGER;
    right_finger_value.data = -GRASPING_FINGER;
    left_finger_pub.publish(left_finger_value);
    right_finger_pub.publish(right_finger_value);

    desired_pose.position.z += 0.05;
    desired_pose.position.x -= 0.05;
    group_variable_values = IK(desired_pose);

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
  ros::init(argc, argv, "komodo_side_grasping_manipulator");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  planning_scene_interface 		     = new moveit::planning_interface::PlanningSceneInterface;
  group 						               = new moveit::planning_interface::MoveGroup("arm");
  my_plan 						             = new moveit::planning_interface::MoveGroup::Plan;

  left_finger_pub                  = node_handle.advertise<std_msgs::Float64>("left_finger_controller/command", 100);
  right_finger_pub                 = node_handle.advertise<std_msgs::Float64>("right_finger_controller/command", 100);

  _getCupPoseArmCmd_pub            = node_handle.advertise<std_msgs::String>("detector/observe_cup_arm_cmd", 100);

  status_pub 					             = node_handle.advertise<std_msgs::String>("side_grasping_manipulator/status", 100);
  ros::Subscriber sub 			       = node_handle.subscribe("side_grasping_manipulator/command", 1, sideGraspingManipulator);
  ros::Subscriber _CupPoseArm_sub  = node_handle.subscribe("detector/observe_cup_arm_res", 1, cupPoseArm);

  ros::Duration(1.0).sleep(); //time to construct ros subscribers and publishers

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group->getEndEffectorLink().c_str());

  ROS_INFO("Ready to get cup position...");
  
  ros::spin();
  return 0;
}
