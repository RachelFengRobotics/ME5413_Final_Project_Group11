/* goal_publisher_node.cpp

 * Copyright (C) 2023 SS47816

 * ROS Node for publishing goal poses 
 
**/

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace interactive_tools 
{

class GoalPublisherNode
{
 public:
  GoalPublisherNode();
  virtual ~GoalPublisherNode() {};

 private:
  // Helper function to construct pose msgs
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void goalPoseCallback(const std_msgs::String& name);
  tf2::Transform convertPoseToTransform(const geometry_msgs::Pose& pose);
  geometry_msgs::PoseStamped getPoseMsgFromConfig(const std::string& name);

  // ROS declaration
  ros::NodeHandle nh_;
  ros::Publisher pub_goal_;
  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_goal_name_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_bcaster_;
  // Robot pose
  std::string world_frame_;
  std::string map_frame_;
  std::string robot_frame_;
  geometry_msgs::Pose robot_pose_;
};

GoalPublisherNode::GoalPublisherNode() : tf2_listener_(tf2_buffer_)
{
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &GoalPublisherNode::robotOdomCallback, this);
  sub_goal_name_ = nh_.subscribe("/goal_name", 1, &GoalPublisherNode::goalPoseCallback, this);

  // Initialization
  robot_frame_ = "base_link";
  map_frame_ = "map";
  world_frame_ = "world";
};

void GoalPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->robot_pose_ = odom->pose.pose;

  const tf2::Transform T_world_robot = convertPoseToTransform(this->robot_pose_);
  const tf2::Transform T_robot_world = T_world_robot.inverse();

  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->robot_frame_;
  transformStamped.child_frame_id = this->world_frame_;
  transformStamped.transform.translation.x = T_robot_world.getOrigin().getX();
  transformStamped.transform.translation.y = T_robot_world.getOrigin().getY();
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = T_robot_world.getRotation().getX();
  transformStamped.transform.rotation.y = T_robot_world.getRotation().getY();
  transformStamped.transform.rotation.z = T_robot_world.getRotation().getZ();
  transformStamped.transform.rotation.w = T_robot_world.getRotation().getW();
  
  tf2_bcaster_.sendTransform(transformStamped);
  return;
};

void GoalPublisherNode::goalPoseCallback(const std_msgs::String& name)
{
  const auto pose_msg = getPoseMsgFromConfig(name.data);
  this->pub_goal_.publish(pose_msg);
  return;
};

tf2::Transform GoalPublisherNode::convertPoseToTransform(const geometry_msgs::Pose& pose)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));
  tf2::Quaternion q;
  q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  T.setRotation(q);

  return T;
};

geometry_msgs::PoseStamped GoalPublisherNode::getPoseMsgFromConfig(const std::string& name)
{
  /** 
   * Get the Transform from goal to world from the file
   */

  double x, y, yaw;
  nh_.getParam("/me5413_project" + name + "/x", x);
  nh_.getParam("/me5413_project" + name + "/y", y);
  nh_.getParam("/me5413_project" + name + "/yaw", yaw);
  nh_.getParam("/me5413_project/frame_id", this->world_frame_);

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();

  geometry_msgs::PoseStamped P_world_goal;
  P_world_goal.pose.position.x = x;
  P_world_goal.pose.position.y = y;
  P_world_goal.pose.orientation = tf2::toMsg(q);

  /** 
   * Get the Transform from world to map from the tf_listener
   */
  geometry_msgs::TransformStamped transform_map_world;
  try
  {
      transform_map_world = tf2_buffer_.lookupTransform(this->map_frame_, this->world_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
      ROS_WARN("%s", ex.what());
      return geometry_msgs::PoseStamped();
  }

  /** 
   * Transform pose msg to map frame
   */
  geometry_msgs::PoseStamped P_map_goal;
  tf2::doTransform(P_world_goal, P_map_goal, transform_map_world);
  P_map_goal.header.frame_id = map_frame_;

  return P_map_goal;
};

} // namespace lgsvl_utils

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher_node");
  interactive_tools::GoalPublisherNode goal_publisher_node;
  ros::spin();  // spin the ros node.
  return 0;
}