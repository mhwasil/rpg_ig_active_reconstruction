#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "ig_active_reconstruction_ros/param_loader.hpp"
#include "ig_active_reconstruction_msgs/yb_move_arm_using_joints.h"
#include "ig_active_reconstruction_youbot/yb_view_space.hpp"
#include <ig_active_reconstruction_youbot/yb_ros_server.hpp>
#include <fstream>
#include <stdexcept>

void move_arm_using_pose()
{
    //If poses given,     
/*  group.setPoseReferenceFrame("base_footprint");
  geometry_msgs::Pose pose;
  
  pose.position.x = 0.40440883559;
  pose.position.y = -0.000765396669577;
  pose.position.z = 0.285091117453;
  pose.orientation.x = 0.000849928970774;
  pose.orientation.y = 0.990093735747;
  pose.orientation.z = 0.000676013130809;
  pose.orientation.w = 0.140403757288;*/
  //Eigen::Affine3d pose = Eigen::Translation3d(0.40440883559, -0.000765396669577, 0.285091117453)
  //                       * Eigen::Quaterniond(0.000849928970774, 0.990093735747, 0.000676013130809, 0.140403757288);
  
  
/*
  group.setPoseTarget(pose);
  group.move();

  move_group_interface::MoveGroupInterface::Plan plan;
  bool success = group.plan(plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    //non blocking request
  group.asyncExecute(plan); 
  // cancel motion after some times
  sleep(0.1);
  group.stop();
  sleep(1.0); //wait for stop command*/
}

void print_poses_space( std::map<std::string, geometry_msgs::Pose> poses_map )
{
  for ( auto view_ : poses_map )
  {
    std::cout<<view_.first<<" = "<<view_.second.position.x<<" "<<
                                   view_.second.position.y<<" "<<
                                   view_.second.position.z<<" "<<
                                   view_.second.orientation.x<<" "<<
                                   view_.second.orientation.y<<" "<<
                                   view_.second.orientation.z<<" "<<
                                   view_.second.orientation.w<<" "<<"\n";
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "yb_moveit_commander");
  //moveit::planning_interface::MoveGroup group("arm_1");
  //moveit::planning_interface::MoveGroupInterface move_group("arm_1");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Create client for yb_ros_server
  ros::NodeHandle nh;
  //ros::ServiceClient yb_ros_client_ = nh.serviceClient<ig_active_reconstruction_msgs::yb_move_arm_using_joints>("youbot/move_to");
  //ig_active_reconstruction_msgs::yb_move_arm_using_joints srv_;

  //Load file from param
  std::string yb_joints_space_file_path;
  std::string yb_poses_space_file_path;
  ros_tools::getExpParam(yb_joints_space_file_path,"yb_joints_space_file_path");
  ros_tools::getExpParam(yb_poses_space_file_path,"yb_poses_space_file_path");

  // generate view space using joints
  namespace iar_yb = ig_active_reconstruction_youbot;
  iar_yb::views::ViewSpace view_space;

  view_space.set_joints_space( yb_joints_space_file_path );

  view_space.set_poses_space( yb_poses_space_file_path );

  ig_active_reconstruction_youbot::robot::RosServerYoubot yb_comm_unit( nh, view_space.get_joints_map(), view_space.get_poses_map() );

  //std::map<std::string, geometry_msgs::Pose> poses_map = view_space.get_poses_map();
  
  //print_poses_space(poses_map);    

  ros::spin();

}