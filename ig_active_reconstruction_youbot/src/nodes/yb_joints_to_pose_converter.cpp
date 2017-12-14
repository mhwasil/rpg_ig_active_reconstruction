#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "ig_active_reconstruction_ros/param_loader.hpp"
#include "ig_active_reconstruction_msgs/yb_move_arm_using_joints.h"
#include "ig_active_reconstruction_youbot/yb_view_space.hpp"
#include <ig_active_reconstruction_youbot/yb_ros_server.hpp>
#include <movements/ros_movements.h>
#include <fstream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <movements/core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

bool moveArmUsingJoints(std::map<std::string, double> joints_map)
{
  //robot_state::RobotStatePtr current_state;

  moveit::planning_interface::MoveGroup group("arm_1");
  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  
  //set currentState to StartState
  group.setStartStateToCurrentState();
  //group.setPlanningTime(10);
  ROS_INFO_NAMED("Arm_1", "Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("Arm_1", "End effector link: %s", group.getEndEffectorLink().c_str());

  group.setJointValueTarget(joints_map);
  group.move();

  move_group_interface::MoveGroup::Plan plan;
  if (!group.plan(plan))
  {
      ROS_FATAL("Cannot execute motion. No motion plan found. Aborting.");
      //exit(-1);
      return false;
  }

  //non blocking request
  group.asyncExecute(plan);

  //set currentState to StartState - AGAIN
  //group.setStartStateToCurrentState();

  // cancel motion after some times
  sleep(0.1);
  group.stop();
  sleep(1.0); //wait for stop command

  return true;
}

void print_poses_space(std::map<int, movements::Pose> poses_map)
{
  for (auto view_ : poses_map)
  {
    std::cout << view_.first << " = " 
              << view_.second.position(0) << " "
              << view_.second.position(1) << " "
              << view_.second.position(2) << " "
              << view_.second.orientation.x() << " "
              << view_.second.orientation.y() << " "
              << view_.second.orientation.z() << " "
              << view_.second.orientation.w() << " "
              << "\n";
  }
}

void print_joints_space(std::map<int, std::map<std::string, double>> joints_map)
{
  for (auto map_ : joints_map)
  {
      std::cout << map_.first << " = ";
      for (auto joints_ : map_.second)
      {
          std::cout << joints_.second << " ";
      }
      std::cout << "\n";
  }
}

movements::Pose get_transform(std::string yb_cam3d_frame_name_, std::string yb_world_frame_name_)
{
  tf::TransformListener tf_listener;
  bool tf_received = false;
  tf::StampedTransform transform;
  while (!tf_received)
  {
    try
    {
      ros::Time now = ros::Time(0);
      tf_listener.waitForTransform(yb_cam3d_frame_name_, yb_world_frame_name_, now, ros::Duration(1.0));
      tf_listener.lookupTransform(yb_cam3d_frame_name_, yb_world_frame_name_, now, transform);
      //std::cout<<"\nTRANSFORM RESULT \n";
      tf_received = true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  geometry_msgs::Pose pose;
  //tf::poseStampedTFToMsg(transform, pose);
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  pose.orientation.x = transform.getRotation().x();
  pose.orientation.y = transform.getRotation().y();
  pose.orientation.z = transform.getRotation().z();
  pose.orientation.w = transform.getRotation().w();

  movements::Pose current_pose = movements::fromROS(pose);
  //return current_pose_model;
  return current_pose;
}

void save_to_file(std::string filename_, std::map<int, movements::Pose> poses_map_)
{
  std::ofstream out(filename_, std::ofstream::trunc);

  out << poses_map_.size();

  //for( unsigned int i=0; i<view_space_.size(); ++i )
  for (auto &pair : poses_map_)
  {
      //View &view = pair.second;
      out << "\n";
      movements::Pose pose = pair.second; //view_space_[i].pose();
      out << pose.position.x();
      out << " " << pose.position.y();
      out << " " << pose.position.z();
      out << " " << pose.orientation.x();
      out << " " << pose.orientation.y();
      out << " " << pose.orientation.z();
      out << " " << pose.orientation.w();
  }

  out.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yb_joints_to_pose_converter");
  std::cout<<"\nConverter is running! \n";
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
  ros_tools::getExpParam(yb_joints_space_file_path, "yb_joints_space_file_path");
  ros_tools::getExpParam(yb_poses_space_file_path, "yb_poses_space_file_path");
  std::string yb_cam3d_frame_name;
  std::string yb_world_frame_name;
  ros_tools::getExpParam(yb_cam3d_frame_name, "yb_cam3d_frame_name");
  ros_tools::getExpParam(yb_world_frame_name, "yb_world_frame_name");

  // generate view space using joints
  namespace iar_yb = ig_active_reconstruction_youbot;
  iar_yb::views::ViewSpace view_space;

  view_space.set_joints_space(yb_joints_space_file_path);

  //view_space.set_poses_space(yb_poses_space_file_path);

  //ig_active_reconstruction_youbot::robot::RosServerYoubot yb_comm_unit(nh, view_space.get_joints_map(), view_space.get_poses_map());

  //std::map<std::string, geometry_msgs::Pose> joints_map = view_space.get_poses_map();

  //print_poses_space(poses_map);

  std::map<int, std::map<std::string, double>> joints_map = view_space.get_joints_map();
  std::map<int, movements::Pose> poses_map_;
  float sleep_duration = 2.0;
  print_joints_space(joints_map);
  for (auto map_ : joints_map)
  {
    std::map<std::string, double> wrapper;
    //std::string key = map_.first;
    //std::cout << "Int "<<atoi(key.c_str())<<"\n";
    //int key = std::atoi(map_.first.c_str());
    int key = map_.first;
    std::cout<<"\nInt key: "<<key<<"\n";
    wrapper = map_.second;
    bool success = moveArmUsingJoints(wrapper);
    if (success) {
      ros::Duration(sleep_duration).sleep();
      movements::Pose current_pose = get_transform(yb_cam3d_frame_name, yb_world_frame_name);
      poses_map_[key] = current_pose;
      std::cout << "\n Joints conf. " << key << " Success\n";
    }
    else
    {
      std::cout << "\nFailed\n";
    }
  }
  save_to_file(yb_poses_space_file_path, poses_map_);
  print_poses_space(poses_map_);
  std::cout<<"Poses saved"<<"\n";

  ros::spin();

}