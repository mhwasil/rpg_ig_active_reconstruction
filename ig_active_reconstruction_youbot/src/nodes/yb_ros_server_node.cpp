#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group.h>
#include "ig_active_reconstruction_ros/param_loader.hpp"
#include "ig_active_reconstruction_msgs/yb_move_arm_using_joints.h"
#include "ig_active_reconstruction_youbot/yb_view_space.hpp"
#include <ig_active_reconstruction_youbot/yb_ros_server.hpp>
#include "ig_active_reconstruction_msgs/youbotMoveToInit.h"
#include <fstream>
#include <stdexcept>

void print_poses_space( std::map<int, geometry_msgs::Pose> poses_map )
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

void print_workstations( std::map<int, std::string> workstations_map)
{
  for (auto map_ : workstations_map)
  {
    std::cout<<map_.first<<" = " << map_.second<<"\n";
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yb_ros_server_node");
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
  std::string yb_workstations_file_path;
  std::string start_ws;
  bool ws_constraint = false;
  ros_tools::getExpParam(yb_joints_space_file_path,"yb_joints_space_file_path");
  ros_tools::getExpParam(yb_poses_space_file_path,"yb_poses_space_file_path");
  ros_tools::getExpParam(yb_workstations_file_path, "yb_workstations_file_path");
  ros_tools::getExpParam(start_ws, "start_ws");
  // generate view space using joints
  namespace iar_yb = ig_active_reconstruction_youbot;
  iar_yb::views::ViewSpace view_space;

  view_space.set_joints_space( yb_joints_space_file_path );

  view_space.set_poses_space( yb_poses_space_file_path );

  view_space.set_workstations( yb_workstations_file_path );

  //switch to camera point cloud
  ros::Publisher mux_topic = nh.advertise<std_msgs::String>("/mcr_perception/mux_pointcloud/select", 1);
  
  std_msgs::String cloud_mux_msg;
  //cloud_mux_msg.data = "/arm_cam3d/depth_registered/points";
  cloud_mux_msg.data = "/arm_cam3d/depth_registered/points";
  mux_topic.publish(cloud_mux_msg);

  ig_active_reconstruction_youbot::robot::RosServerYoubot yb_comm_unit( nh, view_space.get_joints_map(), 
                                                                            view_space.get_poses_map(), 
                                                                            view_space.get_workstations_map(),
                                                                            start_ws,
                                                                            ws_constraint );

  ROS_INFO("Youbot server started");

  ROS_INFO("Moving arm to look_at_workspace");

  ig_active_reconstruction_msgs::youbotMoveToInit initMsg;

  initMsg.request.arm_safe_position = "look_at_workspace";
  initMsg.request.source = "START";
  initMsg.request.destination = start_ws;

  /*bool init_state = ros::service::call("/youbot/move_to_init", initMsg);
  if (init_state) 
  {
    ROS_INFO("Youbot already in init state");
  }
  else{
    ROS_INFO("Failed to move youbot to init state");
  }*/

  
  std::map<int, geometry_msgs::Pose> poses_map = view_space.get_poses_map();
  
  print_poses_space(poses_map);

  std::map<int, std::map<std::string, double>> joints_map = view_space.get_joints_map();

  print_joints_space(joints_map);

  std::map<int, std::string> workstations_map = view_space.get_workstations_map();

  print_workstations(workstations_map);
  //  std::map<std::string, double> joints = joints_map.at(0);
  //  for (auto joints_ : joints)
  //  {
  //    std::cout << joints_.second << " ";
  //  }
  //  std::cout << "\n";

  ros::spin();
}
