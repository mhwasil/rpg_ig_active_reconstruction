/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich, Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * ig_active_reconstruction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * Please refer to the GNU Lesser General Public License for details on the license,
 * on <http://www.gnu.org/licenses/>.
*/


#include <ros/ros.h>

#include <ig_active_reconstruction_ros/param_loader.hpp>
#include <ig_active_reconstruction_ros/robot_ros_server_ci.hpp>
#include <ig_active_reconstruction_youbot/yb_ros_server.hpp>
#include "ig_youbot_ros_implementation/robot_communication_interface.hpp"


/*! Implements a ROS node interface to a "flying" (staticly placed) gazebo stereo camera.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_interface");
  ros::NodeHandle nh;
  
  // Load parameters
  //------------------------------------------------------------------
  std::string model_name, camera_frame_name, world_frame_name, sensor_in_topic, sensor_out_name;
  ros_tools::getExpParam(model_name,"model_name");
  ros_tools::getExpParam(camera_frame_name,"camera_frame_name");
  ros_tools::getExpParam(world_frame_name,"world_frame_name");
  ros_tools::getExpParam(sensor_in_topic,"sensor_in_topic");
  ros_tools::getExpParam(sensor_out_name,"sensor_out_name");
  
  std::string yb_model_name, yb_cam3d_frame_name, yb_world_frame_name; 
  ros_tools::getExpParam(yb_model_name,"yb_model_name");
  ros_tools::getExpParam(yb_cam3d_frame_name,"yb_cam3d_frame_name");
  ros_tools::getExpParam(yb_world_frame_name,"yb_world_frame_name");

  using namespace ig_youbot_ros_implementation;
  
  // Controller
  //------------------------------------------------------------------
  std::shared_ptr<Controller> controller = std::make_shared<Controller>(model_name, yb_model_name, yb_cam3d_frame_name, yb_world_frame_name); //tell controller the model_name or yb_model_name
  //std::shared_ptr<ybController> yb_controller = std::make_shared<ybController>(yb_model_name);
  
  // publish tf
  controller->startTfPublisher(yb_cam3d_frame_name, yb_world_frame_name);
  //controller->ybStartTfPublisher(yb_cam3d_frame_name, yb_world_frame_name);
  //yb_controller->startTfPublisher(yb_eef_frame_name,yb_world_frame_name);
  
  // Iar communication interface
  //------------------------------------------------------------------
  boost::shared_ptr<CommunicationInterface> robot_interface = boost::make_shared<CommunicationInterface>(nh,controller,sensor_in_topic,sensor_out_name);
  
  // Expose communication interface to ROS
  //------------------------------------------------------------------
  ig_active_reconstruction::robot::RosServerCI comm_unit(nh,robot_interface);
  //ig_active_reconstruction::robot::YoubotRosServerCI yb_comm_unit(nh);
  
  
  // spin...
  ROS_INFO_STREAM("Flying gazebo stereo camera robot interface is setup.");
  ros::spin();
  
  return 0;
}