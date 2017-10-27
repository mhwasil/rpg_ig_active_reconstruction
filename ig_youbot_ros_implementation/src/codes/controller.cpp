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

#include "ig_youbot_ros_implementation/controller.hpp"
#include "ig_active_reconstruction_msgs/youbotMoveToOrder.h"

#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include <movements/ros_movements.h>
#include <stdexcept>

namespace ig_youbot_ros_implementation
{
  
  Controller::Controller(std::string cam_model_name, std::string yb_model_name, std::string yb_cam3d_frame_name, std::string yb_world_frame_name )
  : cam_model_name_(cam_model_name)
  , yb_model_name_(yb_model_name)
  , yb_cam3d_frame_name_(yb_cam3d_frame_name)
  , yb_world_frame_name_(yb_world_frame_name)
  , has_moved_(false)
  , keepPublishing_(false)
  , cam_to_image_(0.5,0.5,-0.5,0.5)
  {
    
  }
  
  Controller::~Controller()
  {
    keepPublishing_=false;
    if(publisher_.joinable())
      publisher_.join();
  }
  
  bool Controller::moveTo( movements::Pose new_pose )
  {
    // call youbot service
    geometry_msgs::Pose pose;
    pose = movements::toROS( new_pose );
    ig_active_reconstruction_msgs::youbotMoveToOrder yb_srv;
    yb_srv.request.pose = pose;

    bool yb_response = ros::service::call( "/youbot/move_to", yb_srv );

    bool success = false;

    if (yb_srv.response.success)
    {
      ROS_INFO("YOUBOT MOVEMENT EXECUTED");
    } 
    else
    {
      ROS_INFO("YOUBOT MOVEMENT FAILED");
    }  

    //return srv_call.response.success;
    return yb_srv.response.success;
    
  }
  
  movements::Pose Controller::currentPose()
  {

    tf::TransformListener tf_listener;
    bool tf_received = false;
    tf::StampedTransform transform;
    while(!tf_received)
    {
      try
      {
        ros::Time now = ros::Time(0);
        tf_listener.waitForTransform(yb_cam3d_frame_name_, yb_world_frame_name_, now, ros::Duration(3.0));
        tf_listener.lookupTransform(yb_cam3d_frame_name_, yb_world_frame_name_, now, transform);
        std::cout<<"\nTRANSFORM RESULT \n";
        std::cout<<"x: "<<transform.getOrigin().x()<<"\n";
        std::cout<<"y: "<<transform.getOrigin().y()<<"\n";
        std::cout<<"z: "<<transform.getOrigin().z()<<"\n";
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
  
  void Controller::startTfPublisher(std::string yb_cam3d_frame_name, std::string yb_world_frame_name)
  {
    keepPublishing_ = true;
    publisher_ = std::thread(&Controller::keepPublishing,this,yb_cam3d_frame_name,yb_world_frame_name);
  }
  
  void Controller::stopTfPublisher()
  {
    keepPublishing_=false;
  }
  
  void Controller::keepPublishing(std::string yb_cam3d_frame_name, std::string yb_world_frame_name)
  {
    while(keepPublishing_)
    {
      if(true||has_moved_)
      {
        geometry_msgs::Pose pose;
        {
          std::lock_guard<std::mutex> guard(protector_);
  	      pose = movements::toROS( currentPose()/*current_pose_*/ );
  	    }
        geometry_msgs::Transform pose_t;
        pose_t.translation.x = pose.position.x;
        pose_t.translation.y = pose.position.y;
        pose_t.translation.z = pose.position.z;
        pose_t.rotation.x = pose.orientation.x;
        pose_t.rotation.y = pose.orientation.y;
        pose_t.rotation.z = pose.orientation.z;
        pose_t.rotation.w = pose.orientation.w;
  	 
        tf::Transform cam_2_origin;
        tf::transformMsgToTF( pose_t, cam_2_origin );
        tf_broadcaster_.sendTransform(tf::StampedTransform(cam_2_origin, ros::Time::now(), yb_cam3d_frame_name, yb_world_frame_name));
      }
      ros::Duration(0.05).sleep();
    }
  }
  
}