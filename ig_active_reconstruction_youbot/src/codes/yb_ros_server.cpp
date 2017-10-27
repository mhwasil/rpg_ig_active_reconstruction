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
 * 
 * Expanded to youbot implementation by emwe
*/


#include <ig_active_reconstruction_msgs/yb_move_arm_using_joints.h>
#include <ig_active_reconstruction_msgs/ybMoveToJoints.h>
#include "ig_active_reconstruction_youbot/yb_ros_server.hpp"
#include <moveit/move_group_interface/move_group.h>
#include "ig_active_reconstruction_msgs/youbotMoveToOrder.h"
#include "ros/ros.h"
#include <fstream>
#include <stdexcept>
#include "geometry_msgs/PoseStamped.h"
//#include "ig_active_reconstruction_ros/robot_ros_server_ci.hpp"
//#include "ig_active_reconstruction_ros/robot_conversions.hpp"
//#include "ig_active_reconstruction_ros/views_conversions.hpp"

namespace ig_active_reconstruction_youbot
{

namespace robot
{

  RosServerYoubot::RosServerYoubot ( ros::NodeHandle nh, std::map<std::string, std::map<std::string, double> > joints_map,
                                      std::map<std::string, geometry_msgs::Pose> poses_map )
  {
    joints_map_ = joints_map;
    poses_map_ = poses_map;
    robot_moving_service_ = nh.advertiseService("youbot/move_to", &RosServerYoubot::moveToService, this );
    //robot_moving_to_joints_service_ = nh.advertiseService("youbot/move_to_joints", &RosServerYoubot::moveToJointsService, this );
  }

	bool RosServerYoubot::moveToService( ig_active_reconstruction_msgs::youbotMoveToOrder::Request& req, 
                                    ig_active_reconstruction_msgs::youbotMoveToOrder::Response& res )
  {

    geometry_msgs::Pose pose;
    pose = req.pose;
    std::string id = RosServerYoubot::poseToJoints( pose );
    bool success = RosServerYoubot::moveArmUsingJoints( joints_map_.at(id) );
    res.success = success;
    return res.success;
  }

  //return joints cnfiguration
  std::string RosServerYoubot::poseToJoints( geometry_msgs::Pose pose )
  {
    geometry_msgs::Pose ordered_pose;
    ordered_pose = pose;
    std::string key;
    geometry_msgs::Pose reference_pose;
    for( auto pm : poses_map_)
    {
      reference_pose = pm.second;
      if ( ordered_pose.position.x == reference_pose.position.x &&
           ordered_pose.position.y == reference_pose.position.y &&
           ordered_pose.position.z == reference_pose.position.z ) 
      {
        key = pm.first;
        break;
      }
    }

    return key;
  }


  bool RosServerYoubot::moveToJointsService( ig_active_reconstruction_msgs::ybMoveToJoints::Request& req, 
                                    ig_active_reconstruction_msgs::ybMoveToJoints::Response& res )
  {
    ///ig_active_reconstruction_youbot::views::ViewSpace view_space;
    std::string id = req.id;
    std::cout<<"Id = "<< id;
    //joints_map_ = view_space.get_joints_map().at(id) ;
    res.success = RosServerYoubot::moveArmUsingJoints( joints_map_.at(id) );
    /*for ( auto view_ : joints_map_ )
    {
        std::cout<<view_.first<<" = "<<view_.second<<"\n";
    }*/

    return res.success;
  }

  bool RosServerYoubot::moveArmUsingJoints( std::map<std::string, double> joints_map )
  {

    moveit::planning_interface::MoveGroup group("arm_1");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //group.setPlanningTime(10);
    ROS_INFO_NAMED("Arm_1", "Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("Arm_1", "End effector link: %s", group.getEndEffectorLink().c_str());

    group.setJointValueTarget( joints_map );
    group.move();

    move_group_interface::MoveGroup::Plan plan;
    if (!group.plan(plan))
    {
      ROS_FATAL("Cannot execute motion. No motion plan found. Aborting.");
      exit(-1);
      return false;
    }
    
    //non blocking request
    group.asyncExecute(plan); 
    
    // cancel motion after some times
    sleep(0.1);
    group.stop();
    sleep(1.0); //wait for stop command

    return true;
  }

}

}