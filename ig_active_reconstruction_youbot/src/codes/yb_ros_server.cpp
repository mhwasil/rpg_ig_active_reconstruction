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
#include "ros/ros.h"
#include <fstream>
#include <stdexcept>
//#include "ig_active_reconstruction_ros/robot_ros_server_ci.hpp"
//#include "ig_active_reconstruction_ros/robot_conversions.hpp"
//#include "ig_active_reconstruction_ros/views_conversions.hpp"

namespace ig_active_reconstruction_youbot
{

namespace robot
{

  RosServerYoubot::RosServerYoubot ( ros::NodeHandle nh, std::map<std::string, std::map<std::string, double> > joints_map )
  {
    joints_map_ = joints_map;
    robot_moving_service_ = nh.advertiseService("youbot/move_to", &RosServerYoubot::moveToService, this );
    robot_moving_to_joints_service_ = nh.advertiseService("youbot/move_to_joints", &RosServerYoubot::moveToJointsService, this );
  }

/*  std::map<std::string, double> RosServerYoubot::get_joints_map()
  {
    return joints_map_;
  }*/

	bool RosServerYoubot::moveToService(ig_active_reconstruction_msgs::yb_move_arm_using_joints::Request  &req,
            ig_active_reconstruction_msgs::yb_move_arm_using_joints::Response &res)
  {

    moveit::planning_interface::MoveGroup group("arm_1");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // group.setPlanningTime(10);
    // ROS_INFO_NAMED("Arm_1", "Reference frame: %s", group.getPlanningFrame().c_str());
    // ROS_INFO_NAMED("Arm_1", "End effector link: %s", group.getEndEffectorLink().c_str());

    //std::map<std::string, double> joints_req = RosServerYoubot::joints_wrapper( req.joints );

    std::map<std::string, double> wrapper;
    wrapper["arm_joint_1"] = req.joint_0;
    wrapper["arm_joint_2"] = req.joint_1;
    wrapper["arm_joint_3"] = req.joint_2;
    wrapper["arm_joint_4"] = req.joint_3;
    wrapper["arm_joint_5"] = req.joint_4;

    group.setJointValueTarget( wrapper );
    group.move();

    move_group_interface::MoveGroup::Plan plan;
    if (!group.plan(plan))
    {
      ROS_FATAL("Cannot execute motion. No motion plan found. Aborting.");
      exit(-1);
      res.success = false;
    }
    res.success = true;
    //non blocking request
    group.asyncExecute(plan); 
    // cancel motion after some times
    sleep(0.1);
    group.stop();
    sleep(1.0); //wait for stop command

    return true;
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

    return true;
  }

  bool RosServerYoubot::moveArmUsingJoints( std::map<std::string, double> joints_map )
  {

    moveit::planning_interface::MoveGroup group("arm_1");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    group.setPlanningTime(10);
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