/* 
 * Youbot server for providing service for youbot/move_to, 
 * Mohammad Wasil RnD project
 * 
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
#include "moveit/robot_model/robot_model.h"
#include "moveit/rdf_loader/rdf_loader.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "mir_yb_action_msgs/MoveBaseSafeAction.h"
#include "mir_yb_action_msgs/MoveBaseSafeGoal.h"
#include "ig_active_reconstruction_msgs/StringMsgs.h"
#include "ig_active_reconstruction_msgs/youbotMoveToInit.h"
//#include "ig_active_reconstruction_ros/robot_ros_server_ci.hpp"
//#include "ig_active_reconstruction_ros/robot_conversions.hpp"
//#include "ig_active_reconstruction_ros/views_conversions.hpp"

namespace ig_active_reconstruction_youbot
{

namespace robot
{

  RosServerYoubot::RosServerYoubot ( ros::NodeHandle nh, std::map<int, std::map<std::string, double> > joints_map,
                                      std::map<int, geometry_msgs::Pose> poses_map, std::map<int, std::string> workstations_map, std::string start_ws, bool ws_constraint )
  {
    joints_map_ = joints_map;
    poses_map_ = poses_map;
    workstations_map_ = workstations_map;
    old_ws_ = start_ws;
    ws_constraint_ = ws_constraint;
    robot_moving_service_ = nh.advertiseService("youbot/move_to", &RosServerYoubot::moveToService, this );
    robot_moving_to_init_service_ = nh.advertiseService("youbot/move_to_init", &RosServerYoubot::move_base_safe_to_init, this);

    //robot_moving_to_joints_service_ = nh.advertiseService("youbot/move_to_joints", &RosServerYoubot::moveToJointsService, this );
  }

	bool RosServerYoubot::moveToService( ig_active_reconstruction_msgs::youbotMoveToOrder::Request& req, 
                                    ig_active_reconstruction_msgs::youbotMoveToOrder::Response& res )
  {

    geometry_msgs::Pose pose;
    pose = req.pose;
    //std::string id = RosServerYoubot::poseToJoints( pose );
    
    int id = RosServerYoubot::poseToJoints(pose);
    if (ws_constraint_ == true)
    {
      if (workstations_map_.at(id) == old_ws_)
      {
        bool success = RosServerYoubot::moveArmUsingJoints(joints_map_.at(id));
        res.success = success;
      }
      else
      { 
        std::string new_ws = workstations_map_.at(id);
        //do youbot nav movement
        //nav_movement_fb = nav_movement(old_ws_, new_ws_)
        bool nav_result = RosServerYoubot::move_base_safe(old_ws_, new_ws);
        
        //res.success = nav_result;
        if (nav_result) {
          bool success = RosServerYoubot::moveArmUsingJoints(joints_map_.at(id));
          res.success = success;
          old_ws_ = new_ws;
        } else {
          //keep old_ws and still return true in the ws
          res.success = true;
        }
      }
    }
    else
    {
      bool success = RosServerYoubot::moveArmUsingJoints(joints_map_.at(id));
      res.success = success;
    }
    
    //res.success = true;
    return res.success;
  }

  //return joints cnfiguration
  int RosServerYoubot::poseToJoints( geometry_msgs::Pose pose )
  {
    geometry_msgs::Pose ordered_pose;
    ordered_pose = pose;
    //std::string key;
    int key = 0;
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

  bool RosServerYoubot::move_base_safe(std::string start_ws, std::string end_ws)
  {
    ig_active_reconstruction_msgs::StringMsgs msg;

    msg.arm_safe_position = "barrier_tape";
    msg.source = start_ws;
    msg.destination = end_ws;

    actionlib::SimpleActionClient<mir_yb_action_msgs::MoveBaseSafeAction> client("move_base_safe_server", true);
    ROS_INFO("Waiting for MoveBaseSafeServer to start.");
    client.waitForServer(); //will wait for infinite time

    ROS_INFO("MoveBaseSafeServer started, sending goal.");
    mir_yb_action_msgs::MoveBaseSafeGoal goal;
    goal.arm_safe_position = msg.arm_safe_position;
    goal.source_location = msg.source;
    goal.destination_location = msg.destination;

    int timeout = 30;
    ROS_INFO("Sending action lib goal to move_base_safe_server");
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(timeout));
    client.cancelGoal();
    
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("MoveBaseSafe server finished: %s",state.toString().c_str());
    std::string result = state.toString();

    bool nav_result = false;
    if (result == "SUCCEEDED"){
      nav_result = true;
    } else {
      nav_result = false;
    }

    return nav_result;
  }

  bool RosServerYoubot::moveToJointsService( ig_active_reconstruction_msgs::ybMoveToJoints::Request& req, 
                                    ig_active_reconstruction_msgs::ybMoveToJoints::Response& res )
  {
   
    std::string id_ = req.id;
    int id = std::stoi(id_);
    std::cout<< "Id = " << id;
    //joints_map_ = view_space.get_joints_map().at(id) ;
    res.success = RosServerYoubot::moveArmUsingJoints( joints_map_.at(id) );
    return res.success;
  }

  bool RosServerYoubot::moveArmUsingJoints( std::map<std::string, double> joints_map )
  {
    robot_state::RobotStatePtr current_state;
    moveit::planning_interface::MoveGroup group("arm_1");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //set currentState to StartState
    group.setStartStateToCurrentState();
    
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
    
    //set currentState to StartState
    //group.setStartStateToCurrentState();

    // cancel motion after some times
    sleep(0.1);
    group.stop();
    sleep(1.0); //wait for stop command

    return true;
  }

  bool RosServerYoubot::move_base_safe_to_init(ig_active_reconstruction_msgs::youbotMoveToInit::Request &req,
                                             ig_active_reconstruction_msgs::youbotMoveToInit::Response &res)
  {
    actionlib::SimpleActionClient<mir_yb_action_msgs::MoveBaseSafeAction> client("move_base_safe_server", true);
    ROS_INFO("Waiting for MoveBaseSafeServer to start.");
    client.waitForServer(); //will wait for infinite time

    ROS_INFO("MoveBaseSafeServer started, sending goal.");
    mir_yb_action_msgs::MoveBaseSafeGoal goal;
    goal.arm_safe_position = req.arm_safe_position;
    goal.source_location = req.source;
    goal.destination_location = req.destination;

    int timeout = 30;
    ROS_INFO("Sending action lib goal to move_base_safe_server");
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(timeout));
    client.cancelGoal();

    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("MoveBaseSafe server finished: %s", state.toString().c_str());
    std::string result = state.toString();

    //bool res.success = false;
    if (result == "SUCCEEDED")
    {
      res.success = true;
    }
    else
    {
      res.success = false;
    }

    return res.success;
  }

}

}
