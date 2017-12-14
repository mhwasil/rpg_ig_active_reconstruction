#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "mir_yb_action_msgs/MoveBaseSafeAction.h"
#include "mir_yb_action_msgs/MoveBaseSafeGoal.h"
#include "mir_yb_action_msgs/MoveBaseSafeResult.h"
#include "ig_active_reconstruction_ros/param_loader.hpp"
#include "ig_active_reconstruction_msgs/StringMsgs.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "yb_move_base_safe");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Create client for yb_ros_server
  ros::NodeHandle nh;
  std::string start_ws;
  std::string end_ws;

  ros_tools::getExpParam(start_ws, "start_ws");
  ros_tools::getExpParam(end_ws, "end_ws");

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
  ROS_INFO("Action finished: %s",state.toString().c_str());
  std::string result = state.toString();
  if (result == "SUCCEEDED"){
    std::cout << "SUCCEEDED" <<"\n";
  } else {
    std::cout << "FAILED" <<"\n";
  }

  //ros::spin();

}