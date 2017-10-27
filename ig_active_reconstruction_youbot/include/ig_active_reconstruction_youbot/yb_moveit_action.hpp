#pragma once

#include <string>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "geometry_msgs/PoseStamped.h"
#include "ig_active_reconstruction_msgs/youbotMoveToOrder.h"
#include <moveit/move_group_interface/move_group.h>

namespace ig_active_reconstruction_youbot
{
namespace robot
{
  class YoubotAction
  {
  public:
    /*! Constructor
     * @param nh ROS node handle defines the namespace in which ROS communication will be carried out.
     * 
     */
    YoubotAction( ros::NodeHandle nh );

  private:
    moveArmUsingJoints( std::map<std::string, double> joints_map );

  protected:
    ros::NodeHandle nh_;
    std::map<std::string, std::map<std::string, double>> joints_map_;
    
  };

}
}