#pragma once

#include <string>
#include <ros/ros.h>
#include <iostream>
#include <fstream>

namespace ig_active_reconstruction_youbot
{
namespace views
{
  class ViewSpace
  {
  public:
    void get_joints_space( std::string file_name );

    std::map<std::string, std::map<std::string, double> > get_joints_map();

  private:
    void set_joints_map( double joint_0, double joint_1, double joint_2, double joint_3, double joint_4, std::string id );

  protected:
    ros::NodeHandle nh_;
    std::map<std::string, std::map<std::string, double>> joints_map_;
    
  };

}
}