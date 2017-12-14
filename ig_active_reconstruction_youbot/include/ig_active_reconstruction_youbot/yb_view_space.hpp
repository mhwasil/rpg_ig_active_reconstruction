#pragma once

#include <string>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "geometry_msgs/PoseStamped.h"
#include "ig_active_reconstruction_msgs/youbotMoveToOrder.h"

namespace ig_active_reconstruction_youbot
{
namespace views
{
  class ViewSpace
  {
  public:
    /*! Constructor
     * @param nh ROS node handle defines the namespace in which ROS communication will be carried out.
     * 
     */
    //ViewSpace( ros::NodeHandle nh );


    void set_joints_space( std::string file_name );

    void set_poses_space( std::string file_name );

    void set_workstations( std::string file_name );

    std::map<int, std::map<std::string, double> > get_joints_map();

    std::map<int, geometry_msgs::Pose > get_poses_map();

    std::map<int, std::string> get_workstations_map();

  private:
    void set_joints_map( double joint_0, double joint_1, double joint_2, double joint_3, double joint_4, int id );

    void set_poses_map( geometry_msgs::Pose& pose, int id ); 

    void set_workstations_map( int id, std::string ws );

  protected:
    ros::NodeHandle nh_;
    std::map<int, std::map<std::string, double>> joints_map_;
    std::map<int, geometry_msgs::Pose> poses_map_;
    std::map<int, std::string> workstations_map_;
    ros::ServiceServer robot_moving_to_joints_service_;
  };

}
}