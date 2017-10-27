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


    std::map<std::string, std::map<std::string, double> > get_joints_map();

    std::map<std::string, geometry_msgs::Pose > get_poses_map();

  private:
    void set_joints_map( double joint_0, double joint_1, double joint_2, double joint_3, double joint_4, std::string id );

    void set_poses_map( geometry_msgs::Pose& pose, std::string id ); 

    bool moveToJointsService( ig_active_reconstruction_msgs::youbotMoveToOrder::Request& req, 
                                    ig_active_reconstruction_msgs::youbotMoveToOrder::Response& res );

  protected:
    ros::NodeHandle nh_;
    std::map<std::string, std::map<std::string, double>> joints_map_;
    std::map<std::string, geometry_msgs::Pose> poses_map_;
    ros::ServiceServer robot_moving_to_joints_service_;
  };

}
}