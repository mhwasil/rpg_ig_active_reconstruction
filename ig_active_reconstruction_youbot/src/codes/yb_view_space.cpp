
#include "ig_active_reconstruction_youbot/yb_view_space.hpp"
#include <string>
#include <ros/ros.h>
#include <iostream>

namespace ig_active_reconstruction_youbot
{
namespace views
{

  /*ViewSpace::ViewSpace ( ros::NodeHandle nh )
  {
    nh_ = nh;
    //robot_moving_service_ = nh.advertiseService("youbot/move_to", &RosServerYoubot::moveToService, this );
    robot_moving_to_joints_service_ = nh.advertiseService("youbot/move_to_joints", &ViewSpace::moveToJointsService, this );
  }
*/
  void ViewSpace::set_joints_space( std::string yb_jointspace_file_path )
  {

    std::ifstream in( yb_jointspace_file_path, std::ifstream::in );
    unsigned int nr_of_views;
    bool success = static_cast<bool>(in >> nr_of_views);

    double joint_0;
    double joint_1;
    double joint_2;
    double joint_3;
    double joint_4;

    int id_ = 0;
    for( unsigned int i=0; i<nr_of_views; ++i )
    {
      success = success && ( in>>joint_0 );
      success = success && ( in>>joint_1 );
      success = success && ( in>>joint_2 );
      success = success && ( in>>joint_3 );
      success = success && ( in>>joint_4 );

      //std::string id = std::to_string(id_);
      ViewSpace::set_joints_map(joint_0, joint_1, joint_2, joint_3, joint_4, id_);
      id_ += 2;
    }
  }

  void ViewSpace::set_poses_space( std::string yb_posespace_file_path )
  {
    std::ifstream in( yb_posespace_file_path, std::ifstream::in );
    unsigned int nr_of_views;
    bool success = static_cast<bool>(in >> nr_of_views);

    geometry_msgs::Pose pose;

    int id_ = 0;
    for( unsigned int i=0; i<nr_of_views; ++i )
    {
      success = success && ( in>>pose.position.x );
      success = success && ( in>>pose.position.y );
      success = success && ( in>>pose.position.z );
      success = success && ( in>>pose.orientation.x );
      success = success && ( in>>pose.orientation.y );
      success = success && ( in>>pose.orientation.z );
      success = success && ( in>>pose.orientation.w );

      //std::string id = std::to_string(id_);
      ViewSpace::set_poses_map( pose, id_ );
      id_ += 2;
    }
  }

  void ViewSpace::set_workstations(std::string yb_workstations_file_path)
  {
    std::ifstream in(yb_workstations_file_path, std::ifstream::in);
    unsigned int nr_of_views;
    bool success = static_cast<bool>(in >> nr_of_views);

    std::string ws;

    int id_ = 0;
    for (unsigned int i = 0; i < nr_of_views; ++i)
    {
      success = success && (in >> ws);
      //std::string id = std::to_string(id_);
      ViewSpace::set_workstations_map(id_, ws);
      id_ += 2;
    }
  }

  void ViewSpace::set_joints_map( double joint_0, double joint_1, double joint_2, double joint_3, double joint_4 , int id)
  {
    std::map<std::string, double> wrapper;
    wrapper["arm_joint_1"] = joint_0;
    wrapper["arm_joint_2"] = joint_1;
    wrapper["arm_joint_3"] = joint_2;
    wrapper["arm_joint_4"] = joint_3;
    wrapper["arm_joint_5"] = joint_4;
    //joints_map_.insert(std::pair<std::string, std::map<std::string, double>>(id,wrapper));
    joints_map_[id] = wrapper;
  }

  void ViewSpace::set_poses_map( geometry_msgs::Pose& pose, int id )
  {
    ///poses_map_.insert(std::pair<int, geometry_msgs::Pose>(id, pose));
    poses_map_[id] = pose;
  }

  void ViewSpace::set_workstations_map(int id, std::string ws)
  {
    workstations_map_[id] = ws;
  }

  std::map<int, std::map<std::string, double> > ViewSpace::get_joints_map()
  {
    return joints_map_;
  }

  std::map<int, geometry_msgs::Pose > ViewSpace::get_poses_map()
  {
    return poses_map_;
  }

  std::map<int, std::string> ViewSpace::get_workstations_map()
  {
    return workstations_map_;
  }

}
}