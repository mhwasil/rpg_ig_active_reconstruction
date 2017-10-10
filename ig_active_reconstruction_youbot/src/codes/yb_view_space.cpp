
#include "ig_active_reconstruction_youbot/yb_view_space.hpp"
#include <string>
#include <ros/ros.h>
#include <iostream>

namespace ig_active_reconstruction_youbot
{
namespace views
{
  void ViewSpace::get_joints_space( std::string yb_viewspace_file_path )
  {

    std::ifstream in( yb_viewspace_file_path, std::ifstream::in );
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

      std::string id = std::to_string(id_);
      ViewSpace::set_joints_map(joint_0, joint_1, joint_2, joint_3, joint_4, id);
      id_ += 2;
    }
  }

  void ViewSpace::set_joints_map( double joint_0, double joint_1, double joint_2, double joint_3, double joint_4 , std::string id)
  {
    std::map<std::string, double> wrapper;
    wrapper["arm_joint_1"] = joint_0;
    wrapper["arm_joint_2"] = joint_1;
    wrapper["arm_joint_3"] = joint_2;
    wrapper["arm_joint_4"] = joint_3;
    wrapper["arm_joint_5"] = joint_4;
    joints_map_.insert(std::pair<std::string, std::map<std::string, double>>(id,wrapper));
    //joints_map_[id] = wrapper;
  }

  std::map<std::string, std::map<std::string, double> > ViewSpace::get_joints_map()
  {
    return joints_map_;
  }

}
}