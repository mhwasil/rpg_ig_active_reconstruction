/* 
 * 
 * youbot implementation by emwe
*/

#pragma once

#include "ros/ros.h"
#include "ig_active_reconstruction_msgs/yb_move_arm_using_joints.h"
#include "ig_active_reconstruction_msgs/ybMoveToJoints.h"
#include "ig_active_reconstruction_youbot/yb_view_space.hpp"
#include "ig_active_reconstruction_msgs/youbotMoveToOrder.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/rdf_loader/rdf_loader.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "mir_yb_action_msgs/MoveBaseSafeAction.h"
#include "mir_yb_action_msgs/MoveBaseSafeGoal.h"
#include "ig_active_reconstruction_msgs/StringMsgs.h"

namespace ig_active_reconstruction_youbot
{
  
namespace robot
{
  
  /*! Generic "resident" ROS robot communication interface implementation.
   * 
   * Uses the ROS communication interface (topics, services etc.) to receive requests and feed it into
   * a robot::CommunicationInterface implementation.
   */
  class RosServerYoubot
  {
  public:
    /*! Constructor
     * @param nh ROS node handle defines the namespace in which ROS communication will be carried out.
     * @param linked_interface (optional) directly add the interface that is linked internally (to which requests are forwarded.
     */
    RosServerYoubot( ros::NodeHandle nh, std::map<int,std::map<std::string, double> > joints_map,
                       std::map<int, geometry_msgs::Pose> poses_map, std::map<int, std::string> workstations_map, std::string start_ws, bool ws_constraint );

    std::map<std::string, double> get_joints_map();

    void set_joints_map(double joint_0, double joint_1, double joint_2, double joint_3, double joint_4 );


    //bool moveToUsingJoints( views::View& target_view );
    
  protected:    

    bool moveToService( ig_active_reconstruction_msgs::youbotMoveToOrder::Request& req, 
                                    ig_active_reconstruction_msgs::youbotMoveToOrder::Response& res );
    //return joints cnfiguration
    int poseToJoints( geometry_msgs::Pose pose );

     //execute joint using moveit
    bool executeJoints( std::map<std::string, double> joints );

    bool moveToJointsService( ig_active_reconstruction_msgs::ybMoveToJoints::Request& req, ig_active_reconstruction_msgs::ybMoveToJoints::Response& res );

    bool moveArmUsingJoints( std::map<std::string, double> joints_map );

    bool move_base_safe( std::string start, std::string end);

    
  protected:
    ros::NodeHandle nh_;
    //std::map<std::string, double> joints_map_;
    std::map<int, std::map<std::string, double>> joints_map_;
    std::map<int, geometry_msgs::Pose> poses_map_;
    std::map<int, std::string> workstations_map_;
    std::string old_ws_;
    bool ws_constraint_;

    ros::ServiceServer robot_moving_service_;
    ros::ServiceServer robot_moving_to_joints_service_;

  };
  
}

}
