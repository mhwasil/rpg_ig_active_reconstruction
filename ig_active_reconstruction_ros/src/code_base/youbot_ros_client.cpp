#include <stdexcept>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ig_active_reconstruction_msgs/youbotObjectDetector.h"
#include "ig_active_reconstruction/robot_communication_interface.hpp"
#include "ig_active_reconstruction_ros/youbot_ros_client.hpp"


namespace ig_active_reconstruction
{
  
namespace robot
{
  
  YoubotRosClient::YoubotRosClient( ros::NodeHandle nh_sub )
  : nh_sub_(nh_sub)
  {
    object_detector_srv_ = nh_sub_.serviceClient<ig_active_reconstruction_msgs::youbotObjectDetector>("youbot/object_detector");
  }
  
  std_msgs::Int32 YoubotRosClient::runObjectDetector(std_msgs::Int32 obj_number)
  {
    ig_active_reconstruction_msgs::youbotObjectDetector request;

    request.request.obj_number = obj_number;
    
    ROS_INFO("Demanding current view");
    bool response = object_detector_srv_.call(request);
    
    if( !response )
      throw std::runtime_error("RosClientCI::getCurrentView failed for unknown reason.");
    
    return request.response.result;
  }
  
  
  
}

}