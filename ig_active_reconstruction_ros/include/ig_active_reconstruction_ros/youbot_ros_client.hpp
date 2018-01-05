#pragma once

#include "ros/ros.h"
#include "ig_active_reconstruction/youbot_communication_interface.hpp"

namespace ig_active_reconstruction
{
  
namespace robot
{
  
  /*! Generic remote ROS client robot communication interface implementation.
   * 
   * Uses the ROS communication interface (topics, services etc.) to forward requests.
   */
  class YoubotRosClient : public YoubotCommunicationInterface
  {
  public:
    /*! Constructor
     * @param nh_sub ROS node handle defines the namespace in which ROS communication will be carried out for any topic or service subscribers.
     */
    YoubotRosClient( ros::NodeHandle nh_sub );

    virtual std_msgs::Int32 runObjectDetector(std_msgs::Int32);
    
  protected:
    ros::NodeHandle nh_sub_;
    
    ros::ServiceClient object_detector_srv_;
  };
  
}

}