/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich, Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * ig_active_reconstruction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * Please refer to the GNU Lesser General Public License for details on the license,
 * on <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "std_msgs/Int32.h"

namespace ig_active_reconstruction
{
  
namespace robot
{

/*! Abstract interface definition for robot interface implementations: Those can be communication units
 * (e.g. RosRemote and RosReceiver in the ig_active_reconstruction_ros package) or implementations that
 * forward the commands to a "robot" (e.g. FlyingStereoCamCommUnit in the flying_stereo_cam package)
 */
class YoubotCommunicationInterface
{
public:
  /*! Reception info
   */
  enum struct ReceptionInfo
  { 
    SUCCEEDED=0, 
    FAILED=1
  };
  
  public:
    /*! Constructor
     * @param nh_sub ROS node handle defines the namespace in which ROS communication will be carried out for any topic or service subscribers.
     */
    virtual ~YoubotCommunicationInterface(){};

    virtual std_msgs::Int32 runObjectDetector(std_msgs::Int32)=0;

};

}

}