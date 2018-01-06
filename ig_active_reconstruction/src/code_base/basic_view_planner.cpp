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

#include "ig_active_reconstruction/basic_view_planner.hpp"

#include <chrono>
#include <boost/smart_ptr.hpp>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/chrono/include.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <string> 
#include "ig_active_reconstruction_msgs/ybMoveToJoints.h"


namespace ig_active_reconstruction
{
  
  BasicViewPlanner::Config::Config()
  : discard_visited(true)
  , max_visits(1)
  , random_view(false)
  {
  }
  
  BasicViewPlanner::BasicViewPlanner(Config config)
  : config_(config)
  , robot_comm_unit_(nullptr)
  , youbot_comm_unit_(nullptr)
  , views_comm_unit_(nullptr)
  , world_comm_unit_(nullptr)
  , utility_calculator_(nullptr)
  , goal_evaluation_module_(nullptr)
  , status_(Status::UNINITIALIZED)
  , runProcedure_(false)
  , pauseProcedure_(false)
  {
    
  }

  /* BasicViewPlanner::BasicViewPlanner(std::map<int, std::string> workstations_map)
  {

    workstations_map_ = workstations_map;
  }
 */
  BasicViewPlanner::~BasicViewPlanner()
  {
    runProcedure_ = false;
    if( running_procedure_.joinable() )
      running_procedure_.join();
  }
  
  void BasicViewPlanner::setRobotCommUnit( boost::shared_ptr<robot::CommunicationInterface> robot_comm_unit )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    robot_comm_unit_ = robot_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }

  void BasicViewPlanner::setYobotCommUnit( boost::shared_ptr<robot::YoubotCommunicationInterface> youbot_comm_unit )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    youbot_comm_unit_ = youbot_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BasicViewPlanner::setViewsCommUnit( boost::shared_ptr<views::CommunicationInterface> views_comm_unit)
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    views_comm_unit_ = views_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BasicViewPlanner::setWorldCommUnit( boost::shared_ptr<world_representation::CommunicationInterface> world_comm_unit )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    world_comm_unit_ = world_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BasicViewPlanner::setUtility( boost::shared_ptr<UtilityCalculator> utility_calculator )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    utility_calculator_ = utility_calculator;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BasicViewPlanner::setGoalEvaluationModule( boost::shared_ptr<GoalEvaluationModule> goal_evaluation_module )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    goal_evaluation_module_ = goal_evaluation_module;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  bool BasicViewPlanner::run()
  {
    if( runProcedure_ || running_procedure_.joinable() )
    {
      if( pauseProcedure_ )
      {
        pauseProcedure_ = false;
        return true;
      }
      return false;
    }
    
    runProcedure_ = true;
    running_procedure_ = std::thread(&BasicViewPlanner::main, this);
    
    return true;
  }
  
  void BasicViewPlanner::pause()
  {
    pauseProcedure_ = true;
  }
  
  void BasicViewPlanner::stop()
  {
    runProcedure_ = false;
  }
  
  BasicViewPlanner::Status BasicViewPlanner::status()
  {
    return status_;
  }
  
  void BasicViewPlanner::set_workstations_map(std::map<int, std::string> workstations_map)
  {
    //if( runProcedure_ || running_procedure_.joinable() )
    //  return;
    workstations_map_ = workstations_map;
    //ROS_INFO("Workstations map is ready");
 
    /*if( isReady() && status ==Status::UNINITIALIZED )
        status_ = Status::IDLE;
    else
        status_ = Status::UNINITIALIZED;*/
  }

  bool BasicViewPlanner::isReady()
  {
    return robot_comm_unit_!=nullptr
  	&& views_comm_unit_!=nullptr
  	&& world_comm_unit_!=nullptr
  	&& utility_calculator_!=nullptr
  	&& goal_evaluation_module_!=nullptr;
  }
  
  void BasicViewPlanner::obj_detector_cb(std_msgs::Int32 data)
  {
    detected_obj_num_ = data.data;
    ROS_INFO("Detected object updated");
    std::cout<<detected_obj_num_<<"\n";
    counted_ = true;
  }
   
  void BasicViewPlanner::main()
  {    
    //node handle
    ros::NodeHandle nh;

    BasicViewPlanner bvp;

    old_ws="WS01";
    detected_obj_num_ = 0;
    counted_ = false;
    //define publisher for e_add_cloud_start
    ros::Publisher client_e_add_cloud_start = nh.advertise<std_msgs::String>("/mcr_perception/cloud_accumulator/event_in", 1);
    
    // preparation
    goal_evaluation_module_->reset();
    
    //these variablesjust for testing
    unsigned int number_idle = 0;

    // get viewspace................................................
    viewspace_ = boost::make_shared<views::ViewSpace>();
    views::CommunicationInterface::ViewSpaceStatus viewspace_status;
    do
    {
      status_ = Status::DEMANDING_VIEWSPACE;
      *viewspace_ = views_comm_unit_->getViewSpace();
      
      if( !runProcedure_ ) // exit point
    	{
    	  status_ = Status::IDLE;
    	  runProcedure_ = false;
    	  return;
    	}
      pausePoint();
      number_idle++;
      
    }while( viewspace_->empty());
    
    unsigned int reception_nr = 0;
    unsigned int moving_nr = 0;
    int iteration_number = 0;
    
    do
    {
      // determine view candidate subset of viewspace .....................
      views::ViewSpace::IdSet view_candidate_ids;
      viewspace_->getGoodViewSpace(view_candidate_ids, config_.discard_visited);
      if (view_candidate_ids.empty())
      {
        break;
      }

      //std::string old_ws;
/*       views::ViewSpace::IdSet new_view_candidate_ids;
      if(iteration_number == 0)
      {
        new_view_candidate_ids = view_candidate_ids;
      }
      else
      {
        if (old_ws == "WS01")
        {
          for (unsigned int i = 0; i < view_candidate_ids.size(); ++i)
          {
            if (view_candidate_ids[i] > 12)
            {
              new_view_candidate_ids.push_back(view_candidate_ids[i]);
            }

          }
          {
            if (view_candidate_ids[i] > 12)
            {
              new_view_candidate_ids.push_back(view_candidate_ids[i]);
            }

          }
        }
        else if (old_ws == "WS02")
        {
          for (unsigned int i = 0; i < view_candidate_ids.size(); ++i)
          {
            if (view_candidate_ids[i] < 12 || view_candidate_ids[i] >= 24)
            {
              new_view_candidate_ids.push_back(view_candidate_ids[i]);
            }
          }
        }
        else if (old_ws == "WS03")
        {
          for (unsigned int i = 0; i < view_candidate_ids.size(); ++i)
          {
            if (view_candidate_ids[i] < 24 || view_candidate_ids[i] >= 36)
            {
              new_view_candidate_ids.push_back(view_candidate_ids[i]);
            }
          }
        }
        else if (old_ws == "WS04")
        {
          for (unsigned int i = 0; i < view_candidate_ids.size(); ++i)
          {
            if (view_candidate_ids[i] < 36)
            {
              new_view_candidate_ids.push_back(view_candidate_ids[i]);
            }
          }
        }
      } */
      
      // receive data....................................................
      robot::CommunicationInterface::ReceptionInfo data_retrieval_status;
      do
      {
      	status_ = Status::DEMANDING_NEW_DATA;
      	data_retrieval_status = robot_comm_unit_->retrieveData(); 
      	
      	if( !runProcedure_ ) // exit point
      	{
      	  status_ = Status::IDLE;
      	  runProcedure_ = false;
      	  return;
      	}
      	pausePoint();
	
      }while( data_retrieval_status != robot::CommunicationInterface::ReceptionInfo::SUCCEEDED );

      std::cout<<"\nData reception nr. "<<++reception_nr<<"." <<"\n";

      //add point cloud accumulator
      //..............................................................................................................
      int numb_subscriber = 0;
      std_msgs::String cloud_msg;
      cloud_msg.data = "e_add_cloud_start";

      while(numb_subscriber == 0)
      {
        client_e_add_cloud_start.publish(cloud_msg);
        numb_subscriber = client_e_add_cloud_start.getNumSubscribers();
      }
      
      if (iteration_number % 2 == 0)
      {
        ROS_INFO("Running object detector...");
        std_msgs::Int32 obj_msg;
        obj_msg.data = 2;

        std_msgs::Int32 obj_result = youbot_comm_unit_->runObjectDetector(obj_msg);

        std::cout << "Obj detector result: " << obj_result.data << "\n";
      }
       
      // check the viewspace......................................................
      std::map<views::View::IdType, views::View > views_index_map = viewspace_->get_views_index_map_();
      for ( auto i : views_index_map) 
      {
        std::cout<<i.first<<", "<<i.second<<" \n";
      }
      
      //...................
      //set workstation constraint
      bool workstation_constraint = false;

      // getting cost and ig is wrapped in the utility calculator..................
      status_ = Status::NBV_CALCULATIONS;
      views::View::IdType nbv_id;
      if (config_.random_view == false)
      {
        //views::View::IdType nbv_id = utility_calculator_->getNbv(new_view_candidate_ids, viewspace_, workstations_map_, workstation_constraint);
        nbv_id = utility_calculator_->getNbv(view_candidate_ids, viewspace_, workstations_map_, workstation_constraint);
        ROS_INFO("NBV is calculated by cost function");
      }
      else
      {
        //std::vector<View::IdType> IdSet = view_candidate_ids;
        nbv_id = rand() % view_candidate_ids.size();
        ROS_INFO("NBV is selected randomly");
      }
/*       if (nbv_id >= 0 && nbv_id <= 12)
      {
        old_ws = "WS01";
      }
      else if (nbv_id >= 12 && nbv_id<=22 )
      {
        old_ws = "WS02";
      }
      else if (nbv_id>=24 && nbv_id<=34 )
      {
        old_ws = "WS03";
      }
      else if (nbv_id>=36 && nbv_id <=46 )
      {
        old_ws = "WS04";
      } */

      //result for next view (the next pose)
      views::View nbv = viewspace_->getView(nbv_id);

      std::cout<<"\n\n Next view: "<<nbv_id<<"\n";
      std::cout<<"Pose: "<<nbv.pose()<<"\n\n";
      

      // check termination criteria ...............................................
      if( goal_evaluation_module_->isDone() )
      {
        std::cout<<"\n\nTermination criteria was fulfilled. Reconstruction procedure ends.\n\n";
        std::cout<<"Number of moving: "<< moving_nr << "\n";
        break;
      }
      
      // move to next best view....................................................
      bool successfully_moved = false;
      do
      {
      	status_ = Status::DEMANDING_MOVE; 
      	successfully_moved = robot_comm_unit_->moveTo(nbv);

        ros::Publisher pub = nh.advertise<std_msgs::String>("text", 5);
        //ros::Rate loop_rate(10);
        std_msgs::String str;
        str.data = "ROBOT IS MOVING";
        pub.publish(str);


      	moving_nr++;
      	if( !runProcedure_ ) // exit point
      	{
      	  status_ = Status::IDLE;
      	  runProcedure_ = false;
      	  return;
      	}
      	pausePoint();
	
      }while(!successfully_moved);

      /*bool successfully_move_arm = false;
      do
      {
        ros::ServiceClient joint_client = nh.serviceClient<ig_active_reconstruction_msgs::ybMoveToJoints>("youbot/move_to_joints");

        ig_active_reconstruction_msgs::ybMoveToJoints move_join_srv;
        std::string nbv_id_string = std::to_string(nbv_id);
        move_join_srv.request.id = nbv_id_string;

        ros::Publisher pub = nh.advertise<std_msgs::String>("text", 5);
        std_msgs::String str;
        str.data = "ARM IS MOVING";
        pub.publish(str);

        if (joint_client.call(move_join_srv))
        {
          ROS_INFO("Successfully moving arm");
        }
        else
        {
          ROS_ERROR("Failed to call service youbot/move_to_joints");
        }
        successfully_move_arm = move_join_srv.response.success;
        pausePoint();

      }while(!successfully_move_arm);*/

      //std::cout<<"Number of moving: " << moving_nr << "\n";

      // update viewspace
      viewspace_->setVisited(nbv_id);
      if( config_.max_visits!=-1 && viewspace_->timesVisited(nbv_id) >= config_.max_visits )
        viewspace_->setBad(nbv_id);
      
      iteration_number ++;

    }while( runProcedure_ );
    
    status_ = Status::IDLE;
    runProcedure_ = false;
    return;
  }
  
  void BasicViewPlanner::pausePoint()
  {
    while(pauseProcedure_)
    {
      status_ = Status::PAUSED;
      boost::this_thread::sleep_for( boost::chrono::seconds(1) );
    }
  }
}
