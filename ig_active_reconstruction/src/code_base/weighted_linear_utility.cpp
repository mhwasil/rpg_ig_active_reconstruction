/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_active_reconstruction, a ROS package for...well,

ig_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ig_active_reconstruction/weighted_linear_utility.hpp"

namespace ig_active_reconstruction
{
  
  WeightedLinearUtility::WeightedLinearUtility( double cost_weight )
  : world_comm_unit_(nullptr)
  , robot_comm_unit_(nullptr)
  , cost_weight_(cost_weight)
  {
    
  }
  
  void WeightedLinearUtility::useInformationGain( std::string name, double weight)
  {
    information_gains_.push_back(name);
    ig_weights_.push_back(weight);
  }
  
  void WeightedLinearUtility::setCostWeight( double weight )
  {
    cost_weight_ = weight;
  }
  
  void WeightedLinearUtility::setIgRetrievalConfig( world_representation::CommunicationInterface::IgRetrievalConfig& config )
  {
    ig_retrieval_config_ = config;
  }
  
  void WeightedLinearUtility::setWorldCommUnit( std::shared_ptr<world_representation::CommunicationInterface> world_comm_unit )
  {
    world_comm_unit_ = world_comm_unit;
  }
  
  void WeightedLinearUtility::setRobotCommUnit( std::shared_ptr<robot::CommunicationInterface> robot_comm_unit )
  {
    robot_comm_unit_ = robot_comm_unit;
  }
  
  views::View::IdType WeightedLinearUtility::getNbv( views::ViewSpace::IdSet& id_set, std::shared_ptr<views::ViewSpace> viewspace )
  {
    // structure to store received values
    std::vector<double> cost_vector;
    std::vector<double> ig_vector;
    
    double total_cost=0;
    double total_ig=0;
    
    world_representation::CommunicationInterface::IgRetrievalCommand command;
    command.config = ig_retrieval_config_;
    command.metric_names = information_gains_;
    
    // receive costs and igs
    for( views::View::IdType& view_id: id_set )
    {
      views::View view = viewspace->getView(view_id);
      
      robot::MovementCost cost;
      world_representation::CommunicationInterface::ViewIgResult information_gains;
      
      double cost_val = 0;
      double ig_val = 0;
      
      if( robot_comm_unit_!=nullptr )
      {
	cost = robot_comm_unit_->movementCost(view);
	
	if( cost.exception != robot::MovementCost::Exception::NONE )
	  continue; // invalid view... disregard in calculation
	else
	  cost_val = cost.cost;
      }
      
      if( world_comm_unit_!=nullptr )
      {
	command.path.clear();
	command.path.push_back( view.pose() );
	
	world_comm_unit_->computeViewIg(command,information_gains);
	for( unsigned int i= 0; i<information_gains.size(); ++i )
	{
	  if( information_gains[i].status == world_representation::CommunicationInterface::ResultInformation::SUCCEEDED )
	  {
	    ig_val += ig_weights_[i]*information_gains[i].predicted_gain;
	  }
	}
      }
      
      total_cost += cost_val;
      cost_vector.push_back(cost_val);
      total_ig += ig_val;
      ig_vector.push_back(ig_val);
    }
    
    // calculate utility and choose nbv
    views::View::IdType nbv;
    double best_util = std::numeric_limits<double>::min();
    
    double cost_factor;
    
    if( total_ig==0 )
      total_ig=1;
    
    if( total_cost==0 )
      cost_factor=0;
    else
      cost_factor = cost_weight_/total_cost;
    
    for( unsigned int i=0; i<id_set.size(); ++i )
    {
      double utility = ig_vector[i]/total_ig - cost_factor*cost_vector[i];
      if( utility>best_util )
      {
	best_util = utility;
	nbv = id_set[i];
      }
    }
    return nbv;
  }
    
  
}