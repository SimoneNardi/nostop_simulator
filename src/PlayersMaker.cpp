#include "PlayersMaker.h"
#include "area.h"
#include "agent.h"
#include "guard.h"
#include "thief.h"

#include "ros/ros.h"

#include "AgentInterface.h"
#include "PlayerIDSender.h"

#include "nostop_agent/PlayerIDData.h"

#include "GazeboDriver.h"

#include <memory>
#include <vector>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

////////////////////////////////////////
PlayersMaker::PlayersMaker(int number_of_players, int number_of_thieves)
	: m_agents()
{
	// wait for agents from simulator

	// wait for agents from other machines

	// add thief to agents
}


////////////////////////////////////////////////////////////////
PlayersMaker::PlayersMaker(std::shared_ptr<Area> area_, int number_of_players, int number_of_thieves, std::shared_ptr<GazeboDriver> gazebo_driver_)
{
	// wait for agent connection
	std::cout << "Wait for " << number_of_players << (number_of_players > 1 ? " players," : " player,") << std::endl;
	std::cout << "and " << number_of_thieves << (number_of_thieves > 1 ? " thieves.": " thief.")<< std::endl;
	
	PlayerIDSender l_sender(number_of_players, number_of_thieves);
	l_sender.sendIDToPlayer();
	l_sender.stop();
		
	ros::Rate r(1); // 1 hz
  
	std::set<Real2D> l_already_assigned;
			
	///////////////////////////
	// add thief to agents  
	std::map<int,std::string> l_IDOfThievesMap = l_sender.getIDOfThieves();
	for (std::map<int,std::string>::iterator it = l_IDOfThievesMap.begin();  it != l_IDOfThievesMap.end() ; ++it)
	{
		Real2D l_position(0.,0.);// = area_->randomPosition();
		bool l_notequal = false;
		while(!l_notequal)
		{
		    if( l_already_assigned.find(l_position) != l_already_assigned.end())
		    {
		      r.sleep();
		      l_position = area_->randomPosition();
		    }
		    else
		    {
		      l_already_assigned.insert(l_position);
		      l_notequal = true;
		    }
		}
	  
	  
		AgentPosition l_pos(l_position, CameraPosition(area_->getDistance() / 10. ) );
		
		// Create Thief
		Real2D l_point = l_pos.getPoint2D();
		ROS_INFO("Thief Position: %ld , %ld", (long int)l_point(0), (long int)l_point(1));

		std::shared_ptr<Agent> l_agent = std::make_shared<Thief>(it->first, l_pos);
		m_agents.insert(l_agent);
		
		if (gazebo_driver_)
		  // Add Thief to Gazebo
		{
		  geometry_msgs::Pose::Ptr l_initialPoseT = boost::make_shared<geometry_msgs::Pose>();
		  Real2D l_point2 = l_pos.getPoint2D();
		  l_initialPoseT->position.x = l_point2[0];
		  l_initialPoseT->position.y = l_point2[1];
		  
		  geometry_msgs::Quaternion l_orientation;
		  l_orientation.x =0;
		  l_orientation.y =0;
		  l_orientation.z =0;
		  l_orientation.w =1;
		  l_initialPoseT->orientation = l_orientation;
		  
		  gazebo_driver_->addThief(it->second, l_initialPoseT);
		}
		
		r.sleep();
	}
	
	/////////////////////////////////////////
	// Simulated Player
	std::map<int,std::string> l_IDOfPlayersMap = l_sender.getIDOfSimPlayers();
	for (std::map<int,std::string>::iterator it = l_IDOfPlayersMap.begin();  it != l_IDOfPlayersMap.end() ; ++it)
	{
		Real2D l_position = area_->randomPosition();
		bool l_notequal = false;
		while(!l_notequal)
		{
		    if( l_already_assigned.find(l_position) != l_already_assigned.end())
		    {
		      r.sleep();
		      l_position = area_->randomPosition();
		    }
		    else
		    {
		      l_already_assigned.insert(l_position);
		      l_notequal = true;
		    }
		}
		
		AgentPosition l_pos(l_position, CameraPosition(area_->getDistance() / 10. ) );
		
		Real2D l_point = l_pos.getPoint2D();
		ROS_INFO( "Guard %s Position: %ld , %ld", it->second.c_str(), (long int)l_point(0), (long int)l_point(1) );
		
		// Create Guard
		std::shared_ptr<Agent> l_agent = std::make_shared<Guard>(1, it->first, l_pos);
		m_agents.insert(l_agent);
		
		if (gazebo_driver_)
		  // Add Guard to Gazebo
		{
		  geometry_msgs::Pose::Ptr l_initialPoseG = boost::make_shared<geometry_msgs::Pose>();
		  Real2D l_point1 = l_pos.getPoint2D();
		  l_initialPoseG->position.x = l_point1[0];
		  l_initialPoseG->position.y = l_point1[1];
		  
		  geometry_msgs::Quaternion l_orientation;
		  l_orientation.x =0;
		  l_orientation.y =0;
		  l_orientation.z =0;
		  l_orientation.w =1;
		  l_initialPoseG->orientation = l_orientation;
		  
		  gazebo_driver_->addGuard(it->second, l_initialPoseG, 2);
		}
		
		r.sleep();
	}
	
	///////////////////////////
	// Real Player
	l_IDOfPlayersMap = l_sender.getIDOfRealPlayers();
	for (std::map<int,std::string>::iterator it = l_IDOfPlayersMap.begin();  it != l_IDOfPlayersMap.end() ; ++it)
	{
		Real2D l_position = area_->randomPosition();
		bool l_notequal = false;
		while(!l_notequal)
		{
		    if( l_already_assigned.find(l_position) != l_already_assigned.end())
		    {
		      r.sleep();
		      l_position = area_->randomPosition();
		    }
		    else
		    {
		      l_already_assigned.insert(l_position);
		      l_notequal = true;
		    }
		}
		
		AgentPosition l_pos(l_position, CameraPosition(area_->getDistance() / 10. ) );
		
		Real2D l_point = l_pos.getPoint2D();
		ROS_INFO( "Guard %s Position: %ld , %ld", it->second.c_str(), (long int)l_point(0), (long int)l_point(1) );
		
		// Create Guard
		std::shared_ptr<Agent> l_agent = std::make_shared<Guard>(1, it->first, l_pos);
		m_agents.insert(l_agent);
		
		if (gazebo_driver_)
		  // Add Guard to Gazebo
		{
		  geometry_msgs::Pose::Ptr l_initialPoseG = boost::make_shared<geometry_msgs::Pose>();
		  Real2D l_point1 = l_pos.getPoint2D();
		  l_initialPoseG->position.x = l_point1[0];
		  l_initialPoseG->position.y = l_point1[1];
		  
		  geometry_msgs::Quaternion l_orientation;
		  l_orientation.x =0;
		  l_orientation.y =0;
		  l_orientation.z =0;
		  l_orientation.w =1;
		  l_initialPoseG->orientation = l_orientation;
		  
		  gazebo_driver_->addGuard(it->second, l_initialPoseG, 0);
		}
		
		r.sleep();
	}
}

////////////////////////////////////////////////////////////////
std::set<std::shared_ptr<Agent> > PlayersMaker::getGuards() const
{
	return m_agents;
}

////////////////////////////////////////////////////////////////
std::set<std::shared_ptr<Agent> > PlayersMaker::getThieves() const
{
	return m_agents; 
}

////////////////////////////////////////////////////////////////
std::set< std::shared_ptr<AgentInterface> > PlayersMaker::getPlayersInterface() const
{
  std::set< std::shared_ptr<AgentInterface> > l_result;
  for(auto it = m_agents.begin(); it != m_agents.end(); ++it)
  {
      l_result.insert(std::make_shared<AgentInterface>(*it));
  }
  return l_result;
}