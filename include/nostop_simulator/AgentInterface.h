////////////////////////////////////////////////////
//	AgentInterface.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AGENT_INTERFACE_H
#define AGENT_INTERFACE_H
#pragma once

#include "agent.h"

#include "ros/ros.h"

#include "nostop_agent/GuardStateData.h"
#include "nostop_agent/ThiefStateData.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class AgentInterface
		{
			ros::NodeHandle m_node;
			ros::Subscriber m_guardSubscriber;
			ros::Subscriber m_thiefSubscriber;
			
			std::shared_ptr<Agent> m_agent;
		  
		public:
			AgentInterface(std::shared_ptr<Agent> agent_);
			
			~AgentInterface();
			
			void GuardStateUpdater(const nostop_agent::GuardStateDataConstPtr& msg);
			void ThiefStateUpdater(const nostop_agent::ThiefStateDataConstPtr& msg);
		};

	}
}


#endif // COLLECT_PLAYERS_H