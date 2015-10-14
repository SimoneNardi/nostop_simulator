////////////////////////////////////////////////////
//	AgentCall.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AGENT_CALL_H
#define AGENT_CALL_H
#pragma once

#include "guard.h"

#include <memory>
#include <map>
#include <set>

#include "Threads.h"

#include "ros/ros.h"
#include "nostop_agent/PlayerNotifyStatus.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class AgentCall
		{
		  	std::map<int, Agent::Status> m_call;
			
			mutable Mutex1 m_lockCall;
			Condition1 m_signalCall;
			
			bool m_notified;
			
			ros::NodeHandle m_node;
			ros::Publisher m_pub;
			ros::ServiceServer m_statusServer;
			
		public:
			AgentCall(std::set< std::shared_ptr<Guard> >& agent_);

			bool readyToGo() const;

			void update(std::set< std::shared_ptr<Guard> >& agent_);

			void update(std::shared_ptr<Guard> agent_);
						
			void wait();
			
			void notify();
			
			void reset();
			
			bool updateStatus_callback(
			  nostop_agent::PlayerNotifyStatus::Request  &req,
			  nostop_agent::PlayerNotifyStatus::Response &res);
		};
	}
}

#endif