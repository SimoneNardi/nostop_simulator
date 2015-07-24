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
			
		public:
			AgentCall(std::set< std::shared_ptr<Guard> >& agent_);

			bool readyToGo() const;

			void update(std::set< std::shared_ptr<Guard> >& agent_);

			void update(std::shared_ptr<Guard> agent_);
						
			void wait();
			
			void notify();
			
			void reset();
		};
	}
}

#endif