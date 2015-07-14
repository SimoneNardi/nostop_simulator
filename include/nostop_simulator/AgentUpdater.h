////////////////////////////////////////////////////
//	AgentUpdater .h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AGENT_UPDATER_H
#define AGENT_UPDATER_H
#pragma once

#include "ThreadBase.h"
#include <memory>
#include <set>

namespace Robotics 
{
	namespace GameTheory
	{
		class LearningWorld;
		class AgentCall;
		class AgentInterface;
		class Guard;

		class AgentUpdater : public ThreadBase
		{
			std::shared_ptr<AgentCall> m_caller;
			std::set< std::shared_ptr<AgentInterface> > m_agentInterface;
			std::set< std::shared_ptr<Guard> > m_guards;

		protected:
			virtual void run();

		public:
			AgentUpdater(std::shared_ptr<LearningWorld> algorithm_, std::shared_ptr<AgentCall> caller_);
			~AgentUpdater();
		};
	}
}


#endif // COLLECT_PLAYERS_H