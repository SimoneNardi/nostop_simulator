////////////////////////////////////////////////////
//	AlgorithmUpdater.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef ALGORITHM_UPDATER_H
#define ALGORITHM_UPDATER_H
#pragma once

#include "ThreadBase.h"
#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class LearningWorld;
		class AgentCall;

		class AlgorithmUpdater : public ThreadBase
		{
			std::shared_ptr<LearningWorld> m_algorithm;
			std::shared_ptr<AgentCall> m_caller;
			
		protected:
			virtual void run();

		public:
			AlgorithmUpdater(std::shared_ptr<LearningWorld> algorithm_, std::shared_ptr<AgentCall> caller_);
			~AlgorithmUpdater();
		};
	}
}


#endif // COLLECT_PLAYERS_H