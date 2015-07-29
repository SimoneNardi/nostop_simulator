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

#include "ros/ros.h"

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
			
			ros::NodeHandle m_node;
			ros::Publisher m_neighboursPub;
			ros::Publisher m_monitorPub;
			ros::Publisher m_energyPub;
			
		protected:
			virtual void run();
			void publish(std::vector<int8_t> & neighboursData_);
		public:
			AlgorithmUpdater(std::shared_ptr<LearningWorld> algorithm_, std::shared_ptr<AgentCall> caller_);
			~AlgorithmUpdater();
		};
	}
}


#endif // COLLECT_PLAYERS_H