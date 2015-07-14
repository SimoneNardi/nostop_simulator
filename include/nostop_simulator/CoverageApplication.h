////////////////////////////////////////////////////
//	CoverageApplication.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef COVERAGE_APPLICATION_H
#define COVERAGE_APPLICATION_H
#pragma once

#include <memory>
#include "Threads.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class LearningWorld;
		class AlgorithmUpdater;
		class AgentUpdater;
		class VisualUpdater;
		class WorldUpdater;
		class AgentCall;

		class CoverageApplication
		{
			Mutex m_mutex;


			std::shared_ptr<LearningWorld> m_algorithm;

			std::shared_ptr<AgentUpdater> m_agentUpdater;
			std::shared_ptr<AlgorithmUpdater> m_algorithmUpdater;
			std::shared_ptr<VisualUpdater> m_visualUpdater;
			std::shared_ptr<WorldUpdater> m_worldUpdater;
			
			std::shared_ptr<AgentCall> m_caller;

		public:
			CoverageApplication(std::shared_ptr<LearningWorld> algorithm_);
			~CoverageApplication();

			bool start();
		};

	}
}


#endif // COLLECT_PLAYERS_H