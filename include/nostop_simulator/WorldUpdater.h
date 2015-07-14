////////////////////////////////////////////////////
//	WorldUpdater .h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef WORLD_UPDATER_H
#define WORLD_UPDATER_H
#pragma once

#include "ThreadBase.h"
#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class LearningWorld;

		class WorldUpdater : public ThreadBase
		{
			std::shared_ptr<LearningWorld> m_algorithm;

		protected:
			virtual void run();

		public:
			WorldUpdater(std::shared_ptr<LearningWorld> algorithm_);
			~WorldUpdater();
		};
	}
}


#endif // VISUAL_UPDATER_H