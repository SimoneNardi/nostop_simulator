////////////////////////////////////////////////////
//	VisualUpdater .h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef VISUAL_UPDATER_H
#define VISUAL_UPDATER_H
#pragma once

#include "ThreadBase.h"
#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{
		class LearningWorld;

		class VisualUpdater : public ThreadBase
		{
			std::shared_ptr<LearningWorld> m_algorithm;

		protected:
			virtual void run();

		public:
			VisualUpdater(std::shared_ptr<LearningWorld> algorithm_);
			~VisualUpdater();
		};
	}
}


#endif // VISUAL_UPDATER_H