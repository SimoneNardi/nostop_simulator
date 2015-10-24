////////////////////////////////////////////////////
//	ThiefAdder.h
//	Created on:	23-oct-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef THIEF_ADDER_H
#define THIEF_ADDER_H
#pragma once

#include "ThreadBase.h"
#include <memory>

class GazeboDriver;

namespace Robotics 
{
	namespace GameTheory
	{
		class Agent;
		class Area;
		class AgentInterface;
		class LearningWorld;

		class ThiefAdder : public ThreadBase
		{
			std::shared_ptr<GazeboDriver> m_gazebo_driver;
	
			std::shared_ptr<Robotics::GameTheory::LearningWorld> m_coverage;
		  		  
		public:
			ThiefAdder(std::shared_ptr<Robotics::GameTheory::LearningWorld> coverage_, std::shared_ptr<GazeboDriver> gazebo_driver_);
			
			~ThiefAdder();
			
		public:
			virtual void run();
		};

	}
}


#endif // COLLECT_PLAYERS_H