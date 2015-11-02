////////////////////////////////////////////////////
//	PlayerAdder.h
//	Created on:	23-oct-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef PLAYER_ADDER_H
#define PLAYER_ADDER_H
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
		class GuardAdder;
		class ThiefAdder;

		class PlayerAdder
		{
		  std::shared_ptr<GuardAdder> m_guard_adder;
		  std::shared_ptr<ThiefAdder> m_thief_adder;
		  		  
		public:
			PlayerAdder(std::shared_ptr<Robotics::GameTheory::LearningWorld> coverage_, std::shared_ptr<GazeboDriver> gazebo_driver_);
			~PlayerAdder();
			
			void start();
			void stop();
		};

	}
}


#endif // COLLECT_PLAYERS_H