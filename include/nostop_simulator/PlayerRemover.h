////////////////////////////////////////////////////
//	PlayerRemover.h
//	Created on:	23-oct-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef PLAYER_REMOVER_H
#define PLAYER_REMOVER_H
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

		class PlayerRemover : public ThreadBase
		{
		  bool m_active;
		  
		  std::shared_ptr<GazeboDriver> m_gazebo_driver;
		  
		  std::shared_ptr<Robotics::GameTheory::LearningWorld> m_coverage;
		  
		public:
			PlayerRemover(std::shared_ptr<Robotics::GameTheory::LearningWorld> coverage_, std::shared_ptr<GazeboDriver> gazebo_driver_);
			
		public:
			virtual void run();
		};

	}
}


#endif // COLLECT_PLAYERS_H