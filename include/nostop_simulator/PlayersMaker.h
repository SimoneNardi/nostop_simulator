////////////////////////////////////////////////////
//	PlayersMaker.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef PLAYERS_MAKER_H
#define PLAYERS_MAKER_H
#pragma once

#include "agent.h"
#include <set>
#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{

		class Agent;
		class Area;
		class AgentInterface;

		class PlayersMaker
		{
			std::set<std::shared_ptr<Agent> > m_agents;
		public:
			PlayersMaker(int number_of_players = 3, int number_of_thieves = 1);
			PlayersMaker(std::shared_ptr<Area> area_, int number_of_players = 3, int number_of_thieves = 1);

		public:      
			std::set<std::shared_ptr<Agent> > getPlayers() const {return m_agents;}
			std::set<std::shared_ptr<Agent> > getGuards() const; 
			std::set<std::shared_ptr<Agent> > getThieves() const;
			
			std::set< std::shared_ptr<AgentInterface> > getPlayersInterface() const;
		};

	}
}


#endif // COLLECT_PLAYERS_H