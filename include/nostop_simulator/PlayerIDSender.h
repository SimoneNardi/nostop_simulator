////////////////////////////////////////////////////
//	PlayerIDSender.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef PLAYER_ID_SENDER_H
#define PLAYER_ID_SENDER_H
#pragma once

#include "ThreadBase.h"

#include "ros/ros.h"

#include "nostop_agent/PlayerIDData.h"

#include "Threads.h"

namespace Robotics 
{
	namespace GameTheory
	{
		class Agent;
		class Area;
		class AgentInterface;

		class PlayerIDSender : public ThreadBase
		{
		  mutable Mutex1 m_mtx;
		  Condition1 m_cv;
		  bool m_ready;
		  
		  ros::NodeHandle m_node;
		  ros::ServiceServer m_serviceGuardID;
		  ros::ServiceServer m_serviceThiefID;
		  
		  int m_number_of_players;
		  int m_number_of_thieves;
		  
		  int m_num_active_players;
		  int m_num_active_thieves;
		  
		  int m_id;
		  
		  std::map<int,std::string> m_IDPlayer;
		  std::map<int,std::string> m_IDThief;
		  
		public:
			PlayerIDSender(int number_of_players = 3, int number_of_thieves = 1);
			
		public:
			void sendIDToPlayer();
		
			bool getValidGuardID(
			  nostop_agent::PlayerIDData::Request  &req,
			  nostop_agent::PlayerIDData::Response &res);
			
			bool getValidThiefID(
			  nostop_agent::PlayerIDData::Request  &req,
			  nostop_agent::PlayerIDData::Response &res);
			  
			virtual void run();
			
			void wait(int minute);
			
			int getNumberOfPlayer() {return m_num_active_players;}
			int getNumberOfThieves() {return m_num_active_thieves;}
			
			std::map<int,std::string> getIDofPlayers() const {return m_IDPlayer;}
			std::map<int,std::string> getIDofThieves() const {return m_IDThief;}
		};

	}
}


#endif // COLLECT_PLAYERS_H