#include "PlayersMaker.h"
#include "area.h"
#include "agent.h"
#include "guard.h"
#include "thief.h"

#include "ros/ros.h"

#include "AgentInterface.h"
#include "PlayerIDSender.h"

#include "nostop_agent/PlayerIDData.h"

#include <memory>
#include <vector>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

////////////////////////////////////////
PlayersMaker::PlayersMaker(int number_of_players, int number_of_thieves)
	: m_agents()
{
	// wait for agents from simulator

	// wait for agents from other machines

	// add thief to agents
}


////////////////////////////////////////////////////////////////
PlayersMaker::PlayersMaker(std::shared_ptr<Area> area_, int number_of_players, int number_of_thieves)
{
	// wait for agent connection
	std::cout << "Wait for " << number_of_players << (number_of_players > 1 ? " players," : " player,") << std::endl;
	std::cout << "and " << number_of_thieves << (number_of_thieves > 1 ? " thieves.": " thief.")<< std::endl;

	PlayerIDSender l_sender(number_of_players, number_of_thieves);
	l_sender.sendIDToPlayer();
	l_sender.stop();
	
	/////////////////////////////////////////
	// Create Agent:
	std::map<int,std::string> l_IDOfPlayersMap = l_sender.getIDofPlayers();
	for (std::map<int,std::string>::iterator it = l_IDOfPlayersMap.begin();  it != l_IDOfPlayersMap.end() ; ++it)
	{
		AgentPosition l_pos(area_->randomPosition(), CameraPosition(area_->getDistance() / 10. ) );
		std::shared_ptr<Agent> l_agent = std::make_shared<Guard>(1, it->first, l_pos);
		m_agents.insert(l_agent);
	}

	// add thief to agents  
	std::map<int,std::string> l_IDOfThievesMap = l_sender.getIDofThieves();
	for (std::map<int,std::string>::iterator it = l_IDOfThievesMap.begin();  it != l_IDOfThievesMap.end() ; ++it)
	{
		AgentPosition l_pos(Real2D(5,15)/*area_->randomPosition()*/, CameraPosition(area_->getDistance() / 10. ) );
		Real2D l_point = l_pos.getPoint2D();
		ROS_INFO("Thief Position: %ld , %ld", (long int)l_point(0), (long int)l_point(1));

		std::shared_ptr<Agent> l_agent = std::make_shared<Thief>(it->first, l_pos);
		m_agents.insert(l_agent);
	}
}

////////////////////////////////////////////////////////////////
std::set<std::shared_ptr<Agent> > PlayersMaker::getGuards() const
{
	return m_agents;
}

////////////////////////////////////////////////////////////////
std::set<std::shared_ptr<Agent> > PlayersMaker::getThieves() const
{
	return m_agents; 
}

////////////////////////////////////////////////////////////////
std::set< std::shared_ptr<AgentInterface> > PlayersMaker::getPlayersInterface() const
{
  std::set< std::shared_ptr<AgentInterface> > l_result;
  for(auto it = m_agents.begin(); it != m_agents.end(); ++it)
  {
      l_result.insert(std::make_shared<AgentInterface>(*it));
  }
  return l_result;
}