#include "PlayerIDSender.h"

#include <chrono>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

////////////////////////////////////////////////////////////////
PlayerIDSender::PlayerIDSender(int num_of_players, int num_of_thieves)
: ThreadBase()
, m_number_of_players(num_of_players)
, m_number_of_thieves(num_of_thieves)
, m_num_active_players(0)
, m_num_active_thieves(0)
, m_num_active_agents (0)
, m_ready(false)
, m_IDPlayer()
, m_IDThief()
{}

////////////////////////////////////////////////////////////////
PlayerIDSender::~PlayerIDSender()
{
  m_serviceGuardID.shutdown();
  m_serviceThiefID.shutdown();
}

////////////////////////////////////////////////////////////////
void PlayerIDSender::run()
{
  ros::Rate loop_rate(10);
  
  while(ros::ok() && !m_ready)
  {
    if ( !(m_num_active_players < m_number_of_players ||  m_num_active_thieves < m_number_of_thieves ) || m_ready)
    {
	// this cycle has to be murdered
	Lock1 lck(m_mtx);
	m_ready = true;
	m_cv.notify_all();
	return;
    }
    
    ros::spinOnce();

    loop_rate.sleep();
    
  }
}

////////////////////////////////////////////////////////////////
bool PlayerIDSender::getValidThiefID(
	nostop_agent::PlayerIDData::Request  &req,
	nostop_agent::PlayerIDData::Response &res)
{
  res.id = m_num_active_agents++;
  ++m_num_active_thieves;
  
  m_IDThief.insert( std::make_pair (res.id, req.name) );
  
  return true;
}

////////////////////////////////////////////////////////////////
bool PlayerIDSender::getValidGuardID(
	nostop_agent::PlayerIDData::Request  &req,
	nostop_agent::PlayerIDData::Response &res)
{
  res.id = m_num_active_agents++;
  ++m_num_active_players;
  
  m_IDPlayer.insert( std::make_pair (res.id, req.name) );
    
  return true;
}

////////////////////////////////////////////////////////////////
void PlayerIDSender::sendIDToPlayer(int num_active_agents)
{
  m_num_active_agents = num_active_agents;
  
  m_serviceGuardID = m_node.advertiseService("/simulator/guard/id", &PlayerIDSender::getValidGuardID, this);
  m_serviceThiefID = m_node.advertiseService("/simulator/thief/id", &PlayerIDSender::getValidThiefID, this);
  
  ros::AsyncSpinner l_spin(0);
  if( l_spin.canStart() )
    l_spin.start();
    
  this->start();
  
  this->wait(5);
  
  l_spin.stop();
}

////////////////////////////////////////////////////////////////
void PlayerIDSender::wait(int minute_)
{
  Lock1 lck(m_mtx);
  while (!m_ready)  // for 
  {
    std::cv_status l_st = m_cv.wait_for( lck,  std::chrono::minutes(minute_) );
  
    if (l_st == std::cv_status::timeout)
      m_ready = true;
  }
}