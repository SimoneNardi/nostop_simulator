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
, m_id (-1)
, m_ready(false)
{}

////////////////////////////////////////////////////////////////
void PlayerIDSender::run()
{
  ros::Rate loop_rate(10);
  
  while(ros::ok() && !m_ready)
  {
    if ( !(m_num_active_players < m_number_of_players ||  m_num_active_thieves < m_number_of_thieves ))
    {
	// this cycle has to be murdered
	Lock1 lck(m_mtx);
	m_ready = true;
	m_cv.notify_all();
	break;
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
  res.id = ++m_id;
  ++m_num_active_thieves;
  return true;
}

////////////////////////////////////////////////////////////////
bool PlayerIDSender::getValidGuardID(
	nostop_agent::PlayerIDData::Request  &req,
	nostop_agent::PlayerIDData::Response &res)
{
  res.id = ++m_id;
  ++m_num_active_players;
  return true;
}

////////////////////////////////////////////////////////////////
void PlayerIDSender::sendIDToPlayer()
{
  m_serviceGuardID = m_node.advertiseService("GuardID", &PlayerIDSender::getValidGuardID, this);
  m_serviceThiefID = m_node.advertiseService("ThiefID", &PlayerIDSender::getValidThiefID, this);
  
  ros::AsyncSpinner l_spin(0);
  if( l_spin.canStart() )
    l_spin.start();
    
  this->start();
  
  this->wait(2);
  
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