#include "AgentInterface.h"
#include "agent.h"

#include "ros/ros.h"
#include <sstream>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

////////////////////////////////////////////////////////////
void AgentInterface::GuardStateUpdater(const nostop_agent::GuardStateConstPtr& msg)
{ 
//   ROS_DEBUG("%d heard: [%s]", int32_t(msg->id), msg->msg.c_str());
  
  CameraPosition l_camera( double(msg->sensor.max_radius), double(msg->sensor.min_radius), double(msg->sensor.heading), double(msg->sensor.fov) );
  
  AgentPosition l_position (Real2D( double(msg->odometry.pose.pose.position.x), double(msg->odometry.pose.pose.position.y) ), l_camera);
  
  m_agent->setCurrentPosition(l_position);
  
  Agent::Status l_stat = Agent::ACTIVE;
  switch( int32_t(1) )//msg->status) ) // CONTROLLARE
  {
    case 0:
      l_stat = Agent::ACTIVE;
      break;
    case 1:
    default:
      l_stat = Agent::DISABLE;
      break;
    case 2:
      l_stat = Agent::STANDBY;
      break;
    case 3:
      l_stat = Agent::WAKEUP;
      break;
  }
  
  m_agent->setStatus( l_stat );
}

////////////////////////////////////////////////////////////
void AgentInterface::ThiefStateUpdater(const nostop_agent::ThiefStateConstPtr& msg)
{ 
//   ROS_DEBUG("%d heard: [%s]", int32_t(msg->id), msg->msg.c_str() );
    
  AgentPosition l_position (Real2D( double(msg->odometry.pose.pose.position.x), double(msg->odometry.pose.pose.position.y) ) );
  
  m_agent->setCurrentPosition(l_position);
}

////////////////////////////////////////////////////////////
AgentInterface::AgentInterface(std::shared_ptr<Agent> agent_)
: m_agent(agent_)
{
  std::stringstream l_guardname;
  l_guardname << "GuardState_";
  l_guardname << m_agent->getID();
  
  std::stringstream l_thiefname;
  l_thiefname << "Thief_State_";
  l_thiefname << m_agent->getID();
  
  m_guardSubscriber = m_node.subscribe(l_guardname.str().c_str(), 10, &AgentInterface::GuardStateUpdater, this);
  m_thiefSubscriber = m_node.subscribe(l_thiefname.str().c_str(), 10, &AgentInterface::ThiefStateUpdater, this);
}

////////////////////////////////////////////////////////////
AgentInterface::~AgentInterface()
{}