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
   //ROS_INFO("%d heard: [%s]", int32_t(msg->id), msg->msg.c_str());
  
  CameraPosition l_camera( double(msg->sensor.max_radius), double(msg->sensor.min_radius), double(msg->sensor.heading), double(msg->sensor.fov) );
  
  AgentPosition l_position_and_camera (Real2D( double(msg->odometry.pose.pose.position.x), double(msg->odometry.pose.pose.position.y) ), l_camera);
  
  m_agent->setCurrentPosition(l_position_and_camera);
  
  ROS_DEBUG("Position %f, %f\nCamera ctrl: %f, %f, %f, %f.\n", 
	   msg->odometry.pose.pose.position.x, msg->odometry.pose.pose.position.y, 
	   msg->sensor.max_radius, msg->sensor.min_radius, msg->sensor.heading, msg->sensor.fov);
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
  l_guardname << "/publisher/state/guard/";
  l_guardname << m_agent->getID();
  
  std::stringstream l_thiefname;
  l_thiefname << "/publisher/state/thief/";
  l_thiefname << m_agent->getID();
  
  m_guardSubscriber = m_node.subscribe(l_guardname.str().c_str(), 1, &AgentInterface::GuardStateUpdater, this);
  m_thiefSubscriber = m_node.subscribe(l_thiefname.str().c_str(), 1, &AgentInterface::ThiefStateUpdater, this);
}

////////////////////////////////////////////////////////////
AgentInterface::~AgentInterface()
{}