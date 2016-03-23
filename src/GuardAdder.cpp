#include "GuardAdder.h"

#include "learningWorld.h"

#include "GazeboDriver.h"

#include "ros/ros.h"

#include "PlayerIDSender.h"

#include "Real2D.h"
#include "guard.h"
#include "thief.h"
#include "world.h"
#include "discretizedArea.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////////////
GuardAdder::GuardAdder(std::shared_ptr<Robotics::GameTheory::LearningWorld> coverage_, std::shared_ptr<GazeboDriver> gazebo_driver_)
: ThreadBase()
, m_coverage(coverage_)
, m_gazebo_driver(gazebo_driver_)
{}

//////////////////////////////////////////////
GuardAdder::~GuardAdder()
{}

//////////////////////////////////////////////
void GuardAdder::run()
{
  ros::Rate loop_rate(1);
  
  ros::Rate r(1); // 5 hzv
  
  m_mutex.lock();
  while(ros::ok() && m_active)
    /// wait new agents:
  {
    m_mutex.unlock();
    
    PlayerIDSender l_sender(1, 0);
    l_sender.sendIDToPlayer(m_coverage->getNumberOfAgents());
    l_sender.stop();
	
    std::set<Real2D> l_already_assigned;
    m_coverage->getPlayersPosition(l_already_assigned);
    m_coverage->getThievesPosition(l_already_assigned);
	
    std::shared_ptr<DiscretizedArea> l_area = m_coverage->getWorld()->getSpace();
    
    /////////////////////////////////////////
    // Create Agent:
    std::map<int,std::string> l_IDOfPlayersMap = l_sender.getIDOfSimPlayers();
    for (std::map<int,std::string>::iterator it = l_IDOfPlayersMap.begin();  it != l_IDOfPlayersMap.end() ; ++it)
    {
	  r.sleep();
	  Real2D l_position;
	  l_area->getRandomPosition(l_position);
	  bool l_notequal = false;
	  while(!l_notequal)
	  {
	      if( l_already_assigned.find(l_position) != l_already_assigned.end())
	      {
		r.sleep();
		l_area->getRandomPosition(l_position);
	      }
	      else
	      {
		l_already_assigned.insert(l_position);
		l_notequal = true;
	      }
	  }
	    
	  AgentPosition l_pos( l_position, CameraPosition( l_area->getDistance() / 10. ) );
	  
	  Real2D l_point = l_pos.getPoint2D();
	  ROS_INFO( "Guard %s Position: %ld, %ld", it->second.c_str(), (long int)l_point(0), (long int)l_point(1) );
	  
	  // Create Guard
	  std::shared_ptr<Agent> l_agent = std::make_shared<Guard>(1, it->first, l_pos);
	  
	  m_coverage->addGuard(l_agent);
	  
	  if (m_gazebo_driver)
	    // Add Guard to Gazebo
	  {
	    geometry_msgs::Pose::Ptr l_initialPoseG = boost::make_shared<geometry_msgs::Pose>();
	    Real2D l_point1 = l_pos.getPoint2D();
	    l_initialPoseG->position.x = l_point1[0];
	    l_initialPoseG->position.y = l_point1[1];
	    m_gazebo_driver->addGuard(it->second, l_initialPoseG, 2);
	  }
    }
    ///
    
    l_IDOfPlayersMap = l_sender.getIDOfRealPlayers();
    for (std::map<int,std::string>::iterator it = l_IDOfPlayersMap.begin();  it != l_IDOfPlayersMap.end() ; ++it)
    {
	  r.sleep();
	  Real2D l_position;
	  l_area->getRandomPosition(l_position);
	  bool l_notequal = false;
	  while(!l_notequal)
	  {
	      if( l_already_assigned.find(l_position) != l_already_assigned.end())
	      {
		r.sleep();
		l_area->getRandomPosition(l_position);
	      }
	      else
	      {
		l_already_assigned.insert(l_position);
		l_notequal = true;
	      }
	  }
	    
	  AgentPosition l_pos( l_position, CameraPosition( l_area->getDistance() / 10. ) );
	  
	  Real2D l_point = l_pos.getPoint2D();
	  ROS_INFO( "Guard %s Position: %ld, %ld", it->second.c_str(), (long int)l_point(0), (long int)l_point(1) );
	  
	  // Create Guard
	  std::shared_ptr<Agent> l_agent = std::make_shared<Guard>(1, it->first, l_pos);
	  
	  m_coverage->addGuard(l_agent);
	  
	  if (m_gazebo_driver)
	    // Add Guard to Gazebo
	  {
	    geometry_msgs::Pose::Ptr l_initialPoseG = boost::make_shared<geometry_msgs::Pose>();
	    Real2D l_point1 = l_pos.getPoint2D();
	    l_initialPoseG->position.x = l_point1[0];
	    l_initialPoseG->position.y = l_point1[1];
	    m_gazebo_driver->addGuard(it->second, l_initialPoseG, 0);
	  }
    }
    
    
    ros::spinOnce();

    loop_rate.sleep();
    
    m_mutex.lock();
  }
  m_mutex.unlock();
}