#include "ThiefAdder.h"

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
ThiefAdder::ThiefAdder(std::shared_ptr<Robotics::GameTheory::LearningWorld> coverage_, std::shared_ptr<GazeboDriver> gazebo_driver_)
: ThreadBase()
, m_coverage(coverage_)
, m_gazebo_driver(gazebo_driver_)
{}

//////////////////////////////////////////////
ThiefAdder::~ThiefAdder()
{}

//////////////////////////////////////////////
void ThiefAdder::run()
{
  ros::Rate loop_rate(1);
  
  ros::Rate r(1); // 5 hzv
  
  m_mutex.lock();
  while(ros::ok() && m_active)
    /// wait new agents:
  {
    m_mutex.unlock();
    
    PlayerIDSender l_sender(0, 1);
    l_sender.sendIDToPlayer(m_coverage->getNumberOfAgents());
    l_sender.stop();
	
    std::set<Real2D> l_already_assigned;
    m_coverage->getPlayersPosition(l_already_assigned);
    m_coverage->getThievesPosition(l_already_assigned);
	
    std::shared_ptr<DiscretizedArea> l_area = m_coverage->getWorld()->getSpace();
    
    ///////////////////////////////////////
    // add thief to agents  
    std::map<int,std::string> l_IDOfThievesMap = l_sender.getIDOfThieves();
    for (std::map<int,std::string>::iterator it = l_IDOfThievesMap.begin();  it != l_IDOfThievesMap.end() ; ++it)
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
      
	    AgentPosition l_pos(l_position, CameraPosition( l_area->getDistance() / 10. ) );
	    
	    // Create Thief
	    Real2D l_point = l_pos.getPoint2D();
	    ROS_INFO("Thief Position: %ld , %ld", (long int)l_point(0), (long int)l_point(1));

	    std::shared_ptr<Agent> l_agent = std::make_shared<Thief>(it->first, l_pos);
	    
	    m_coverage->addThief(l_agent);
	    
	    if (m_gazebo_driver)
	      // Add Thief to Gazebo
	    {
	      geometry_msgs::Pose::Ptr l_initialPoseT = boost::make_shared<geometry_msgs::Pose>();
	      Real2D l_point2 = l_pos.getPoint2D();
	      l_initialPoseT->position.x = l_point2[0];
	      l_initialPoseT->position.y = l_point2[1];
	      m_gazebo_driver->addThief(it->second, l_initialPoseT);
	    }
    }
    ///
    
    ros::spinOnce();

    loop_rate.sleep();
    
    m_mutex.lock();
  }
  m_mutex.unlock();
}