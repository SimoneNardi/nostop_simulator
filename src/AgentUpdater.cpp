#include "AgentUpdater.h"
#include "LearningWorld.h"
#include "AgentCall.h"
#include "AgentInterface.h"

#include "guard.h"
#include "world.h"

#include "ros/ros.h"
#include "ros/time.h"

#include <mutex>
#include <condition_variable>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

///////////////////////////////////////////////////////////
void AgentUpdater::run()
{
	ros::Duration l_duration(2.0);

	ROS_INFO("Application is running on AgentUpdater thread.");

	int count = 0;
	while (ros::ok())
	{
		ros::spinOnce();

		l_duration.sleep();
		
		++count;
		
		ROS_DEBUG("AgentUpdater Run.");
		
		if ( m_caller->readyToGo() ) // used to avoid spurious wakeups 
		{
		    m_caller->notify();
		    ROS_DEBUG("Agent ready to move.");
		}
		else
		{
		    m_caller->update( m_guards );
		}
	}
}

////////////////////////////////////////////////////////////
AgentUpdater::AgentUpdater(std::shared_ptr<LearningWorld> algorithm_, std::shared_ptr<AgentCall> caller_)
	: ThreadBase()
	, m_caller(caller_)
{
	m_guards = algorithm_->getWorld()->getGuards();
  
	auto l_agents = algorithm_->getWorld()->getAgents();
	
	std::set< std::shared_ptr<AgentInterface> > l_result;
	for(auto it = l_agents.begin(); it != l_agents.end(); ++it)
	{
	    m_agentInterface.insert( std::make_shared<AgentInterface>(*it) );
	}
}

////////////////////////////////////////////////////////////
AgentUpdater::~AgentUpdater()
{}