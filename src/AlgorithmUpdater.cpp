#include "AlgorithmUpdater.h"
#include "LearningWorld.h"
#include "AgentCall.h"

#include "ros/ros.h"
using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

///////////////////////////////////////////////////////////
void AlgorithmUpdater::run()
{
	ros::Duration l_duration(5.0);

	ROS_INFO("Application is running on AlgorithmUpdater thread.");

	int count = 0;
	while (ros::ok())
	{
		m_caller->wait();
		
		if( !m_algorithm->evaluateScreenshot() )
			break;

		m_caller->reset();
		
		ros::spinOnce();

		l_duration.sleep();
		++count;
		
		ROS_DEBUG("AlgorithmUpdater Run.");
	}
}

////////////////////////////////////////////////////////////
AlgorithmUpdater::AlgorithmUpdater(std::shared_ptr<LearningWorld> algorithm_, std::shared_ptr<AgentCall> caller_)
	: ThreadBase()
	, m_algorithm(algorithm_)
	, m_caller(caller_)
{}

////////////////////////////////////////////////////////////
AlgorithmUpdater::~AlgorithmUpdater()
{}