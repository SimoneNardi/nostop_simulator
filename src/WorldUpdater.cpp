#include "WorldUpdater.h"
#include "learningWorld.h"

#include "ros/ros.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

///////////////////////////////////////////////////////////
void WorldUpdater::run()
{
	ros::Rate loop_rate(1);

	ROS_INFO("Application is running on WorldUpdater thread.");

	int count = 0;
	while (ros::ok())
	{
		if( !m_algorithm->worldUpdate() )
			break;

		ros::spinOnce();

		loop_rate.sleep();
		++count;

		ROS_DEBUG("WorldUpdater Run.");
	}
}

////////////////////////////////////////////////////////////
WorldUpdater::WorldUpdater(std::shared_ptr<LearningWorld> algorithm_)
	: ThreadBase()
	, m_algorithm(algorithm_)
{}

////////////////////////////////////////////////////////////
WorldUpdater::~WorldUpdater()
{}