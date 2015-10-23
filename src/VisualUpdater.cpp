#include "VisualUpdater.h"
#include "learningWorld.h"

#include "ros/ros.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

///////////////////////////////////////////////////////////
void VisualUpdater::run()
{
	ros::Rate loop_rate(10);

	ROS_INFO("Application is running on VisualUpdater thread.");

	int count = 0;
	while (ros::ok())
	{
		if( !m_algorithm->visualUpdate() )
			break;

		ros::spinOnce();

		loop_rate.sleep();
		++count;

		ROS_INFO("VisualUpdater Run.");
	}
}

////////////////////////////////////////////////////////////
VisualUpdater::VisualUpdater(std::shared_ptr<LearningWorld> algorithm_)
	: ThreadBase()
	, m_algorithm(algorithm_)
{}

////////////////////////////////////////////////////////////
VisualUpdater::~VisualUpdater()
{}