#include "AlgorithmUpdater.h"
#include "learningWorld.h"
#include "AgentCall.h"

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

///////////////////////////////////////////////////////////
void AlgorithmUpdater::run()
{
	ros::Duration l_duration(5.0);

	ROS_INFO("Application is running on AlgorithmUpdater thread.");

	std::vector<int8_t> l_data;
	int count = 0;
	while (ros::ok())
	{
		m_caller->wait();
		
		this->publish(l_data);

		m_caller->reset();
		
		ros::spinOnce();

		l_duration.sleep();
		++count;
		
		ROS_INFO("AlgorithmUpdater Run.");
	}
}

////////////////////////////////////////////////////////////
AlgorithmUpdater::AlgorithmUpdater(std::shared_ptr<LearningWorld> algorithm_, std::shared_ptr<AgentCall> caller_)
	: ThreadBase()
	, m_algorithm(algorithm_)
	, m_caller(caller_)
{
  m_neighboursPub = m_node.advertise<nav_msgs::OccupancyGrid>("/simulator/neighbours", 1);
}

////////////////////////////////////////////////////////////
AlgorithmUpdater::~AlgorithmUpdater()
{}

////////////////////////////////////////////////////////////
void AlgorithmUpdater::publish(std::vector<int8_t> & data_)
{
  m_algorithm->evaluateScreenshot();
  m_algorithm->getNeighboursScreeShot(data_);
  nav_msgs::OccupancyGrid l_msg;
  l_msg.data = data_;
  m_neighboursPub.publish(l_msg);
}