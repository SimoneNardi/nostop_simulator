#include "WorldUpdater.h"
#include "learningWorld.h"

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

///////////////////////////////////////////////////////////
void WorldUpdater::run()
{
	ros::Rate loop_rate(1);

	ROS_INFO("Application is running on WorldUpdater thread.");

	std::vector<int8_t> l_data;
	
	int count = 0;
	while (ros::ok())
	{
		this->publish(l_data);
		
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
{
  m_monitorPub = m_node.advertise<nav_msgs::OccupancyGrid>("/monitor/update", 10);
  m_energyPub = m_node.advertise<nav_msgs::OccupancyGrid>("/energy/update", 10);
}

////////////////////////////////////////////////////////////
WorldUpdater::~WorldUpdater()
{}

////////////////////////////////////////////////////////////
void WorldUpdater::publish(std::vector<int8_t> & data_)
{
  m_algorithm->worldUpdate();
  
  nav_msgs::OccupancyGrid l_msg;
    
  m_algorithm->getMonitorScreenShot(data_);
  l_msg.data = data_;
  m_monitorPub.publish(l_msg);
  
  m_algorithm->getEnergyScreenShot(data_);
  l_msg.data = data_;
  m_energyPub.publish(l_msg);
}