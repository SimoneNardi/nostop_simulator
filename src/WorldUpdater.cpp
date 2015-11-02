#include "WorldUpdater.h"

#include "learningWorld.h"
#include "world.h"
#include "discretizedArea.h"

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
		// TODO ...update solo allo spostamento del ladro!
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
  m_monitorPub = m_node.advertise<nav_msgs::OccupancyGrid>("/simulator/monitor", 10);
  m_energyPub = m_node.advertise<nav_msgs::OccupancyGrid>("/simulator/energy", 10);
}

////////////////////////////////////////////////////////////
WorldUpdater::~WorldUpdater()
{}

////////////////////////////////////////////////////////////
void WorldUpdater::publish(std::vector<int8_t> & data_)
{
  m_algorithm->worldUpdate();
  
  nav_msgs::OccupancyGrid l_msg_monitor;
  l_msg_monitor.info.height = m_algorithm->getWorld()->getSpace()->getNumRow();
  l_msg_monitor.info.width = m_algorithm->getWorld()->getSpace()->getNumCol();
  std::vector<int8_t> l_data_monitor;
  m_algorithm->getMonitorScreenShot(l_data_monitor);
  l_msg_monitor.data = l_data_monitor;
  m_monitorPub.publish(l_msg_monitor);
  
  nav_msgs::OccupancyGrid l_msg_energy;
  l_msg_energy.info.height = m_algorithm->getWorld()->getSpace()->getNumRow();
  l_msg_energy.info.width = m_algorithm->getWorld()->getSpace()->getNumCol();
  std::vector<int8_t> l_data_energy;
  m_algorithm->getEnergyScreenShot(l_data_energy);
  l_msg_energy.data = l_data_energy;
  m_energyPub.publish(l_msg_energy);
}