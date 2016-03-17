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
  
  std::shared_ptr<World> l_world = m_algorithm->getWorld();
  std::shared_ptr<DiscretizedArea> l_space = l_world->getSpace();
  
  Real2D l_origin2D = l_space->getOrigin();
  geometry_msgs::Pose l_origin;
  l_origin.position.x = l_origin2D[0];
  l_origin.position.y = l_origin2D[1];
  
  std::vector<double> l_data_monitor;
  m_algorithm->getMonitorScreenShot(l_data_monitor);
  
  double l_min = Math::Infinity, l_max = -Math::Infinity;
  data_.resize(l_data_monitor.size());
  for(auto i = 0; i < l_data_monitor.size(); ++i)
  {
    if(l_min > l_data_monitor[i])
      l_min = l_data_monitor[i];
      
    if(l_max < l_data_monitor[i])
      l_max = l_data_monitor[i];
  }
  
  for(auto i = 0; i < l_data_monitor.size(); ++i)
    data_[i] = (l_data_monitor.at(i) - l_min) / l_max * 100.;

  nav_msgs::OccupancyGrid l_msg_monitor;
  l_msg_monitor.info.height = l_space->getNumRow();
  l_msg_monitor.info.width = l_space->getNumCol();
  l_msg_monitor.info.resolution = (l_space->getXStep() + l_space->getYStep()) / 2;
  l_msg_monitor.info.origin = l_origin;
  l_msg_monitor.header.frame_id = "map";
  l_msg_monitor.data = data_;
  
  m_monitorPub.publish(l_msg_monitor);
 
  std::vector<double> l_data_energy;
  m_algorithm->getEnergyScreenShot(l_data_energy);
  
  l_min = Math::Infinity;
  l_max = -Math::Infinity;
  data_.resize(l_data_energy.size());
  for(auto i = 0; i < l_data_energy.size(); ++i)
  {
    if(l_min > l_data_energy[i])
      l_min = l_data_energy[i];
      
    if(l_max < l_data_energy[i])
      l_max = l_data_energy[i];
  }  
  
  for(auto i = 0; i < l_data_energy.size(); ++i)
    data_[i] = (l_data_energy.at(i) - l_min) / l_max * 100.;
  
  nav_msgs::OccupancyGrid l_msg_energy;
  l_msg_energy.info = l_msg_monitor.info;
  l_msg_energy.header = l_msg_monitor.header;
  l_msg_energy.data = data_;
    
  m_energyPub.publish(l_msg_energy);
}