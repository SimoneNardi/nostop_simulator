#include "AlgorithmUpdater.h"
#include "learningWorld.h"
#include "world.h"
#include "discretizedArea.h"
#include "AgentCall.h"

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

///////////////////////////////////////////////////////////
void AlgorithmUpdater::run()
{
	ros::Duration l_duration(1.0);

	ROS_INFO("Application is running on AlgorithmUpdater thread.");

	std::vector<int8_t> l_data;
	int count = 0;
	while (ros::ok())
	{
		m_caller->wait();
		
		this->publish(l_data);

		//m_caller->reset();
		
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
  std::vector<double> l_data;
  m_algorithm->getNeighboursScreeShot(l_data);
    
  double l_min = Math::Infinity, l_max = -Math::Infinity;
  data_.resize(l_data.size());
  for(auto i = 0; i < l_data.size(); ++i)
  {
    if(l_min > l_data[i])
      l_min = l_data[i];
      
    if(l_max < l_data[i])
      l_max = l_data[i];
  }  
  
  for(auto i = 0; i < l_data.size(); ++i)
    data_[i] = (l_data.at(i) - l_min) / l_max * 100.;
  
  std::shared_ptr<World> l_world = m_algorithm->getWorld();
  std::shared_ptr<DiscretizedArea> l_space = l_world->getSpace();
  
  Real2D l_origin2D = l_space->getOrigin();
  geometry_msgs::Pose l_origin;
  l_origin.position.x = l_origin2D[0];
  l_origin.position.y = l_origin2D[1];
  
  nav_msgs::OccupancyGrid l_msg;  
  l_msg.info.height = l_space->getNumRow();
  l_msg.info.width = l_space->getNumCol();
  l_msg.info.resolution = (l_space->getXStep() + l_space->getYStep()) / 2;
  l_msg.info.origin = l_origin;
  l_msg.header.frame_id = "world";
  //l_msg.header.seq = ros::Time::now();
    
  l_msg.data = data_;
  m_neighboursPub.publish(l_msg);
}