#include "ros/ros.h"
#include "nostop_agent/GuardBenefitData.h"
#include "nostop_agent/GuardNeighboursData.h"
#include "nostop_agent/AreaData.h"

#include "agent.h"
#include "area.h"

#include "AreaCreator.h"
#include "PlayersMaker.h"
#include "PlayerAdder.h"
#include "PlayerRemover.h"
#include "learningWorld.h"
#include "CoverageApplication.h"

#include "learningAlgorithm.h"
#include "world.h"
#include "discretizedArea.h"

#include <set>

#include <stdio.h>
#include <stdlib.h>

#include <memory>

#include "GazeboDriver.h"

////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Simulator");
	
	ROS_INFO("Simulator is running.");
	
	std::shared_ptr<GazeboDriver> l_gazebo_driver = std::make_shared<GazeboDriver>();
	
	ROS_INFO("Gazebo Driver is created.");
	
	int l_number_of_guards(0), l_number_of_thieves(0);
	
	std::string l_str;
	ros::NodeHandle l_node("~");
	if (l_node.getParam("number_of_guards", l_str))
	{
	  l_number_of_guards = std::stoi(l_str);
	  ROS_INFO("Received number of guards: %d", l_number_of_guards);
	}
	else
	{
	  l_number_of_guards = 1;
	  ROS_ERROR("Apriori number of guards: %d", l_number_of_guards);
	}
	
	if (l_node.getParam("number_of_thieves", l_str))
	{
	  l_number_of_thieves = std::stoi(l_str);
	  ROS_INFO("Received number of thieves: %d", l_number_of_thieves);
	}
	else
	{
	  l_number_of_thieves = 0;
	  ROS_ERROR("Apriori number of thieves: %d", l_number_of_thieves);
	}

	/////////////////////////////////////////////////
	// AREA CREATOR
	Robotics::GameTheory::AreaPtr l_area = nullptr;
	ros::NodeHandle l_nodeArea;
	ros::ServiceClient l_clientArea = l_nodeArea.serviceClient<nostop_agent::AreaData>("AreaInitializer");
	nostop_agent::AreaData l_srvArea;
	if (l_clientArea.call(l_srvArea))
	{
		ROS_INFO("Area description received");
		// creazione dell'area:
		Robotics::GameTheory::AreaCreator l_areaCreator(l_srvArea.response.external, l_srvArea.response.internal);
		l_area = l_areaCreator.getArea();
	}
	else
	{
		ROS_ERROR("Failed to call service AreaInitializer");
		Robotics::GameTheory::AreaCreator l_areaCreator;
		l_area = l_areaCreator.getArea();
	}
	
	ROS_INFO("Area is created.");

	/////////////////////////////////////////////////
	// PLAYER CREATOR
	Robotics::GameTheory::PlayersMaker l_playersCreator(l_area, l_number_of_guards, l_number_of_thieves, l_gazebo_driver);
	std::set<Robotics::GameTheory::AgentPtr> l_players = l_playersCreator.getPlayers();

	ROS_INFO("Players are connected.");

	/////////////////////////////////////////////////
	// LEARNING CREATOR
	std::shared_ptr<Robotics::GameTheory::LearningWorld> l_coverage = 
		std::make_shared<Robotics::GameTheory::LearningWorld>(l_players, l_area->discretize(), Robotics::GameTheory::DISL);

	ROS_INFO("Learning algorithm is created.");
	
	/////////////////////////////////////////////////
	// APPLICATION RUNNING
	Robotics::GameTheory::CoverageApplication l_application(l_coverage);
	l_application.start();
	
	ROS_INFO("Application is running.");
// 	std::shared_ptr<Robotics::GameTheory::PlayerAdder> l_player_adder = std::make_shared<Robotics::GameTheory::PlayerAdder>(l_coverage, l_gazebo_driver);
// 	std::shared_ptr<Robotics::GameTheory::PlayerRemover> l_player_remover = std::make_shared<Robotics::GameTheory::PlayerRemover>(l_coverage, l_gazebo_driver);
// 		
// 	l_player_adder->start();
// 	l_player_remover->start();
	/////////////////////////////////////////////////
	// WAIT FOR ROS MESSAGES
	ros::spin();

// 	l_player_adder->stop();
// 	l_player_remover->stop();
	
	ROS_INFO("Ending process.");

	return 0;
}