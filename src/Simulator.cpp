#include "ros/ros.h"
#include "nostop_agent/GuardBenefitData.h"
#include "nostop_agent/GuardNeighboursData.h"
#include "nostop_agent/AreaData.h"

#include "agent.h"
#include "area.h"

#include "AreaCreator.h"
#include "PlayersMaker.h"
#include "LearningWorld.h"
#include "CoverageApplication.h"

#include "learningAlgorithm.h"
#include "world.h"
#include "discretizedArea.h"

#include <set>

#include <stdio.h>
#include <stdlib.h>

#include <memory>

//#include "nostopConfig.h"

std::shared_ptr<Robotics::GameTheory::LearningWorld> g_coverage = nullptr;

////////////////////////////////////////////////////////////////
bool getBenefit(
	nostop_agent::GuardBenefitData::Request  &req,
	nostop_agent::GuardBenefitData::Response &res)
{
	Robotics::GameTheory::WorldPtr l_world = g_coverage->getWorld();
	Robotics::GameTheory::DiscretizedAreaPtr l_space = l_world->getSpace();
	Robotics::GameTheory::SquarePtr l_square = l_space->getSquare(req.row, req.col);

	// calcolare il benefit associato all'area (col,row)
	if ( l_square && l_square->isValid() )
	{
		res.benefit = l_square->getThiefValue();
	}
	else
		res.benefit = 0;

	ROS_INFO("Benefit requested for: col=%ld, row=%ld", (long int)req.col, (long int)req.row);
	ROS_INFO("Sending back response: %ld", (long int)res.benefit);
	return true;
}

////////////////////////////////////////////////////////////////
bool getNeighbours(
	nostop_agent::GuardNeighboursData::Request  &req,
	nostop_agent::GuardNeighboursData::Response &res)
{
	Robotics::GameTheory::WorldPtr l_world = g_coverage->getWorld();
	Robotics::GameTheory::DiscretizedAreaPtr l_space = l_world->getSpace();
	Robotics::GameTheory::SquarePtr l_square = l_space->getSquare(req.row, req.col);

	// calcolare il numero di robot che monitora l'area (col,row)
	if ( l_square && l_square->isValid() )
	{
		res.neighbors = l_square->getTheNumberOfAgent();
	}
	else
		res.neighbors = 0;

	ROS_INFO("Neighbours requested for: col=%ld, row=%ld", (long int)req.col, (long int)req.row);
	ROS_INFO("Sending back response: %ld", (long int)res.neighbors);
	return true;
}

////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Simulator");

	ROS_INFO("Simulator is running.");

	/////////////////////////////////////////////////
	// ROS MAIN SERVICE
	ros::NodeHandle n;
	ros::ServiceServer serviceBenefit = n.advertiseService("GuardBenefit", getBenefit);
	ros::ServiceServer serviceNeighbours = n.advertiseService("GuardNeighbours", getNeighbours);
	
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
	Robotics::GameTheory::PlayersMaker l_playersCreator(l_area, 3, 1);
	std::set<Robotics::GameTheory::AgentPtr> l_players = l_playersCreator.getPlayers();

	ROS_INFO("Players are connected.");

	/////////////////////////////////////////////////
	// LEARNING CREATOR
	g_coverage = std::make_shared<Robotics::GameTheory::LearningWorld>(l_players, l_area->discretize(), Robotics::GameTheory::DISL);

	ROS_INFO("Learning algorithm is created.");
	
	/////////////////////////////////////////////////
	// APPLICATION RUNNING
	Robotics::GameTheory::CoverageApplication l_application(g_coverage);
	l_application.start();

	ROS_INFO("Application is running.");
	
	/////////////////////////////////////////////////
	// WAIT FOR ROS MESSAGES
	ros::spin();

	ROS_INFO("Ending process.");

	return 0;
}