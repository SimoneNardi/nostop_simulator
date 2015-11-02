#include "PlayerRemover.h"

#include "learningWorld.h"
#include "GazeboDriver.h"
#include "ros/ros.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

////////////////////////////////////////
PlayerRemover::PlayerRemover(std::shared_ptr<Robotics::GameTheory::LearningWorld> coverage_, std::shared_ptr<GazeboDriver> gazebo_driver_)
: ThreadBase()
, m_coverage(coverage_)
, m_gazebo_driver(gazebo_driver_)
{}

////////////////////////////////////////
void PlayerRemover::run()
{
  return;
}

//////////////////////////////////////////////
PlayerRemover::~PlayerRemover()
{
  this->stop();
}