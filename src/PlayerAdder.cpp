#include "PlayerAdder.h"

#include "learningWorld.h"
#include "GazeboDriver.h"

#include "GuardAdder.h"
#include "ThiefAdder.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//////////////////////////////////////////////
PlayerAdder::PlayerAdder(std::shared_ptr<Robotics::GameTheory::LearningWorld> coverage_, std::shared_ptr<GazeboDriver> gazebo_driver_)
: m_guard_adder(nullptr)
, m_thief_adder(nullptr)
{
  m_guard_adder = std::make_shared<GuardAdder>(coverage_, gazebo_driver_);
  m_thief_adder = std::make_shared<ThiefAdder>(coverage_, gazebo_driver_);
}

//////////////////////////////////////////////
void PlayerAdder::start()
{
  m_guard_adder->start();
  m_thief_adder->start();
}

//////////////////////////////////////////////
void PlayerAdder::stop()
{
  m_guard_adder->stop();
  m_thief_adder->stop();
}

//////////////////////////////////////////////
PlayerAdder::~PlayerAdder()
{
  this->stop();
}