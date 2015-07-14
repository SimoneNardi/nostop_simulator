#include "CoverageApplication.h"
#include "AlgorithmUpdater.h"
#include "AgentUpdater.h"
#include "WorldUpdater.h"
#include "VisualUpdater.h"
#include "LearningWorld.h"
#include "AgentCall.h"

#include "world.h"
#include "guard.h"

#include "ros/ros.h"

#include <memory>
#include <set>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

///////////////////////////////////////////////////////////
CoverageApplication::CoverageApplication( LearningWorldPtr algorithm_ )
	: m_mutex()
	, m_algorithm(nullptr)
	, m_agentUpdater(nullptr)
	, m_worldUpdater(nullptr)
	, m_algorithmUpdater(nullptr)
	, m_visualUpdater(nullptr)
	, m_caller(nullptr)
{
	m_algorithm = algorithm_;
	if (m_algorithm)
	{
		auto l_guards = m_algorithm->getWorld()->getGuards();
	  
		m_caller = std::make_shared<AgentCall>( l_guards );
	  	  
		m_agentUpdater = std::make_shared<AgentUpdater>(m_algorithm, m_caller);
		m_algorithmUpdater = std::make_shared<AlgorithmUpdater>(m_algorithm, m_caller);
		m_worldUpdater = std::make_shared<WorldUpdater>(m_algorithm);
		m_visualUpdater = std::make_shared<VisualUpdater>(m_algorithm);
	}
}

///////////////////////////////////////////////////////////
CoverageApplication::~CoverageApplication()
{}

bool CoverageApplication::start()
{
	// initialize algorithm
	m_algorithm->start();
  
	// start updater threads
	if (m_agentUpdater)
		m_agentUpdater->start();
	
	if (m_worldUpdater)
		m_worldUpdater->start();

	if (m_algorithmUpdater)
		m_algorithmUpdater->start();

	if (m_visualUpdater)
		m_visualUpdater->start();
}