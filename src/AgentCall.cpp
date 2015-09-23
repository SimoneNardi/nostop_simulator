#include "AgentCall.h"
#include "agent.h"

#include <memory>

#include "std_msgs/Bool.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
AgentCall::AgentCall(std::set< std::shared_ptr<Guard> >& agent_)
: m_notified( false )
, m_call()
{
	m_pub = m_node.advertise<std_msgs::Bool>("1", 1);
  
	for(auto it = agent_.begin(); it != agent_.end(); ++it)
	{
		GuardPtr l_agent = *it;
		int l_id = l_agent->getID();
		m_call.insert( std::make_pair( l_id, l_agent->getStatus() ) );
	}
}

/////////////////////////////////////////////
bool AgentCall::readyToGo() const
{
	Lock1 l_locker( m_lockCall );
	std_msgs::Bool l_msg;
	l_msg.data = true;
	for(auto it = m_call.begin(); it != m_call.end(); ++it)
	{
		if (it->second != Agent::STANDBY)
		{
			l_msg.data = false;
			break;
		}
	}
	
	m_pub.publish<std_msgs::Bool>(l_msg);
	return l_msg.data;
}

/////////////////////////////////////////////
void AgentCall::update(std::set< std::shared_ptr<Guard> >& agent_)
{
  
	for(auto it = agent_.begin(); it != agent_.end(); ++it)
	{
		GuardPtr l_agent = *it;
		Lock1 l_locker( m_lockCall );
		m_call[ l_agent->getID() ] = l_agent->getStatus();
	}
}

/////////////////////////////////////////////
void AgentCall::update(std::shared_ptr<Guard> agent_)
{
	Lock1 l_locker( m_lockCall );
	m_call[ agent_->getID() ] = agent_->getStatus();
}

/////////////////////////////////////////////
void AgentCall::wait()
{
	Lock1 l_locker( m_lockCall );
	while(!m_notified) // used to avoid spurious wakeups 
	{
	  m_signalCall.wait(l_locker);
	}
}

/////////////////////////////////////////////
void AgentCall::notify()
{
	Lock1 l_locker(m_lockCall);
	m_notified = true;
	m_signalCall.notify_all();
}

/////////////////////////////////////////////
void AgentCall::reset()
{
	Lock1 l_locker(m_lockCall);
	m_notified = false;
	for(auto it = m_call.begin(); it != m_call.end(); ++it)
	{
		it->second = Agent::ACTIVE;
	}
}