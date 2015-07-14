#include "AreaCreator.h"

#include "area.h"
#include "structuredArea.h"

#include "nostop_agent/ShapeData.h"

using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////////
AreaCreator::AreaCreator() 
	: m_external()
	, m_internal()
{
	// Collect point from rviz or read configuration file if not available
	std::vector<IDSReal2D> l_points;

	l_points.push_back(IDSReal2D (-50.,-50.) );
	l_points.push_back(IDSReal2D (50.,-50.) );
	l_points.push_back(IDSReal2D (50.,50.) );
	l_points.push_back(IDSReal2D (-50.,50.) );

	m_external = l_points;
}

AreaCreator::AreaCreator(nostop_agent::ShapeData external_, std::vector<nostop_agent::ShapeData> internal_) 
	: m_external()
	, m_internal()
{
	// Collect point from rviz or read configuration file if not available
	
	std::vector<IDSReal2D> l_points;
	for(size_t i=0; i < external_.vertex.size(); i+=2)
	{
	  l_points.push_back( IDSReal2D (external_.vertex[2*i+0],external_.vertex[2*i+1]) );
	}
	m_external = l_points;
	
	for(size_t j=0; j < internal_.size(); ++j)
	{
	  std::vector<IDSReal2D> l_obs;
	  for(size_t i=0; i < internal_[j].vertex.size(); i+=2)
	  {
	      l_obs.push_back( IDSReal2D (external_.vertex[2*i+0],external_.vertex[2*i+1]) );
	  }
	  if (l_obs.size() > 2)
	    m_internal.push_back(l_obs);
	}
}

/////////////////////////////////////////////////
AreaPtr AreaCreator::getArea() const
{
	AreaPtr l_area = std::make_shared<StructuredArea>(m_external);
	return l_area;
}