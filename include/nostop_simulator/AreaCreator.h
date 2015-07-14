////////////////////////////////////////////////////
//	AreaCreator.h
//	Created on:	12-jun-2015
//	Original author: Simone Nardi
////////////////////////////////////////////////////
#ifndef AREA_CREATOR_H
#define AREA_CREATOR_H
#pragma once

#include "agent.h"
#include "area.h"

#include "nostop_agent/AreaData.h"

#include <vector>
#include <memory>

namespace Robotics 
{
	namespace GameTheory
	{

		class Agent;

		class AreaCreator
		{
			std::vector<IDSReal2D> m_external;
			std::vector< std::vector<IDSReal2D> > m_internal;

		public:
			AreaCreator();
			
			AreaCreator(nostop_agent::ShapeData external_, std::vector<nostop_agent::ShapeData> internal_);

		public:      
			AreaPtr getArea() const;
		};

	}
}


#endif // CREATE_AREA_H