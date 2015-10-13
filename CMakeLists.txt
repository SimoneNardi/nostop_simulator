cmake_minimum_required(VERSION 2.8.3)
project(nostop_simulator)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  message_generation 
  base_math 
  nostop_learning 
  tf
  nostop_agent
  nostop_gazebo_simulator
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  DEPENDS base_math nostop_learning nostop_agent nostop_gazebo_simulator gazebo_ros
)

###########
## Build ##
###########

include_directories(include/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
include_directories(src/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_INSTALL_PREFIX}/include) # this line is to find lemon include files.

set(simulator_SOURCES
        src/AgentCall.cpp
        src/AgentInterface.cpp
        src/AgentUpdater.cpp
        src/AlgorithmUpdater.cpp
        src/CoverageApplication.cpp
        src/PlayerIDSender.cpp
        src/PlayersMaker.cpp
        src/Simulator.cpp
        src/VisualUpdater.cpp
        src/WorldUpdater.cpp
)

add_executable(Simulator ${simulator_SOURCES})
target_link_libraries(Simulator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} base_math lemon nostop_agent nostop_learning nostop_gazebo_simulator)
add_dependencies(Simulator nostop_simulator_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS Simulator
 EXPORT nostop_simualtor
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
