cmake_minimum_required(VERSION 2.8.3)
project(ase_exploration)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  map_msgs
  roscpp
  tf
  move_base
  actionlib
  actionlib_msgs
  message_generation
  dynamic_reconfigure
  frontier_exploration
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Use OpenMP
OPTION (USE_OpenMP "Use OpenMP" ON)
IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()
ENDIF()

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Explore.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  actionlib_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Planner.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# ROS interface for planner
add_library(exploration_planner_ros
  src/ExplorationPlannerROS/ExplorationPlannerROS.cpp
)

## ExplorationPlanner related libraries
add_library(ase_exploration_grid
  src/ExplorationPlanner/grid/PlanarGridOccupancyMap.cpp
  src/ExplorationPlanner/grid/PlanarGridBinaryMap.cpp
)

add_library(ase_exploration_trajectory
  src/ExplorationPlanner/trajectory/TrajectoryChecker.cpp
  src/ExplorationPlanner/trajectory/TrajectoryEvaluator.cpp
  src/ExplorationPlanner/trajectory/TrajectoryGenerator.cpp
  src/ExplorationPlanner/trajectory/TrajectoryValue.cpp
  src/ExplorationPlanner/trajectory/TrajectoryValueHandler.cpp
)

add_library(ase_exploration_planner
  src/ExplorationPlanner/planner/ExplorationTaskState.cpp
  src/ExplorationPlanner/planner/ICoolingSchedule.cpp
  src/ExplorationPlanner/planner/Particle.cpp
  src/ExplorationPlanner/planner/ParticleSet.cpp
  src/ExplorationPlanner/planner/SMCPlanner.cpp
  src/ExplorationPlanner/planner/SMCPlannerParameters.cpp
  src/ExplorationPlanner/kinematics/RangeConstraint.cpp
  src/ExplorationPlanner/kinematics/VelocityPlanarKinematics.cpp
  src/ExplorationPlanner/kinematics/PlanarPose.cpp
  src/ExplorationPlanner/kinematics/PlanarRobotVelCmd.cpp
  src/ExplorationPlanner/kinematics/PlanarRobotVelCmdSamplerIndependentGaussian.cpp
  src/ExplorationPlanner/kinematics/PlanarRobotVelCmdSamplerUniform.cpp
)

add_library(ase_exploration_sensor
  src/ExplorationPlanner/sensor/Bresenham2D.cpp
  src/ExplorationPlanner/sensor/Raytracer2D.cpp
  src/ExplorationPlanner/sensor/LaserScanner2D.cpp
)

add_library(ase_exploration_lookup_tables
  src/ExplorationPlanner/utils/EntropyLUT.cpp
  src/ExplorationPlanner/utils/LogOddsLUT.cpp
)

set(ASE_EXPLORATION_PLANNER_LIBS
  ase_exploration_grid
  ase_exploration_trajectory
  ase_exploration_planner
  ase_exploration_sensor
  ase_exploration_lookup_tables
)


# Exploration planner ROS needs the messages generated
add_dependencies(exploration_planner_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# Exploration planner
add_executable(${PROJECT_NAME}_planner_node src/exploration_planner_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(exploration_planner_ros
   ${catkin_LIBRARIES}
)

target_link_libraries(ase_exploration_planner
  ase_exploration_trajectory
)

target_link_libraries(${PROJECT_NAME}_planner_node
  ${catkin_LIBRARIES}
  exploration_planner_ros
  ${ASE_EXPLORATION_PLANNER_LIBS}
)