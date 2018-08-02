cmake_minimum_required(VERSION 2.8.7)
project(weaselball_gazebo)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

find_package(gazebo REQUIRED)
#------------------------------------------------------------------------------
## Dependencies
include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO REQUIRED gazebo )
endif()
message (${GAZEBO_LIBRARY_DIRS})
link_directories( ${GAZEBO_LIBRARY_DIRS} )
include_directories( ${GAZEBO_INCLUDE_DIRS} )

#if( PKG_CONFIG_FOUND )
#  pkg_check_modules( RAVELIN ravelin )
#endif()
#include_directories( ${RAVELIN_INCLUDE_DIRS} )
#link_directories( ${RAVELIN_LIBRARY_DIRS} )

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()
link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE-Terrain_INCLUDE_DIRS})


#------------------------------------------------------------------------------
## Code
set( LIBS 
  ${GAZEBO_LIBRARIES} 
  ${RAVELIN_LIBRARIES}
  libRavelin.so
) 

set( HDRS 
  include/common.h
  include/math.h
  include/log.h
  include/state.h
  include/virtual_time.h
  include/vicon.h
  include/video.h
  include/movie.h
  include/models.h
  include/weazelball.h
  include/gazebo_log.h
  include/bumpSensor.h
)

set( SRCS
)


add_library(configure SHARED plugins/initConditionHandler.cpp)
target_link_libraries(configure ${GAZEBO_LIBRARIES})

add_library(StateRecorder SHARED plugins/stateRecorder.cpp)
target_link_libraries(StateRecorder ${GAZEBO_LIBRARIES} ${CATKIN_LIBRARIES})


#add_library(bumpSensor SHARED plugins/bumpSensor.cpp)
#target_link_libraries(bumpSensor ${GAZEBO_LIBRARIES})

#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library( simulation-controller SHARED plugins/simulation_controller.cpp ${HDRS} ${SRCS} )
target_link_libraries( simulation-controller ${LIBS} )

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
