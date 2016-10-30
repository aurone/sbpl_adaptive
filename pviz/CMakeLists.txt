cmake_minimum_required(VERSION 2.8)
project(pviz)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#set(BOOST_REQUIRED_VERSION "1.46.1")

find_package(cmake_modules REQUIRED)

find_package(Boost  REQUIRED) #${BOOST_REQUIRED_VERSION} EXACT
find_package(Bullet REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
#    ikfast_pr2
    kdl_parser
    roscpp
    sensor_msgs
    smpl
    tf
    trajectory_msgs
    visualization_msgs)


find_package(PkgConfig REQUIRED)

# external include directories
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${BULLET_INCLUDE_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

# internal include directories
#include_directories(door_msgs/msg_gen/cpp/include)

catkin_package(
   INCLUDE_DIRS 
	include
   LIBRARIES 
	${PROJECT_NAME}
#  DEPENDS system_lib
   CATKIN_DEPENDS 
    geometry_msgs
#    ikfast_pr2
    kdl_parser
    roscpp
    tf
    visualization_msgs
)

include_directories(include)

add_definitions(-DROS)

add_library(${PROJECT_NAME} 
	src/pviz.cpp 
	src/simpleviz.cpp)


install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

