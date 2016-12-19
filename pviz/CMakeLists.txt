cmake_minimum_required(VERSION 2.8)
project(pviz)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(cmake_modules REQUIRED)

find_package(Boost REQUIRED)
find_package(Bullet REQUIRED)
find_package(orocos_kdl)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    geometry_msgs
    kdl_parser
    roscpp
    sensor_msgs
    smpl
    tf
    trajectory_msgs
    visualization_msgs)

find_package(Eigen REQUIRED)

find_package(PkgConfig REQUIRED)

# external include directories
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${BULLET_INCLUDE_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS 
       include
   LIBRARIES 
	${PROJECT_NAME}
   CATKIN_DEPENDS 
    geometry_msgs
    kdl_parser
    roscpp
    tf
    visualization_msgs
)

include_directories(include)

add_definitions(-DROS)

add_library(${PROJECT_NAME} src/simpleviz.cpp)

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

