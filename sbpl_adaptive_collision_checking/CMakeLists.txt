cmake_minimum_required(VERSION 2.8)
project(sbpl_adaptive_collision_checking)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

execute_process(
	COMMAND rosversion -d
	COMMAND xargs echo -n
	OUTPUT_VARIABLE ROS_DISTRO)

MESSAGE("ROS: (${ROS_DISTRO})")

add_definitions(-D__ROS_DISTRO_${ROS_DISTRO}__)

find_package(catkin REQUIRED COMPONENTS roscpp)
MESSAGE("ROSCPP: (${roscpp_VERSION}) ${roscpp_VERSION_MAJOR}.${roscpp_VERSION_MINOR}")
if(${roscpp_VERSION} VERSION_GREATER "1.9.55")
    find_package(cmake_modules REQUIRED)
endif()

#find_package(Boost REQUIRED)
#include_directories(${Boost_INCLUDE_DIRS})

find_package(Bullet REQUIRED)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)
include_directories(${OROCOS_KDL_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    leatherman
    moveit_core
    moveit_msgs
    moveit_ros_planning
    octomap_msgs
    sbpl_adaptive
    smpl
    sbpl_geometry_utils
    tf
    tf_conversions)

# external include directories
add_definitions(-DROS)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    #DEPENDS sbpl
    CATKIN_DEPENDS roscpp leatherman smpl sbpl_geometry_utils)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(
    sbpl_adaptive_collision_checking
    src/sbpl_collision_model.cpp
    src/sbpl_collision_space.cpp
    src/urdf_collision_model.cpp)

target_link_libraries(sbpl_adaptive_collision_checking ${catkin_LIBRARIES})
set_target_properties(sbpl_adaptive_collision_checking PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations")

add_executable(
    sbpl_adaptive_collision_checking_test
    src/test.cpp)
target_link_libraries(
    sbpl_adaptive_collision_checking_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES})

install(
    TARGETS sbpl_adaptive_collision_checking
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

