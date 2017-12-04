cmake_minimum_required(VERSION 2.8)
project(sbpl_adaptive_collision_checking)

set(CMAKE_BUILD_TYPE Release)
add_compile_options("-std=c++11")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(orocos_kdl REQUIRED)

find_package(sbpl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    leatherman
    moveit_core
    moveit_ros_planning
    eigen_conversions
    moveit_msgs
#    octomap_msgs
    sbpl_adaptive
    smpl
    sbpl_geometry_utils
    tf
    tf_conversions
    std_msgs
    geometry_msgs
    angles
    urdf
    visualization_msgs
)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR} ${SBPL_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME} ${SBPL_LIBRARIES}
    DEPENDS
        Boost
    CATKIN_DEPENDS
        angles
        geometry_msgs
        moveit_core
        moveit_msgs
        moveit_ros_planning
        roscpp
        sbpl_adaptive
        sbpl_geometry_utils
        smpl
        std_msgs
        tf_conversions
        urdf
        visualization_msgs
)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${SBPL_INCLUDE_DIRS})
include_directories(include)

add_library(
    sbpl_adaptive_collision_checking
    src/sbpl_collision_model.cpp
    src/sbpl_collision_space.cpp
    src/urdf_collision_model.cpp)

target_link_libraries(sbpl_adaptive_collision_checking ${catkin_LIBRARIES})

add_executable(sbpl_adaptive_collision_checking_test src/test.cpp)
target_link_libraries(
    sbpl_adaptive_collision_checking_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES})

install(
    TARGETS sbpl_adaptive_collision_checking
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

