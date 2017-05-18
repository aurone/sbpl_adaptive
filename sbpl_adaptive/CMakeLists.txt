cmake_minimum_required(VERSION 2.8)
project(sbpl_adaptive)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(
    catkin
    REQUIRED
    COMPONENTS leatherman roscpp smpl std_msgs visualization_msgs)

find_package(PkgConfig REQUIRED)

pkg_check_modules(gsl REQUIRED gsl)

find_package(sbpl REQUIRED)

# external include directories

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS leatherman roscpp smpl std_msgs visualization_msgs
    DEPENDS sbpl)

#add_definitions(-DROS)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(
    ${PROJECT_NAME}
    src/adaptive_grid_3d.cpp
    src/sparse_adaptive_grid_3d.cpp
    src/common.cpp
    src/core/search/adaptive_planner.cpp
    src/core/search/araplanner_ad.cpp
    src/mrep/graph/adaptive_state_representation.cpp
    src/mrep/graph/multirep_adaptive_discrete_space.cpp
    src/mrep/graph/projection.cpp
    src/mrep/search/mhaplanner_ad.cpp)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${SBPL_LIBRARIES}
    ${gsl_LIBRARIES})

install(
    TARGETS sbpl_adaptive
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

