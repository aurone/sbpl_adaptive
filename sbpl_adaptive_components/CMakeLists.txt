cmake_minimum_required(VERSION 2.8.12)
project(sbpl_adaptive_components)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        leatherman
        roscpp
        sbpl_adaptive
        sbpl_adaptive_collision_checking
        tf
        visualization_msgs)

find_package(Eigen REQUIRED)

find_package(sbpl REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        leatherman
        roscpp
        sbpl_adaptive
        sbpl_adaptive_collision_checking
        tf
        visualization_msgs
    DEPENDS sbpl)

add_definitions(-DROS)
add_definitions(${EIGEN_DEFINITIONS})

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(
    ${PROJECT_NAME}
    src/adaptive_grid_3d.cpp
    src/expansion_grid_3d.cpp)
target_link_libraries(${PROJECT_NAME} sbpl_adaptive ${SBPL_LIBRARIES})

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

