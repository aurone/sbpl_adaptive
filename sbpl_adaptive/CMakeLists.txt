cmake_minimum_required(VERSION 2.8)
project(sbpl_adaptive)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

find_package(PkgConfig REQUIRED)
#find_package(sbpl REQUIRED)

pkg_check_modules( gsl REQUIRED gsl )

find_package(sbpl REQUIRED)

MESSAGE("SBPL:")
MESSAGE("libdir: ${SBPL_LIBRARY_DIRS}")
MESSAGE("incdir: ${SBPL_INCLUDE_DIRS}")
MESSAGE("lib: ${SBPL_LIBRARIES}")

# external include directories

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    DEPENDS sbpl)

#add_definitions(-DROS)

include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

link_directories(${SBPL_LIBRARY_DIRS})
link_libraries(${SBPL_LIBRARIES})

add_library(
    ${PROJECT_NAME}
    src/common.cpp
    src/discrete_space_information/multirep_adaptive_environment.cpp
    src/planners/AdaptivePlanner/araplanner_ad.cpp
    src/planners/AdaptivePlanner/mhaplanner_ad.cpp
    src/planners/AdaptivePlanner/improved_mhaplanner_ad.cpp
    src/planners/AdaptivePlanner/adaptive_planner.cpp)

target_link_libraries(${PROJECT_NAME} sbpl ${gsl_LIBRARIES})

install(
    TARGETS sbpl_adaptive
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

