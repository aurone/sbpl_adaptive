cmake_minimum_required(VERSION 2.8)
project(sbpl_adaptive_components)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS roscpp)
MESSAGE("ROSCPP: (${roscpp_VERSION}) ${roscpp_VERSION_MAJOR}.${roscpp_VERSION_MINOR}")
if(${roscpp_VERSION} VERSION_GREATER "1.9.55")
find_package(cmake_modules REQUIRED)
endif()

#find_package(Boost REQUIRED)
#include_directories(${Boost_INCLUDE_DIRS})

#find_package(Bullet REQUIRED)
#include_directories(${BULLET_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)
include_directories(${OROCOS_KDL_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sbpl_adaptive
    sbpl_adaptive_collision_checking
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)

# external include directories
add_definitions(-DROS)

catkin_package(
   INCLUDE_DIRS 
	include
   LIBRARIES 
	${PROJECT_NAME}
   CATKIN_DEPENDS roscpp sbpl_adaptive 
   DEPENDS sbpl
)

include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
link_directories(${SBPL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
            src/sbpl_adaptive_grid3D.cpp
            src/sbpl_expansion_grid3D.cpp)

target_link_libraries(${PROJECT_NAME} sbpl_adaptive sbpl)

install(
    TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

