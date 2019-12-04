cmake_minimum_required(VERSION 2.8.3)
project(nav_position)

set(LCM_DIR "${CATKIN_DEVEL_PREFIX}/include")
add_definitions(-std=c++11)
# CMAKE TWEAKS
#========================================================================
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE RelWithDebInfo)
endif ()

# INCLUDE LOCAL MODULES
#========================================================================
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "./cmake")
file (GLOB MODULES "${CMAKE_MODULE_PATH}/*.cmake")
foreach (MODULE ${MODULES})
  include (${MODULE})
endforeach ()

find_package(catkin REQUIRED cmake_modules
  COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  visualization_msgs
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  pcl_msgs
)
find_package(Eigen REQUIRED)
find_library (LCM_LIBRARIES LCM)

catkin_package (
  #INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    pcl_ros
    pcl_conversions
    pcl_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########
set (SRC
  src/main.cpp
  src/nav_position.cpp src/nav_position.h
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${LCM_DIR}
)

add_executable(nav_position ${SRC})
target_link_libraries(nav_position 
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${LCM_LIBRARIES}
)
