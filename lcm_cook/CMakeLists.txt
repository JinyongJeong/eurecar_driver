cmake_minimum_required(VERSION 2.8.3)
project(lcm_cook)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "./cmake")
file (GLOB MODULES "${CMAKE_MODULE_PATH}/*.cmake")
foreach (MODULE ${MODULES})
  include (${MODULE})
endforeach ()

set(LCM_DIR "${CATKIN_DEVEL_PREFIX}/include")

include_directories(
  ${LCM_DIR}
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

include(${CMAKE_CURRENT_SOURCE_DIR}/lcmdefs/PACKAGES.cmake)
lcmtypes_build()
