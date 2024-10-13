cmake_minimum_required(VERSION 3.0.2)
project(test_pcl)
set(CMAKE_BUILD_TYPE Debug)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(LIB_OPENCV_IMGPROC_DIR /usr/local/lib) 
add_library(CSF SHARED IMPORTED)
set_target_properties(CSF PROPERTIES IMPORTED_LOCATION ${LIB_OPENCV_IMGPROC_DIR}/libCSF.a)

find_package(PCL 1.10 REQUIRED)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
)
# find_package(PCL  REQUIRED)
# find_package(PCL  COMPONENTS common visualization io surface REQUIRED)
FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES CSF
   CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs
#  DEPENDS system_lib
)


include_directories(
  include/test_pcl
  ${catkin_INCLUDE_DIRS}
  # /usr/local/include/OpenNURBS/
  # /usr/local/include/opennurbsStatic/
  # /usr/local/include/surface/on_nurbs/
)
 add_executable(${PROJECT_NAME}_node 
  src/test_pcl_node.cpp
  )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  CSF
  ${catkin_LIBRARIES}
  pcl_visualization
  # /usr/local/lib/libOpenNURBS.so  
  # /usr/local/lib/libandroid_uuid.a
  # /usr/local/lib/libopennurbs_public_freetype.a
  # /usr/local/lib/libzlib.a
  # /usr/local/lib/libopennurbsStatic.a
  # /usr/local/lib/libpcl_surface.so
)
