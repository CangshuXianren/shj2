cmake_minimum_required(VERSION 2.8.3)
project(ndt_test)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  # livox_ros_driver
  message_generation
#  eigen_conversions
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

#message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIR}
#  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRSRY_DIRS}
  ${OpenCV_LIBRSRY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS
)

add_executable(ndt_node ndt_node.cpp voxel_grid_large.cpp)
target_link_libraries(ndt_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBRSRY_DIRS})
target_include_directories(ndt_node PRIVATE )

# add_executable(pcd_save pcdSave.cpp)
# target_link_libraries(pcd_save ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBRSRY_DIRS})
# target_include_directories(pcd_save PRIVATE )
