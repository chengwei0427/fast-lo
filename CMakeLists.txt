cmake_minimum_required(VERSION 2.8.3)
project(floam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
# find_package(Eigen3)
# if(NOT EIGEN3_FOUND)
#   # Fallback to cmake_modules
#   find_package(cmake_modules REQUIRED)
#   find_package(Eigen REQUIRED)
#   set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
#   set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
#   # Possibly map additional variables to the EIGEN3_ prefix.
# else()
#   set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
# endif()
set(EIGEN3_FOUND true)
find_package(Eigen3 REQUIRED)
IF(EIGEN3_FOUND) 
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
MESSAGE("EIGEN3 [OK]:" ${EIGEN3_INCLUDE_DIR})
ENDIF(EIGEN3_FOUND)

# find_package(PCL REQUIRED PATHS /home/cc/workspace/3rdparty/pcl1.7.2)
# set(PCL_INCLUDE_DIRS /home/cc/workspace/3rdparty/pcl1.9/include/pcl-1.9)
# set(PCL_LIBRARY_DIRS /home/cc/workspace/3rdparty/pcl1.9//lib)
# file(GLOB PCL_LIBRARIES ${PCL_LIBRARY_DIRS}/*.a ${PCL_LIBRARY_DIRS}/*.so*)
find_package(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

set(3RDPARTY_DIR /home/zhaochengwei/workspace/3rdparty)
find_package(Ceres REQUIRED PATHS ${3RDPARTY_DIR}/ceres-1.14 )

message(STATUS "ceres: " ${GERES_LIBRARIES})
# find_package(Ceres REQUIRED)

message(STATUS "pcl dir: ${PCL_INCLUDE_DIRS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL Ceres 
  INCLUDE_DIRS include
)



add_executable(floam_laser_processing_node src/laserProcessingNode.cpp src/laserProcessingClass.cpp src/lidar.cpp)
target_link_libraries(floam_laser_processing_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(floam_odom_estimation_node src/odomEstimationNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/odomEstimationClass.cpp)
target_link_libraries(floam_odom_estimation_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(floam_laser_mapping_node src/laserMappingNode.cpp src/laserMappingClass.cpp src/lidar.cpp)
target_link_libraries(floam_laser_mapping_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(floam_feature_extract_node src/featureExtractNode.cpp src/featureExtractClass.cpp )
target_link_libraries(floam_feature_extract_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(floam_laser_odom_node 
                                  src/laserOdomNode.cpp 
                                  src/lidarOptimization.cpp 
                                  src/lidar.cpp 
                                  src/laserOdomClass.cpp 
                                  include/ikd-Tree/ikd_Tree.cpp
                                  src/lidarOpt.cpp
                                  src/utils.cpp)
target_link_libraries(floam_laser_odom_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})