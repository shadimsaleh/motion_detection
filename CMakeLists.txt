cmake_minimum_required(VERSION 2.8.3)
project(motion_detection)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    pcl_ros
    tf
    visualization_msgs
    cv_bridge
    image_transport
)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    visualization_msgs
)

include_directories(
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

#...: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
endif(CMAKE_COMPILER_IS_GNUCXX)

add_definitions(-fpermissive)


### EXECUTABLES ###############################################
add_executable(motion_detection
  ros/src/motion_detection_node.cpp
  common/src/expected_flow_calculator.cpp
  common/src/optical_flow_calculator.cpp
  common/src/flow_clusterer.cpp
  common/src/flow_difference_calculator.cpp
  common/src/optical_flow_visualizer.cpp
  common/src/trajectory_visualizer.cpp
  common/src/vector_cluster.cpp
  common/src/point_cluster.cpp
  common/src/background_subtractor.cpp
  common/src/flow_neighbour_similarity_calculator.cpp
  common/src/outlier_detector.cpp
  common/src/motion_logger.cpp
  common/src/VarFlow.cpp
  common/src/slic.cpp
)
target_link_libraries(motion_detection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
