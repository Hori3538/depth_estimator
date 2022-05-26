cmake_minimum_required(VERSION 3.0.2)
project(depth_estimator)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    pluginlib
    nodelet
    camera_apps_msgs
    pcl_ros
    eigen_conversions
)
SET(OCV_VERSION "4.5.5")
find_package(OpenCV ${OCV_VERSION})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES depth_estimator
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(depth_estimator_node src/depth_estimator_node.cpp src/depth_estimator.cpp)
add_dependencies(depth_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    depth_estimator_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_library(depth_estimator_nodelet src/depth_estimator.cpp src/depth_estimator_nodelet.cpp)
target_link_libraries(depth_estimator_nodelet
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)
