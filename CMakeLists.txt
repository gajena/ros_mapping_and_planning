cmake_minimum_required(VERSION 2.8.3)
project(ros_mapping_and_planning)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)
catkin_package()

find_package (OpenCV REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(waypoint src/waypoint.cpp)
target_link_libraries(waypoint
    	${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
)

add_dependencies(waypoint  ${catkin_EXPORTED_TARGETS})

add_executable(depth_map src/depth_map.cpp)
target_link_libraries(depth_map
        ${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
)

add_dependencies(depth_map  ${catkin_EXPORTED_TARGETS})

