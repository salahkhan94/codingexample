cmake_minimum_required(VERSION 2.8)
project(track_and_chase)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED eigen_conversions cv_bridge image_geometry image_transport nodelet nodelet_topic_tools roscpp sensor_msgs geometry_msgs)
find_package(OpenCV REQUIRED)
find_package(Eigen3)
find_package(Boost REQUIRED COMPONENTS thread)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()


catkin_package(
  CATKIN_DEPENDS image_geometry roscpp sensor_msgs geometry_msgs
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED  src/nodelets/blob_detector.cpp
                                    src/nodes/kalman_filter.cpp
                                    src/nodelets/pose_estimator.cpp
                                    src/nodelets/display_output.cpp
                                    src/nodes/chase_robot.cpp
                                    src/nodes/chase_planner.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})

# Standalone node
add_executable(track_and_chase_node src/nodes/track_and_chase.cpp)
target_link_libraries(track_and_chase_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(chase_robot_node src/nodes/chase_robot_node.cpp)
target_link_libraries(chase_robot_node ${PROJECT_NAME} ${catkin_LIBRARIES})

