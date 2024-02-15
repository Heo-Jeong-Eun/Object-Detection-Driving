cmake_minimum_required(VERSION 3.0.2)
project(yolov3_trt_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)
find_package(OpenCV REQUIRED PATHS ~/OpenCV4/install/lib/cmake/opencv4)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
)

# add_executable(yolov3_drive src/main.cpp)
# add_executable(dir_test src/test.cpp)

# target_link_libraries(yolov3_drive
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

# target_link_libraries(dir_test
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )