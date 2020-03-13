cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_compile_options(-march=armv7-a)

# ROS
find_package(catkin REQUIRED COMPONENTS
roscpp
cv_bridge
image_transport
message_generation
std_msgs
sensor_msgs
geometry_msgs
)
# OPENVINO
find_package(InferenceEngine 2.0)
if (NOT InferenceEngine_FOUND)
    message(FATAL_ERROR "")
endif()
# OPENCV
find_package(OpenCV REQUIRED)

#MSG DEPS
add_message_files(
  FILES
  BoxMax.msg
  Centroid.msg
  Deviation.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)


include_directories(
include
${catkin_INCLUDE_DIRS}
${InferenceEngine_INCLUDE_DIRS}
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES object_detection
CATKIN_DEPENDS roscpp message_runtime 
)

add_executable(object_detection_node 
src/object_detection_node.cpp
include/object_detection.h
include/common.hpp
include/ocv_common.hpp)


add_dependencies(object_detection_node 
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS})

#add_dependencies(object_detection tracking_drone )


target_link_libraries(object_detection_node ${catkin_LIBRARIES})
target_link_libraries(object_detection_node ${InferenceEngine_LIBRARIES})
target_link_libraries(object_detection_node ${OpenCV_LIBS})


