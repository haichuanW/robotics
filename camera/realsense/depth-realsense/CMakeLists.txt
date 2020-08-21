cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesHelloRealSense)

add_executable(saveImage saveImage.cpp)

find_package(realsense2 2.36.0)
find_package(OpenCV 3.0 QUIET)
find_package(yaml-cpp REQUIRED)

target_link_libraries(saveImage ${DEPENDENCIES} ${realsense2_LIBRARY} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

