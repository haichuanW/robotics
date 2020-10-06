cmake_minimum_required(VERSION 2.8.3)
project(PMStereo)

# default cmake build type is release
IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
ENDIF()


IF(${CMAKE_BUILD_TYPE} STREQUAL Debug)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g -std=c++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -O0 -g")
ENDIF()

IF(${CMAKE_BUILD_TYPE} STREQUAL Release)
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -std=c++11")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -std=c++11") # turn on O3 optimization, can choose O2 optimization
ENDIF()

# if multiple opencv exists, set to customized folder
set(OpenCV_DIR /home/haichuan/opencv_build/opencv/build/installation/share/OpenCV)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    src
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)


find_package(Pangolin REQUIRED)

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})