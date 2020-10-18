cmake_minimum_required(VERSION 3.9)

project( stereo_estimation )

# default cmake build type is release
IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
ENDIF()


IF(${CMAKE_BUILD_TYPE} STREQUAL Debug)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g -std=c++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -O0 -g")
ENDIF()

IF(${CMAKE_BUILD_TYPE} STREQUAL Release)
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O2 -march=native -std=c++11")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O2 -march=native -std=c++11") # turn on O3 optimization, can choose O2 optimization
ENDIF()


###############
# packages #
###############

find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Pangolin REQUIRED)


find_package(OpenMP)

###############
# executables #
###############

add_executable( OpenCV_stereo src/main.cpp )
target_link_libraries( OpenCV_stereo ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

if(OpenMP_CXX_FOUND)
    target_link_libraries(OpenCV_stereo OpenMP::OpenMP_CXX)
endif()