cmake_minimum_required(VERSION 2.8)

project( HelloWorld_Proj )


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



find_package ( OpenCV 3 REQUIRED )

file(GLOB SOURCES src/*.h src/*.cpp)

add_executable (filter ${SOURCES})
target_link_libraries ( filter ${OpenCV_LIBS} )