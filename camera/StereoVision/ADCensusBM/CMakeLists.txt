project(ADCensusBM)
cmake_minimum_required(VERSION 2.8)

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


FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(OpenCV_DIR /home/haichuan/opencv_build/opencv/build/installation/share/OpenCV)

find_package(OpenCV 3 REQUIRED)
SET(LIBS ${LIBS} ${OpenCV_LIBS})


SET(Boost_ADDITIONAL_VERSIONS "1.65")
set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost 1.65.1 COMPONENTS filesystem system REQUIRED)
set(BOOST_FOUND ${Boost_FOUND})
set(BOOST_INCLUDE_DIRS "${Boost_INCLUDE_DIR}")
set(BOOST_LIBRARY_DIRS "${Boost_LIBRARY_DIRS}")
set(BOOST_LIBRARIES ${Boost_LIBRARIES})
INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIRS})
SET(LIBS ${LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_DATE_TIME_LIBRARY})

find_package(yaml-cpp REQUIRED)

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS "USE_PCL=1")

target_link_libraries (${PROJECT_NAME} ${LIBS} ${YAML_CPP_LIBRARIES} pthread)


