cmake_minimum_required(VERSION 2.8.3)
project(stereovision)

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


# if multiple opencv exists, set to customized folder
set(OpenCV_DIR /home/haichuan/opencv_build/opencv/build/installation/share/OpenCV)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost 1.65 COMPONENTS filesystem system REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(
    src
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

file(GLOB LIB_SRC SemiGlobalMatching/*.cpp ADCensusStereo/*.cpp ADCensusBM/*.cpp)
add_library(${PROJECT_NAME} SHARED ${LIB_SRC})


add_executable(test_adCensus examples/test_adCensus.cpp)
target_link_libraries(test_adCensus ${OpenCV_LIBS} ${PROJECT_NAME} pthread)

add_executable(test_sgm examples/test_sgm.cpp)
target_link_libraries(test_sgm ${PROJECT_NAME} ${OpenCV_LIBS} pthread)

add_executable(test_adCensusBM examples/test_adCensusBM.cpp)
target_link_libraries(test_adCensusBM ${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_DATE_TIME_LIBRARY}  ${YAML_CPP_LIBRARIES} pthread)