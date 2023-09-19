cmake_minimum_required(VERSION 2.8)
project(copperTag)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(DEBUG)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# set compilation options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++17 -Wall -w ")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# find 3rd package
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  message("The OpenCV head file directory is: " ${OpenCV_INCLUDE_DIRS})
endif()
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
message("The Eigen3 head file directory is: " ${EIGEN3_INCLUDE_DIR})

# head file directory
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

file(GLOB_RECURSE PROJECT_SOURCES "src/*.cpp")

# copper_tag library
add_library(copperTagLib ${PROJECT_SOURCES})
target_link_libraries(copperTagLib ${OpenCV_LIBS} yaml-cpp)

# test program
add_executable(coppertag example/main.cpp)
target_link_libraries(coppertag copperTagLib)
