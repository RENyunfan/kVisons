cmake_minimum_required(VERSION 2.8)
project(lk_flow)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")


# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(demo src/demo.cpp)
target_link_libraries(demo
        ${OpenCV_LIBS}
)
