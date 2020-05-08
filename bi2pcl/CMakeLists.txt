cmake_minimum_required(VERSION 2.8.3)
project(bi2pc)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_compile_options(-std=c++11)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)

# ros
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        pcl_ros
        roscpp
        sensor_msgs
        nav_msgs
        sensor_msgs
        cv_bridge
        image_transport
        geometry_msgs
        )

# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(
        include ${catkin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${PCL_LIBRARIES}
)

catkin_package(
)

set(THIRD_PARTY_LIBS
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${PCL_LIBRARIES}
)

add_executable(bino
        app/node.cpp
        include/bino/bino_toolbox.h
        src/bino_toolbox.cpp
        include/bino/config.h
        src/config.cpp)

add_dependencies(bino ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bino
        ${THIRD_PARTY_LIBS}
        )
