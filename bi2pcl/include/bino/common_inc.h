//
// Created by kevin on 2020/5/7.
//

#ifndef BI2PC_COMMON_INC_H
#define BI2PC_COMMON_INC_H
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
// STL
#include <vector>
#include <string>
#include <unistd.h>
#include<iostream>
#include<ctime>
#include <iostream>
// Eigen
#include <Eigen/Core>
// Pangolin
#include <pangolin/pangolin.h>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>      //图像处理
#include <opencv2/highgui/highgui.hpp>       //opencv GUI
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>              //cv_bridge
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <sensor_msgs/PointCloud2.h>


#endif //BI2PC_COMMON_INC_H
