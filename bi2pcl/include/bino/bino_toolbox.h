//
// Created by kevin on 2020/5/7.
//

#ifndef BINOCULAR_BINO_TOOLBOX_H
#define BINOCULAR_BINO_TOOLBOX_H
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include<iostream>
#include<ctime>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include<iostream>
#include<ctime>
#include <vector>


#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <ros/console.h>
namespace kbino{

    class Bi2PC{
    private:
        cv::Mat disparity_;
        cv::Mat left_,right_;   // save the frame
        double fx_,fy_,cx_,cy_;   // set intrinsics
        double b_ ;            // length of base line
        bool has_init =false;

    public:
        Bi2PC(){
            has_init=false;
        };
        ~Bi2PC(){};
        void bi2pc(cv::Mat left, cv::Mat right,pcl::PointCloud<pcl::PointXYZ>::Ptr output);
        void init(double fx, double fy, double cx, double cy, double b);
        void showInPango();
    };

    class BiRosInterface{
        ros::NodeHandle nh_;        //实例化一个节点
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_left_;     //订阅节点
        image_transport::Subscriber image_sub_right_;     //订阅节点
//        image_transport::Publisher image_pub_left_;      //发布节点
//        image_transport::Publisher image_pub_right_;      //发布节点
        cv_bridge::CvImage cv_ptr_left, cv_ptr_right;  //申明一个CvImagePtr
        cv::VideoCapture cap;
    public:
        BiRosInterface()
            : it_(nh_)
            {
                // Subscrive to input video feed and publish output video feed
                image_sub_ = it_.subscribe("/camera/left/image_raw", 1, &ImageConverter::imageCb_left, this);
                image_sub_ = it_.subscribe("/camera/left/image_raw", 1, &ImageConverter::imageCb_right, this);
//                image_pub_left_ = it_.advertise("/camera/left/image_raw", 1);
//                image_pub_right_ = it_.advertise("/camera/image_raw", 1);
//                cv_ptr_left.header.frame_id = "image";
//                cv_ptr_left.encoding = "mono8";
//                cv_ptr_right.header.frame_id = "image";
//                cv_ptr_right.encoding = "mono8";
//                cap.open(3);
            }

            ~BiRosInterface(){};

        void imageCb_left(const sensor_msgs::ImageConstPtr& msg)   //回调函数
            {
                cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }
        //转化为opencv的格式之后就可以对图像进行操作了
                // Draw an example circle on the video stream
                if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
                    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));  //画圆

                // Update GUI Window
                cv::imshow(OPENCV_WINDOW, cv_ptr->image);
                cv::waitKey(3);

                // Output modified video stream
                image_pub_.publish(cv_ptr->toImageMsg());
            }
        void imageCb_right(const sensor_msgs::ImageConstPtr& msg)   //回调函数
        {
            cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            //转化为opencv的格式之后就可以对图像进行操作了
            // Draw an example circle on the video stream
            if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
                cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));  //画圆

            // Update GUI Window
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::waitKey(3);

            // Output modified video stream
            image_pub_.publish(cv_ptr->toImageMsg());
        }

    };

    class ImageConverter    //申明一个图像转换的类
    {
        ros::NodeHandle nh_;        //实例化一个节点
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;     //订阅节点
        image_transport::Publisher image_pub_left_;      //发布节点
        image_transport::Publisher image_pub_right_;      //发布节点
        cv_bridge::CvImage cv_ptr_left, cv_ptr_right;  //申明一个CvImagePtr
        cv::VideoCapture cap;
    public:
        ImageConverter()
                : it_(nh_)
        {
            // Subscrive to input video feed and publish output video feed
//        image_sub_ = it_.subscribe("/rgb/image_raw", 1, &ImageConverter::imageCb, this);
            image_pub_left_ = it_.advertise("/camera/left/image_raw", 1);
            image_pub_right_ = it_.advertise("/camera/image_raw", 1);

//        cv_ptr_left.header.frame_id = "image";
//        cv_ptr_left.encoding = "bgr8";
//
//        cv_ptr_right.header.frame_id = "image";
//        cv_ptr_right.encoding = "bgr8";
            cv_ptr_left.header.frame_id = "image";
            cv_ptr_left.encoding = "mono8";

            cv_ptr_right.header.frame_id = "image";
            cv_ptr_right.encoding = "mono8";
            cap.open(3);
        }

        ~ImageConverter()
        {

        }
        void loadCam(){
            if(cap.isOpened())
            {
                cv::Mat imag1;
                cap>>imag1;

                cv::cvtColor(imag1,imag1,CV_BGR2GRAY);
                cv::resize(imag1,imag1,cv::Size(640,480));
                this->imagePublish(imag1,imag1);
                imshow("left",imag1);
                cv::waitKey(1);
            }
        }
        void loadImage(int i){
            cv::Mat img1,img2,temp,gray1,gray2;
            char left_img_[100],right_img_[100];
            std::string left_img = IMG_PATH,right_img=IMG_PATH;
            char windowname1[100],windowname2[100];
            left_img = IMG_PATH;
            sprintf(left_img_, "/left/tsukuba_fluorescent_L_%05d.png", i);
            sprintf(right_img_, "/right/tsukuba_fluorescent_R_%05d.png",i);
            left_img += left_img_;
            right_img += right_img_;
            img1 = cv::imread(left_img);
            img2 = cv::imread(right_img);
            assert(img1.data);
            assert(img2.data);

            cv::cvtColor(img1,img1,CV_BGR2GRAY);
            cv::cvtColor(img2,img2,CV_BGR2GRAY);
//        cv::resize(img1,img1,cv::Size(752,480));
//        cv::resize(img2,img2,cv::Size(752,480));
            imshow("left",img1);
            imshow("right",img2);
            this->imagePublish(img1,img2);
            cv::waitKey(1);
//        std::cout<<"Frame : "<<i<<std::endl;
        }
//    void imageCb(const sensor_msgs::ImageConstPtr& msg)   //回调函数
//    {
//        cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr
//        try
//        {
//            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//        }
//        catch (cv_bridge::Exception& e)
//        {
//            ROS_ERROR("cv_bridge exception: %s", e.what());
//            return;
//        }
////转化为opencv的格式之后就可以对图像进行操作了
//        // Draw an example circle on the video stream
//        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));  //画圆
//
//        // Update GUI Window
//        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//        cv::waitKey(3);
//
//        // Output modified video stream
//        image_pub_.publish(cv_ptr->toImageMsg());
//    }



        void imagePublish(cv::Mat left, cv::Mat right){
            ros::Time time=ros::Time::now();
            cv_ptr_left.header.stamp = time;
            cv_ptr_right.header.stamp = time;
            cv_ptr_left.image = left;
            cv_ptr_right.image = right;
            image_pub_left_.publish(cv_ptr_left.toImageMsg());
            image_pub_right_.publish(cv_ptr_right.toImageMsg());
//        ROS_INFO("Converted Successfully!");
        }
    };

}


#endif //BINOCULAR_BINO_TOOLBOX_H
