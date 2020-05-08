//
// Created by kevin on 2020/5/7.
//

#ifndef BINOCULAR_BINO_TOOLBOX_H
#define BINOCULAR_BINO_TOOLBOX_H
#include "bino/common_inc.h"
#include "config.h"
namespace kbino{

    class Bi2PC{
    private:
        cv::Mat disparity_;
        cv::Mat left_,right_;   // save the frame
        double fx_,fy_,cx_,cy_,zoom_;   // set intrinsics
        double b_ ;            // length of base line
        const std::string config_path_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud ;
        // 点云

    public:

        Bi2PC(const std::string path);
        ~Bi2PC(){};

        void bi2pc(cv::Mat left, cv::Mat right);
        // 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud();
        void showInPango();
        cv::Mat getDis(){
            return disparity_;
        }
    };

    class BiRosInterface{
        ros::NodeHandle nh_;                                 //实例化一个节点
        cv::Mat left_,right_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_left_;         //订阅节点
        image_transport::Subscriber image_sub_right_;        //订阅节点
        image_transport::Publisher image_pub_img_;           //发布节点
        ros::Publisher pub_pc;
        cv_bridge::CvImage cv_ptr_left, cv_ptr_right,cv_ptr_img;  //申明一个CvImagePtr
    public:
        BiRosInterface()
            : it_(nh_)
            {
                std::string left_toc = kbino::Config::Get<std::string>("topic_name.left");
                std::string right_toc = kbino::Config::Get<std::string>("topic_name.right");
                // Subscrive to input video feed and publish output video feed
                std::cout<<"topic_name.left: "<<left_toc<<std::endl;
                std::cout<<"topic_name.right: "<<right_toc<<std::endl;
                image_sub_left_ = it_.subscribe(left_toc, 1, &BiRosInterface::imageCb_left, this);
                image_sub_right_ = it_.subscribe(right_toc, 1, &BiRosInterface::imageCb_right, this);
                image_pub_img_ = it_.advertise("/bi2pc/camera/depth", 1);
                pub_pc = nh_.advertise<sensor_msgs::PointCloud2>("/bi2pc/camera/PointCloud", 10);
                cv_ptr_img.header.frame_id = "image";
                cv_ptr_img.encoding = "bgr8";
            }
            ~BiRosInterface(){};

        cv::Mat getLeft(){
            return left_;
        }
        cv::Mat getRight(){
            return right_;
        }

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
                left_ = cv_ptr->image;
                cv::imshow("?",left_);
                cv::waitKey(1);
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
            right_ = cv_ptr->image;

        }

//        void imagePublish(cv::Mat left, cv::Mat right){
//            ros::Time time=ros::Time::now();
//            cv_ptr_left.header.stamp = time;
//            cv_ptr_right.header.stamp = time;
//            cv_ptr_left.image = left;
//            cv_ptr_right.image = right;
//            image_pub_left_.publish(cv_ptr_left.toImageMsg());
//            image_pub_right_.publish(cv_ptr_right.toImageMsg());
//            ROS_INFO("Converted Successfully!");
//        }
        void imagePublish(cv::Mat img){
            img.convertTo(img,CV_8UC1);
            ros::Time time=ros::Time::now();
            cv_ptr_img.header.stamp = time;
            cv_ptr_img.image = img;
            cv_ptr_img.encoding = "mono8";
            image_pub_img_.publish(cv_ptr_img.toImageMsg());
//        ROS_INFO("Converted Successfully!");
        }

        void PcPublish(pcl::PointCloud<pcl::PointXYZI>::Ptr input){
            //创建ROS类型
            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*(input), cloud);
            ros::Time time=ros::Time::now();
            cloud.header.stamp = time;
            cloud.header.frame_id = "map";
            pub_pc.publish(cloud);
        }
    };


}


#endif //BINOCULAR_BINO_TOOLBOX_H
