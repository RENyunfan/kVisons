//
// Created by kevin on 2020/5/7.
//
#include "bino/bino_toolbox.h"
#include "bino/config.h"
#include <gflags/gflags.h>

using namespace std;
using namespace Eigen;
clock_t start,en;
// 文件路径
std::string config_file = "/home/kevin/Documents/Github/kVisons/bi2pcl/config/default.yaml";


int main(int argc, char **argv) {

    google::InitGoogleLogging(argv[0]);
    kbino::Bi2PC b2p(config_file);

    ros::init(argc, argv, "bi");
    kbino::BiRosInterface node;

    // 点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    // timer
    // 读取图像
    while(!ros::isShuttingDown()){
        start = clock();
        cv::Mat left = node.getLeft();
        cv::Mat right = node.getRight();
        if(left.empty()||right.empty()){
//            LOG(ERROR)<<"Cannot load image";
        }
        else{
//
            b2p.bi2pc(left,right,cloud);
            // 画出点云
//            node.imagePublish(b2p.getDis());
//            pcl::visualization::CloudViewer viewer("Cloud Viewer");
//            viewer.showCloud(cloud);
            en= clock();   //结束时间
            cout<<"Freq = "<<CLOCKS_PER_SEC/(double)(en-start)<<"Hz"<<endl;
        }

        ros::spinOnce();
    }

    return 0;
}



