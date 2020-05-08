//
// Created by kevin on 2020/5/7.
//
#include "bino/bino_toolbox.h"
using namespace std;
#define N 10000
#define CLK_TCK  CLOCKS_PER_SEC //机器时钟每秒的打点数
using namespace std;
using namespace Eigen;
clock_t Begin,End;
double duration;
// 文件路径
string left_file = "/home/kevin/Documents/Github/kVisons/Triangulation/data/left.png";
string right_file = "/home/kevin/Documents/Github/kVisons/Triangulation/data/right.png";

// 在pangolin中画图，已写好，无需调整
void showPointCloud(
        const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

//    [458.654, 457.296, 367.215, 248.375
    // 内参
    double fx = 615, fy = 615, cx = 0, cy = 0
            ;
    // 基线
    double b = 1;
    // 点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // timer
    Begin = clock();
    // 读取图像
    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
//    cv::resize(left,left,cv::Size(left.cols*0.5,left.rows*0.5));
//    cv::resize(right,right,cv::Size(right.cols*0.5,right.rows*0.5));

    kbino::Bi2PC b2p;
    b2p.init(615, 615, 0, 0,1);
    b2p.bi2pc(left,right,cloud);
    b2p.showInPango();
    End = clock();



    duration =(double) (End-Begin)/CLK_TCK;
    cout<<"tick:"<<End-Begin<<endl;
    cout<<CLK_TCK<<endl;
    cout<<"duration:"<<duration<<endl;
    // 画出点云
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while(1);
    return 0;
}

