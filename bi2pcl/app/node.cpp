//
// Created by kevin on 2020/5/7.
//
#include "bino/bino_toolbox.h"
#include "bino/config.h"

using namespace std;
using namespace Eigen;
clock_t start,en;
// 文件路径
std::string config_file = "/home/kevin/Documents/Github/kVisons/bi2pcl/config/default.yaml";


int main(int argc, char **argv) {

    ros::init(argc, argv, "bi2pc");
    kbino::Bi2PC b2p(config_file);
    kbino::BiRosInterface node;

    while(!ros::isShuttingDown()){
        // Start clock
        start = clock();

        cv::Mat left = node.getLeft();
        cv::Mat right = node.getRight();
        if(left.empty()||right.empty()){
            ros::spinOnce();
        }

        else{
            b2p.bi2pc(left,right);
//            node.PcPublish(b2p.getPointCloud());
            node.imagePublish(b2p.getDis());

            en= clock();   //结束时间
            cout<<"Freq = "<<CLOCKS_PER_SEC/(double)(en-start)<<"Hz"<<endl;
        }
        ros::spinOnce();
    }
    return 0;
}



