//
// Created by kevin on 2020/5/7.
//

#include "bino/bino_toolbox.h"
#include "bino/config.h"




kbino::Bi2PC::Bi2PC(const std::string path) {
        // read from config file
        if (Config::SetParameterFile(path) == false) {
            std::cout<<"Config file error!"<<std::endl;
        }

        fx_ = kbino::Config::Get<double>("camera.fx");
        fy_ = kbino::Config::Get<double>("camera.fy");
        cx_ = kbino::Config::Get<double>("camera.cx");
        cy_ = kbino::Config::Get<double>("camera.cy");
        b_ =  kbino::Config::Get<double>("camera.d");
        zoom_ = kbino::Config::Get<double>("image.zoom");
        std::cout<<"Read config file success!"<<std::endl;
        std::cout<<"fx_ = "<<fx_<<std::endl;
        std::cout<<"fy_ = "<<fy_<<std::endl;
        std::cout<<"cx_ = "<<cx_<<std::endl;
        std::cout<<"cy_ = "<<cy_<<std::endl;
        std::cout<<"b_ = "<<b_<<std::endl;
        std::cout<<"zoom_ = "<<zoom_<<std::endl;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr kbino::Bi2PC::getPointCloud(){
    return cloud;
};
void kbino::Bi2PC::bi2pc(cv::Mat left, cv::Mat right) {

    if(zoom_ != 1){
        cv::resize(left,left,cv::Size(left.cols*zoom_,left.rows*zoom_));
        cv::resize(right,right,cv::Size(right.cols*zoom_,right.rows*zoom_));
    }
    left_ = left;
    right_ = right;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);    // 神奇的参数
    cv::Mat disparity_sgbm;
    sgbm->compute(left, right, disparity_sgbm);
    disparity_sgbm.convertTo(disparity_, CV_32F, 1.0 / 16.0f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr out (new pcl::PointCloud<pcl::PointXYZI>);
    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++) {
            if (disparity_.at<float>(v, u) <= 0.0 || disparity_.at<float>(v, u) >= 96.0) continue;
            double x = (u - cx_) / fx_;
        double y = (v - cy_) / fy_;
        double depth = fx_ * b_ / (disparity_.at<float>(v, u));
        pcl::PointXYZI point ;
        point.x = x * depth;;
        point.y = y * depth;
        point.z = depth;
        point.intensity = left.at<uchar>(v, u) / 255.0;
        out->push_back(point);
    }
    cloud = out;
}


void kbino::Bi2PC::showInPango(){
    // 生成点云
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud;

    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
    for (int v = 0; v < left_.rows; v++)
        for (int u = 0; u < left_.cols; u++) {
            if (disparity_.at<float>(v, u) <= 0.0 || disparity_.at<float>(v, u) >= 96.0) continue;

            Eigen::Vector4d point(0, 0, 0, left_.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色

            // 根据双目模型计算 point 的位置
            double x = (u - cx_) / fx_;
            double y = (v - cy_) / fy_;
            double depth = fx_ * b_ / (disparity_.at<float>(v, u));
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointcloud.push_back(point);
        }

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}



