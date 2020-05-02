#include "myslam/dataset.h"
#include "myslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace myslam {

Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path) {}

bool Dataset::Init() {
    // read camera intrinsics and extrinsics
    ifstream fin(dataset_path_ + "/calib.txt");
    if (!fin) {
        LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
        return false;
    }
    distCoeffs_l = Mat::zeros(5, 1, CV_64F);
    distCoeffs_l.at<double>(0, 0) = -0.28340811;
    distCoeffs_l.at<double>(1, 0) = 0.07395907;
    distCoeffs_l.at<double>(2, 0) = 0.00019359;
    distCoeffs_l.at<double>(3, 0) =  1.76187114e-05;
    distCoeffs_l.at<double>(4, 0) = 0;

    distCoeffs_r = Mat::zeros(5, 1, CV_64F);
    distCoeffs_r.at<double>(0, 0) = -0.28368365;
    distCoeffs_r.at<double>(1, 0) = 0.07451284;
    distCoeffs_r.at<double>(2, 0) = -0.00010473;
    distCoeffs_r.at<double>(3, 0) =  -3.55590700e-05;
    distCoeffs_r.at<double>(4, 0) = 0;

    imageSize = cv::Size(752,480);
    for (int i = 0; i < 4; ++i) {
        char camera_name[3];
        for (int k = 0; k < 3; ++k) {
            fin >> camera_name[k];
        }
        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }
        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];

        KK = Mat::zeros(3, 3, CV_64F);
        KK.at<double>(0,0) = K(0,0);
        KK.at<double>(0,2) = K(0,2);
        KK.at<double>(1,1) = K(1,1);
        KK.at<double>(1,2) = K(1,2);
        KK.at<double>(2,2) = 1;
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        K = K * 0.5;
//        KK = KK * 0.5;
        // Init undistortion
        if(i == 1){
            initUndistortRectifyMap(KK, distCoeffs_l, Mat(),
                                    getOptimalNewCameraMatrix(KK, distCoeffs_l, imageSize, 1, imageSize, 0),
                                    imageSize, CV_16SC2, l_map1, l_map2);
        }
        else if (i == 2){
            initUndistortRectifyMap(KK, distCoeffs_r, Mat(),
                                    getOptimalNewCameraMatrix(KK, distCoeffs_l, imageSize, 1, imageSize, 0),
                                    imageSize, CV_16SC2, r_map1, r_map2);
        }
//        LOG(INFO)<<K;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), SE3(SO3(), t)));
        cameras_.push_back(new_camera);
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }
    fin.close();
    current_image_index_ = 0;
    return true;
}

Frame::Ptr Dataset::NextFrame() {
    boost::format fmt("%s/image_%d/%06d.png");
    cv::Mat image_left, image_right;
    // read images
    image_left =
        cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
    image_right =
        cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);

    if (image_left.data == nullptr || image_right.data == nullptr) {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }
//
//    remap(image_left, image_left, l_map1, l_map2, cv::INTER_LINEAR);
//    remap(image_right, image_right, r_map1, r_map2, cv::INTER_LINEAR);

    LOG(INFO)<<(fmt % dataset_path_ % 1 % current_image_index_).str();
    cv::Mat image_left_resized, image_right_resized;
    cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left_resized;
    new_frame->right_img_ = image_right_resized;
    current_image_index_++;
    return new_frame;
}

}  // namespace myslam
