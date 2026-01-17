/// @file test_core.cpp
#include "InfCore.hpp"
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

TEST(InfCore, Version)
{
    auto ver = inf::core::version();
    EXPECT_FALSE(ver.empty());
    EXPECT_EQ(ver, "0.1.0");
}

TEST(InfCore, ErrorDefault) {
    inf::core::Error err;
    EXPECT_TRUE(err.ok());
    EXPECT_EQ(err.code, inf::core::ErrorCode::kSuccess);
}

// ============================================================================
// Undistorter 测试
// ============================================================================

TEST(Undistorter, PinholeNoDistortion) {
    // 针孔相机无畸变，去畸变后图像应基本不变
    inf::core::Camera cam;
    cam.model = inf::core::CameraModel::PINHOLE;
    cam.width = 640;
    cam.height = 480;
    cam.params = {500.0, 500.0, 320.0, 240.0};  // fx, fy, cx, cy

    inf::core::Undistorter undistorter(cam);
    EXPECT_TRUE(undistorter.valid());
    EXPECT_EQ(undistorter.output_size(), cv::Size(640, 480));

    // 去畸变后的相机应为 PINHOLE
    auto new_cam = undistorter.undistorted_camera();
    EXPECT_EQ(new_cam.model, inf::core::CameraModel::PINHOLE);
    EXPECT_EQ(new_cam.width, 640u);
    EXPECT_EQ(new_cam.height, 480u);
}

TEST(Undistorter, OpenCVModel) {
    // OpenCV 8 参数模型
    inf::core::Camera cam;
    cam.model = inf::core::CameraModel::OPENCV;
    cam.width = 1920;
    cam.height = 1080;
    cam.params = {1000.0, 1000.0, 960.0, 540.0,  // fx, fy, cx, cy
                  -0.1, 0.05, 0.001, -0.001};     // k1, k2, p1, p2

    inf::core::Undistorter undistorter(cam);
    EXPECT_TRUE(undistorter.valid());

    // 测试去畸变
    cv::Mat src = cv::Mat::zeros(1080, 1920, CV_8UC3);
    cv::circle(src, cv::Point(960, 540), 100, cv::Scalar(255, 255, 255), -1);

    cv::Mat dst = undistorter.undistort(src);
    EXPECT_EQ(dst.size(), cv::Size(1920, 1080));
    EXPECT_EQ(dst.type(), CV_8UC3);
}

TEST(Undistorter, RadialModel) {
    inf::core::Camera cam;
    cam.model = inf::core::CameraModel::RADIAL;
    cam.width = 800;
    cam.height = 600;
    cam.params = {600.0, 400.0, 300.0, -0.2, 0.1};  // f, cx, cy, k1, k2

    inf::core::Undistorter undistorter(cam);
    EXPECT_TRUE(undistorter.valid());

    auto new_cam = undistorter.undistorted_camera();
    EXPECT_EQ(new_cam.model, inf::core::CameraModel::PINHOLE);
}

TEST(Undistorter, FisheyeModel) {
    inf::core::Camera cam;
    cam.model = inf::core::CameraModel::OPENCV_FISHEYE;
    cam.width = 1280;
    cam.height = 720;
    cam.params = {400.0, 400.0, 640.0, 360.0,  // fx, fy, cx, cy
                  0.1, -0.05, 0.01, -0.005};    // k1, k2, k3, k4

    inf::core::Undistorter undistorter(cam);
    EXPECT_TRUE(undistorter.valid());

    cv::Mat src = cv::Mat::ones(720, 1280, CV_8UC1) * 128;
    cv::Mat dst = undistorter.undistort(src);
    EXPECT_FALSE(dst.empty());
}

TEST(Undistorter, ConvenienceFunction) {
    inf::core::Camera cam;
    cam.model = inf::core::CameraModel::SIMPLE_RADIAL;
    cam.width = 640;
    cam.height = 480;
    cam.params = {500.0, 320.0, 240.0, -0.1};  // f, cx, cy, k

    cv::Mat src = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::Mat dst = inf::core::undistort_image(src, cam);
    EXPECT_EQ(dst.size(), src.size());
}

TEST(Undistorter, OptimalNewIntrinsic) {
    inf::core::Camera cam;
    cam.model = inf::core::CameraModel::OPENCV;
    cam.width = 1920;
    cam.height = 1080;
    cam.params = {1000.0, 1000.0, 960.0, 540.0, -0.2, 0.1, 0.0, 0.0};

    auto K_alpha0 = inf::core::compute_optimal_new_intrinsic(cam, 0.0);
    auto K_alpha1 = inf::core::compute_optimal_new_intrinsic(cam, 1.0);

    // alpha=0 裁剪黑边，焦距通常更大
    // alpha=1 保留所有像素，焦距通常更小
    EXPECT_GT(K_alpha0(0, 0), 0);
    EXPECT_GT(K_alpha1(0, 0), 0);
}
