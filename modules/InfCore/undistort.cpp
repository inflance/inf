/// @file undistort.cpp
#include "undistort.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace inf::core {

namespace {

/// 转换插值方式为 OpenCV 枚举
int to_cv_interp(Undistorter::Interpolation interp) {
    switch (interp) {
        case Undistorter::Interpolation::kNearest:  return cv::INTER_NEAREST;
        case Undistorter::Interpolation::kBilinear: return cv::INTER_LINEAR;
        case Undistorter::Interpolation::kBicubic:  return cv::INTER_CUBIC;
        case Undistorter::Interpolation::kLanczos4: return cv::INTER_LANCZOS4;
        default: return cv::INTER_LINEAR;
    }
}

/// 从 Camera 提取 OpenCV 格式的内参和畸变系数
/// @return {camera_matrix (3x3), dist_coeffs, is_fisheye}
std::tuple<cv::Mat, cv::Mat, bool> extract_cv_params(const Camera& cam) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist;
    bool is_fisheye = false;

    const auto& p = cam.params;

    auto set_intrinsic = [&](double fx, double fy, double cx, double cy) {
        K.at<double>(0, 0) = fx;
        K.at<double>(1, 1) = fy;
        K.at<double>(0, 2) = cx;
        K.at<double>(1, 2) = cy;
    };

    switch (cam.model) {
        case CameraModel::SIMPLE_PINHOLE:
            // f, cx, cy
            set_intrinsic(p[0], p[0], p[1], p[2]);
            dist = cv::Mat::zeros(4, 1, CV_64F);
            break;

        case CameraModel::PINHOLE:
            // fx, fy, cx, cy
            set_intrinsic(p[0], p[1], p[2], p[3]);
            dist = cv::Mat::zeros(4, 1, CV_64F);
            break;

        case CameraModel::SIMPLE_RADIAL:
            // f, cx, cy, k
            set_intrinsic(p[0], p[0], p[1], p[2]);
            dist = (cv::Mat_<double>(4, 1) << p[3], 0.0, 0.0, 0.0);
            break;

        case CameraModel::RADIAL:
            // f, cx, cy, k1, k2
            set_intrinsic(p[0], p[0], p[1], p[2]);
            dist = (cv::Mat_<double>(4, 1) << p[3], p[4], 0.0, 0.0);
            break;

        case CameraModel::OPENCV:
            // fx, fy, cx, cy, k1, k2, p1, p2
            set_intrinsic(p[0], p[1], p[2], p[3]);
            dist = (cv::Mat_<double>(4, 1) << p[4], p[5], p[6], p[7]);
            break;

        case CameraModel::OPENCV_FISHEYE:
            // fx, fy, cx, cy, k1, k2, k3, k4 (鱼眼模型)
            set_intrinsic(p[0], p[1], p[2], p[3]);
            dist = (cv::Mat_<double>(4, 1) << p[4], p[5], p[6], p[7]);
            is_fisheye = true;
            break;

        case CameraModel::FULL_OPENCV:
            // fx, fy, cx, cy, k1-k6, p1, p2 (12 params)
            set_intrinsic(p[0], p[1], p[2], p[3]);
            dist = (cv::Mat_<double>(8, 1) << p[4], p[5], p[10], p[11], p[6], p[7], p[8], p[9]);
            break;

        case CameraModel::SIMPLE_RADIAL_FISHEYE:
            // f, cx, cy, k (鱼眼)
            set_intrinsic(p[0], p[0], p[1], p[2]);
            dist = (cv::Mat_<double>(4, 1) << p[3], 0.0, 0.0, 0.0);
            is_fisheye = true;
            break;

        case CameraModel::RADIAL_FISHEYE:
            // f, cx, cy, k1, k2 (鱼眼)
            set_intrinsic(p[0], p[0], p[1], p[2]);
            dist = (cv::Mat_<double>(4, 1) << p[3], p[4], 0.0, 0.0);
            is_fisheye = true;
            break;

        default:
            // FOV / THIN_PRISM_FISHEYE 等暂不支持，按无畸变处理
            K.at<double>(0, 0) = p.size() > 0 ? p[0] : 1.0;
            K.at<double>(1, 1) = p.size() > 1 ? p[1] : K.at<double>(0, 0);
            K.at<double>(0, 2) = p.size() > 2 ? p[2] : cam.width / 2.0;
            K.at<double>(1, 2) = p.size() > 3 ? p[3] : cam.height / 2.0;
            dist = cv::Mat::zeros(4, 1, CV_64F);
            break;
    }

    return {K, dist, is_fisheye};
}

cv::Mat eigen_to_cv(const Mat3d& m) {
    cv::Mat out(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out.at<double>(i, j) = m(i, j);
    return out;
}

Mat3d cv_to_eigen(const cv::Mat& m) {
    Mat3d out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out(i, j) = m.at<double>(i, j);
    return out;
}

}  // namespace

// ============================================================================
// Undistorter 实现
// ============================================================================

Undistorter::Undistorter(const Camera& camera,
                         const Mat3d* new_camera_matrix,
                         cv::Size output_size) {
    build_maps(camera, new_camera_matrix, output_size);
}

void Undistorter::build_maps(const Camera& camera, const Mat3d* new_K, cv::Size out_size) {
    orig_width_ = camera.width;
    orig_height_ = camera.height;

    if (out_size.width == 0 || out_size.height == 0) {
        out_size = cv::Size(static_cast<int>(camera.width), static_cast<int>(camera.height));
    }

    auto [K_cv, dist, is_fisheye] = extract_cv_params(camera);
    const cv::Size img_size(static_cast<int>(camera.width), static_cast<int>(camera.height));

    // 计算新相机矩阵
    cv::Mat new_K_cv;
    if (new_K) {
        new_K_cv = eigen_to_cv(*new_K);
    } else if (is_fisheye) {
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            K_cv, dist, img_size, cv::Matx33d::eye(), new_K_cv, 0.0, out_size);
    } else {
        new_K_cv = cv::getOptimalNewCameraMatrix(K_cv, dist, img_size, 0.0, out_size);
    }
    new_K_ = cv_to_eigen(new_K_cv);

    // 构建 remap 表
    if (is_fisheye) {
        cv::fisheye::initUndistortRectifyMap(
            K_cv, dist, cv::Matx33d::eye(), new_K_cv, out_size, CV_32FC1, map_x_, map_y_);
    } else {
        cv::initUndistortRectifyMap(
            K_cv, dist, cv::Mat(), new_K_cv, out_size, CV_32FC1, map_x_, map_y_);
    }
}

void Undistorter::undistort(const cv::Mat& src, cv::Mat& dst, Interpolation interp) const {
    if (map_x_.empty()) {
        src.copyTo(dst);
        return;
    }
    cv::remap(src, dst, map_x_, map_y_, to_cv_interp(interp), cv::BORDER_CONSTANT);
}

cv::Mat Undistorter::undistort(const cv::Mat& src, Interpolation interp) const {
    cv::Mat dst;
    undistort(src, dst, interp);
    return dst;
}

Camera Undistorter::undistorted_camera() const {
    Camera cam;
    cam.model = CameraModel::PINHOLE;
    cam.width = map_x_.empty() ? orig_width_ : static_cast<uint32_t>(map_x_.cols);
    cam.height = map_x_.empty() ? orig_height_ : static_cast<uint32_t>(map_x_.rows);
    cam.params = {new_K_(0, 0), new_K_(1, 1), new_K_(0, 2), new_K_(1, 2)};
    return cam;
}

// ============================================================================
// 便捷函数
// ============================================================================

cv::Mat undistort_image(const cv::Mat& src, const Camera& camera) {
    Undistorter undistorter(camera);
    return undistorter.undistort(src);
}

Mat3d compute_optimal_new_intrinsic(const Camera& camera, double alpha) {
    auto [K_cv, dist, is_fisheye] = extract_cv_params(camera);
    const cv::Size img_size(static_cast<int>(camera.width), static_cast<int>(camera.height));

    cv::Mat new_K_cv;
    if (is_fisheye) {
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            K_cv, dist, img_size, cv::Matx33d::eye(), new_K_cv, alpha, img_size);
    } else {
        new_K_cv = cv::getOptimalNewCameraMatrix(K_cv, dist, img_size, alpha, img_size);
    }
    return cv_to_eigen(new_K_cv);
}

}  // namespace inf::core
