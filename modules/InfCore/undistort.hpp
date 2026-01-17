#pragma once
/// @file undistort.hpp
/// @brief 图像去畸变类 (基于 OpenCV)

#include "camera.hpp"
#include <opencv2/core.hpp>

namespace inf::core {

/// 图像去畸变器
/// 预计算 remap 表，支持高效批量处理
class INFCORE_API Undistorter {
public:
    /// 插值方式
    enum class Interpolation {
        kNearest,   // 最近邻
        kBilinear,  // 双线性 (默认)
        kBicubic,   // 双三次
        kLanczos4,  // Lanczos 4x4
    };

    Undistorter() = default;

    /// 从 Camera 构建去畸变器
    /// @param camera 相机参数
    /// @param new_camera_matrix 可选新相机矩阵 (nullptr 则使用原始内参)
    /// @param output_size 可选输出尺寸 (0,0 则保持原尺寸)
    explicit Undistorter(const Camera& camera,
                         const Mat3d* new_camera_matrix = nullptr,
                         cv::Size output_size = {0, 0});

    /// 是否已初始化
    [[nodiscard]] bool valid() const noexcept { return !map_x_.empty(); }

    /// 去畸变单张图像
    /// @param src 输入畸变图像
    /// @param dst 输出去畸变图像 (可与 src 相同)
    /// @param interp 插值方式
    void undistort(const cv::Mat& src, cv::Mat& dst,
                   Interpolation interp = Interpolation::kBilinear) const;

    /// 去畸变单张图像 (返回新图像)
    [[nodiscard]] cv::Mat undistort(const cv::Mat& src,
                                    Interpolation interp = Interpolation::kBilinear) const;

    /// 获取输出尺寸
    [[nodiscard]] cv::Size output_size() const noexcept {
        return map_x_.empty() ? cv::Size{0, 0} : map_x_.size();
    }

    /// 获取新相机内参矩阵
    [[nodiscard]] const Mat3d& new_intrinsic() const noexcept { return new_K_; }

    /// 构建针孔模型的新 Camera (去畸变后)
    [[nodiscard]] Camera undistorted_camera() const;

private:
    void build_maps(const Camera& camera, const Mat3d* new_K, cv::Size out_size);

    cv::Mat map_x_;       // x 方向 remap 表
    cv::Mat map_y_;       // y 方向 remap 表
    Mat3d   new_K_ = Mat3d::Identity();
    uint32_t orig_width_ = 0;
    uint32_t orig_height_ = 0;
};

// ============================================================================
// 便捷函数
// ============================================================================

/// 单次去畸变 (不预计算 map，适合单张图像)
[[nodiscard]] INFCORE_API cv::Mat undistort_image(const cv::Mat& src, const Camera& camera);

/// 计算最优新相机矩阵 (类似 cv::getOptimalNewCameraMatrix)
/// @param camera 原相机
/// @param alpha 自由缩放参数 [0,1], 0=裁剪所有黑边, 1=保留所有像素
[[nodiscard]] INFCORE_API Mat3d compute_optimal_new_intrinsic(
    const Camera& camera, double alpha = 0.0);

}  // namespace inf::core
