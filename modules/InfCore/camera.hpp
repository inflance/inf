#pragma once
/// @file camera.hpp
/// @brief 相机模型与 SFM 数据结构 (COLMAP 兼容)

#include "core.hpp"

namespace inf::core {

// ============================================================================
// 相机模型枚举 (COLMAP 兼容)
// ============================================================================
enum class CameraModel : uint8_t {
    SIMPLE_PINHOLE = 0,   // f, cx, cy (3 params)
    PINHOLE,              // fx, fy, cx, cy (4 params)
    SIMPLE_RADIAL,        // f, cx, cy, k (4 params)
    RADIAL,               // f, cx, cy, k1, k2 (5 params)
    OPENCV,               // fx, fy, cx, cy, k1, k2, p1, p2 (8 params)
    OPENCV_FISHEYE,       // fx, fy, cx, cy, k1, k2, k3, k4 (8 params)
    FULL_OPENCV,          // fx, fy, cx, cy, k1-k6, p1, p2 (12 params)
    FOV,                  // fx, fy, cx, cy, omega (5 params)
    SIMPLE_RADIAL_FISHEYE,// f, cx, cy, k (4 params)
    RADIAL_FISHEYE,       // f, cx, cy, k1, k2 (5 params)
    THIN_PRISM_FISHEYE,   // fx, fy, cx, cy, k1-k4, p1, p2, sx1, sy1 (12 params)
};

[[nodiscard]] constexpr uint32_t camera_model_num_params(CameraModel model) noexcept {
    constexpr uint32_t kParams[] = {3, 4, 4, 5, 8, 8, 12, 5, 4, 5, 12};
    const auto idx = static_cast<size_t>(model);
    return idx < std::size(kParams) ? kParams[idx] : 0;
}

[[nodiscard]] INFCORE_API CameraModel camera_model_from_string(std::string_view name) noexcept;
[[nodiscard]] INFCORE_API std::string_view camera_model_to_string(CameraModel model) noexcept;

// ============================================================================
// Camera 数据结构
// ============================================================================
struct INFCORE_API Camera {
    uint32_t            id = 0;
    CameraModel         model = CameraModel::PINHOLE;
    uint32_t            width = 0;
    uint32_t            height = 0;
    std::vector<double> params;

    /// 投影：相机坐标系 3D 点 -> 像素坐标
    [[nodiscard]] Vec2d project(const Vec3d& camera_point) const;

    /// 反投影：像素坐标 -> 归一化射线方向
    [[nodiscard]] Vec3d unproject(const Vec2d& pixel) const;

    /// 内参矩阵 K (有畸变时为近似值)
    [[nodiscard]] Mat3d intrinsic_matrix() const;
};

// ============================================================================
// SFM 数据结构
// ============================================================================
struct Point2D {
    Vec2d   xy;
    int64_t point3d_id = -1;  // -1 表示无对应 3D 点
};

struct Image {
    uint32_t             id = 0;
    Quatd                rotation;     // world-to-camera
    Vec3d                translation;  // world-to-camera
    uint32_t             camera_id = 0;
    std::string          name;
    std::vector<Point2D> points2d;

    [[nodiscard]] Vec3d camera_center() const { return -(rotation.inverse() * translation); }
    [[nodiscard]] Mat3d rotation_matrix() const { return rotation.toRotationMatrix(); }
};

struct TrackElement {
    uint32_t image_id = 0;
    uint32_t point2d_idx = 0;
};

struct Point3D {
    uint64_t                  id = 0;
    Vec3d                     position;
    Vec3f                     color;
    double                    error = 0.0;
    std::vector<TrackElement> track;
};

struct SparseModel {
    std::unordered_map<uint32_t, Camera>  cameras;
    std::unordered_map<uint32_t, Image>   images;
    std::unordered_map<uint64_t, Point3D> points3d;
};

}  // namespace inf::core
