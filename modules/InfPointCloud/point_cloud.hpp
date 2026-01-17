#pragma once
/// @file point_cloud.hpp
/// @brief InfPointCloud 内部实现

#if defined(_WIN32)
  #ifdef INFPOINTCLOUD_EXPORTS
    #define INFPOINTCLOUD_API __declspec(dllexport)
  #else
    #define INFPOINTCLOUD_API __declspec(dllimport)
  #endif
#else
  #define INFPOINTCLOUD_API __attribute__((visibility("default")))
#endif

#include "InfCore.hpp"
#include <vector>

namespace inf::pc {

// ============================================================================
// 点云数据结构
// ============================================================================
struct PointCloud {
    std::vector<core::Vec3f> positions;
    std::vector<core::Vec3f> normals;
    std::vector<core::Vec3f> colors;
};

// ============================================================================
// 滤波函数
// ============================================================================

/// @brief 统计离群点移除
[[nodiscard]] INFPOINTCLOUD_API PointCloud statistical_outlier_removal(
    const std::vector<core::Vec3f>& points,
    int k_neighbors = 20,
    double std_ratio = 2.0);

/// @brief 体素下采样
[[nodiscard]] INFPOINTCLOUD_API PointCloud voxel_downsample(
    const std::vector<core::Vec3f>& points,
    float voxel_size);

}  // namespace inf::pc
