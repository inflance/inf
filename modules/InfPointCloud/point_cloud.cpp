/// @file point_cloud.cpp
#include "point_cloud.hpp"

namespace inf::pc {

PointCloud statistical_outlier_removal(
    std::span<const core::Vec3f> points, int k_neighbors, double std_ratio) {
    // TODO: 实现
    (void)points; (void)k_neighbors; (void)std_ratio;
    return {};
}

PointCloud voxel_downsample(std::span<const core::Vec3f> points, float voxel_size) {
    // TODO: 实现
    (void)points; (void)voxel_size;
    return {};
}

}  // namespace inf::pc
