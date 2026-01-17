#pragma once

#include "point_cloud.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>
#include <cmath>

namespace inf::pc::detail {

using PclCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PclCloudNormal = pcl::PointCloud<pcl::PointNormal>;

inline PclCloudXYZ::Ptr to_pcl_xyz(const PointCloud& cloud) {
    auto pcl_cloud = pcl::make_shared<PclCloudXYZ>();
    pcl_cloud->points.reserve(cloud.size());
    pcl_cloud->width = static_cast<uint32_t>(cloud.size());
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;
    for (const auto& p : cloud.positions) {
        pcl_cloud->points.push_back({p.x(), p.y(), p.z()});
    }
    return pcl_cloud;
}

inline PclCloudNormal::Ptr to_pcl_normal(const PointCloud& cloud) {
    auto pcl_cloud = pcl::make_shared<PclCloudNormal>();
    pcl_cloud->points.reserve(cloud.size());
    pcl_cloud->width = static_cast<uint32_t>(cloud.size());
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;
    const bool has_n = cloud.has_normals();
    for (size_t i = 0; i < cloud.size(); ++i) {
        pcl::PointNormal pt;
        pt.x = cloud.positions[i].x();
        pt.y = cloud.positions[i].y();
        pt.z = cloud.positions[i].z();
        if (has_n) {
            pt.normal_x = cloud.normals[i].x();
            pt.normal_y = cloud.normals[i].y();
            pt.normal_z = cloud.normals[i].z();
        }
        pcl_cloud->points.push_back(pt);
    }
    return pcl_cloud;
}

inline PointCloud from_pcl_normal(const PclCloudNormal& pcl_cloud) {
    PointCloud result;
    result.positions.reserve(pcl_cloud.size());
    result.normals.reserve(pcl_cloud.size());
    for (const auto& pt : pcl_cloud.points) {
        result.positions.emplace_back(pt.x, pt.y, pt.z);
        result.normals.emplace_back(pt.normal_x, pt.normal_y, pt.normal_z);
    }
    return result;
}

struct VoxelKey {
    int32_t x, y, z;
    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        size_t h = 14695981039346656037ULL;
        h ^= static_cast<size_t>(k.x); h *= 1099511628211ULL;
        h ^= static_cast<size_t>(k.y); h *= 1099511628211ULL;
        h ^= static_cast<size_t>(k.z); h *= 1099511628211ULL;
        return h;
    }
};

inline VoxelKey make_voxel_key(const core::Vec3f& p, float inv_voxel) {
    return {
        static_cast<int32_t>(std::floor(p.x() * inv_voxel)),
        static_cast<int32_t>(std::floor(p.y() * inv_voxel)),
        static_cast<int32_t>(std::floor(p.z() * inv_voxel))
    };
}

}  // namespace inf::pc::detail
