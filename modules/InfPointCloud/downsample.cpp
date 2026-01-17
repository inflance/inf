#include "point_cloud.hpp"
#include "pcl_adapter.hpp"
#include <algorithm>
#include <numeric>
#include <random>
#include <unordered_map>

namespace inf::pc {

namespace {
core::Unexpected<core::Error> err(const char* msg) {
    return core::unexpected(core::Error{core::ErrorCode::kInvalidArgument, msg});
}
}

core::Result<PointCloud> voxel_downsample(const PointCloud& cloud, float voxel_size) {
    if (cloud.empty()) return err("empty point cloud");
    if (voxel_size <= 0.0f) return err("voxel_size must be positive");

    const float inv_voxel = 1.0f / voxel_size;

    struct VoxelData {
        core::Vec3f position_sum = core::Vec3f::Zero();
        core::Vec3f normal_sum = core::Vec3f::Zero();
        core::Vec3f color_sum = core::Vec3f::Zero();
        float confidence_sum = 0.0f, scale_sum = 0.0f;
        size_t count = 0;
    };

    std::unordered_map<detail::VoxelKey, VoxelData, detail::VoxelKeyHash> voxel_map;
    voxel_map.reserve(cloud.size() / 8);

    const bool has_n = cloud.has_normals(), has_c = cloud.has_colors();
    const bool has_conf = cloud.has_confidences(), has_sc = cloud.has_scales();

    for (size_t i = 0; i < cloud.size(); ++i) {
        auto key = detail::make_voxel_key(cloud.positions[i], inv_voxel);
        auto& d = voxel_map[key];
        d.position_sum += cloud.positions[i];
        if (has_n) d.normal_sum += cloud.normals[i];
        if (has_c) d.color_sum += cloud.colors[i];
        if (has_conf) d.confidence_sum += cloud.confidences[i];
        if (has_sc) d.scale_sum += cloud.scales[i];
        ++d.count;
    }

    PointCloud result;
    result.positions.reserve(voxel_map.size());
    if (has_n) result.normals.reserve(voxel_map.size());
    if (has_c) result.colors.reserve(voxel_map.size());
    if (has_conf) result.confidences.reserve(voxel_map.size());
    if (has_sc) result.scales.reserve(voxel_map.size());

    for (const auto& [key, d] : voxel_map) {
        const float inv = 1.0f / static_cast<float>(d.count);
        result.positions.push_back(d.position_sum * inv);
        if (has_n) {
            core::Vec3f n = d.normal_sum * inv;
            float len = n.norm();
            if (len > 1e-6f) n /= len;
            result.normals.push_back(n);
        }
        if (has_c) result.colors.push_back(d.color_sum * inv);
        if (has_conf) result.confidences.push_back(d.confidence_sum * inv);
        if (has_sc) result.scales.push_back(d.scale_sum * inv);
    }
    return result;
}

core::Result<PointCloud> random_downsample(const PointCloud& cloud, size_t target_count, uint32_t seed) {
    if (cloud.empty()) return err("empty point cloud");
    if (target_count == 0) return err("target_count must be positive");
    if (target_count >= cloud.size()) return PointCloud(cloud);

    std::vector<size_t> indices(cloud.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::mt19937 gen(seed == 0 ? std::random_device{}() : seed);
    std::shuffle(indices.begin(), indices.end(), gen);
    indices.resize(target_count);
    std::sort(indices.begin(), indices.end());
    return extract_by_indices(cloud, indices);
}

core::Result<PointCloud> uniform_downsample(const PointCloud& cloud, float min_distance) {
    if (cloud.empty()) return err("empty point cloud");
    if (min_distance <= 0.0f) return err("min_distance must be positive");

    const float inv_voxel = 1.0f / min_distance;
    const float min_dist_sq = min_distance * min_distance;

    std::unordered_map<detail::VoxelKey, std::vector<size_t>, detail::VoxelKeyHash> voxel_map;
    voxel_map.reserve(cloud.size() / 4);

    for (size_t i = 0; i < cloud.size(); ++i) {
        voxel_map[detail::make_voxel_key(cloud.positions[i], inv_voxel)].push_back(i);
    }

    std::vector<bool> selected(cloud.size(), false);
    std::vector<size_t> result_indices;
    result_indices.reserve(cloud.size() / 8);

    for (size_t i = 0; i < cloud.size(); ++i) {
        if (selected[i]) continue;
        const auto& p = cloud.positions[i];
        auto key = detail::make_voxel_key(p, inv_voxel);

        bool too_close = false;
        for (int dx = -1; dx <= 1 && !too_close; ++dx) {
            for (int dy = -1; dy <= 1 && !too_close; ++dy) {
                for (int dz = -1; dz <= 1 && !too_close; ++dz) {
                    detail::VoxelKey nk{key.x + dx, key.y + dy, key.z + dz};
                    auto it = voxel_map.find(nk);
                    if (it == voxel_map.end()) continue;
                    for (size_t j : it->second) {
                        if (!selected[j] || j == i) continue;
                        if ((cloud.positions[j] - p).squaredNorm() < min_dist_sq) {
                            too_close = true;
                            break;
                        }
                    }
                }
            }
        }
        if (!too_close) {
            selected[i] = true;
            result_indices.push_back(i);
        }
    }
    return extract_by_indices(cloud, result_indices);
}

core::Result<PointCloud> grid_downsample(const PointCloud& cloud, size_t step) {
    if (cloud.empty()) return err("empty point cloud");
    if (step == 0) return err("step must be positive");
    if (step == 1) return PointCloud(cloud);

    std::vector<size_t> indices;
    indices.reserve((cloud.size() + step - 1) / step);
    for (size_t i = 0; i < cloud.size(); i += step) indices.push_back(i);
    return extract_by_indices(cloud, indices);
}

}  // namespace inf::pc
