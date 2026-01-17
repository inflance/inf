#include "point_cloud.hpp"
#include "pcl_adapter.hpp"
#include <algorithm>
#include <unordered_map>

namespace inf::pc {

namespace {
core::Unexpected<core::Error> err(const char* msg) {
    return core::unexpected(core::Error{core::ErrorCode::InvalidArgument, msg});
}
}

core::Result<PointCloud> filter_by_confidence(const PointCloud& cloud, float min_confidence) {
    if (cloud.empty()) return err("empty point cloud");
    if (!cloud.has_confidences()) return PointCloud(cloud);

    std::vector<size_t> indices;
    indices.reserve(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (cloud.confidences[i] >= min_confidence) indices.push_back(i);
    }
    return extract_by_indices(cloud, indices);
}

core::Result<PointCloud> filter_by_visibility(const PointCloud& cloud, size_t min_views) {
    if (cloud.empty()) return err("empty point cloud");
    if (!cloud.has_visibility()) return PointCloud(cloud);

    std::vector<size_t> indices;
    indices.reserve(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (cloud.visible_views[i].size() >= min_views) indices.push_back(i);
    }
    return extract_by_indices(cloud, indices);
}

core::Result<PointCloud> from_sparse_model(const core::SparseModel& model, size_t min_track_length) {
    if (model.points3d.empty()) return err("empty sparse model");

    PointCloud result;
    size_t estimated = 0;
    for (const auto& [id, pt] : model.points3d) {
        if (pt.track.size() >= min_track_length) ++estimated;
    }

    result.positions.reserve(estimated);
    result.colors.reserve(estimated);
    result.visible_views.reserve(estimated);

    for (const auto& [id, pt] : model.points3d) {
        if (pt.track.size() < min_track_length) continue;
        result.positions.emplace_back(
            static_cast<float>(pt.position.x()),
            static_cast<float>(pt.position.y()),
            static_cast<float>(pt.position.z()));
        result.colors.push_back(pt.color);

        std::vector<uint32_t> views;
        views.reserve(pt.track.size());
        for (const auto& elem : pt.track) views.push_back(elem.image_id);
        result.visible_views.push_back(std::move(views));
    }

    if (result.empty()) return err("no points meet min_track_length");
    return result;
}

core::Result<PointCloud> merge_clouds(const std::vector<PointCloud>& clouds, float merge_distance) {
    if (clouds.empty()) return err("empty clouds list");
    if (clouds.size() == 1) return PointCloud(clouds[0]);

    size_t total = 0;
    for (const auto& c : clouds) total += c.size();

    if (merge_distance <= 0.0f) {
        PointCloud result;
        result.reserve(total);
        for (const auto& c : clouds) result.append(PointCloud(c));
        return result;
    }

    const float inv_voxel = 1.0f / merge_distance;

    struct MergeData {
        core::Vec3f pos_sum = core::Vec3f::Zero();
        core::Vec3f norm_sum = core::Vec3f::Zero();
        core::Vec3f color_sum = core::Vec3f::Zero();
        float conf_sum = 0.0f, scale_sum = 0.0f;
        size_t count = 0;
        std::vector<uint32_t> all_views;
    };

    std::unordered_map<detail::VoxelKey, MergeData, detail::VoxelKeyHash> voxel_map;
    voxel_map.reserve(total / 2);

    bool any_n = false, any_c = false, any_conf = false, any_sc = false, any_vis = false;
    for (const auto& c : clouds) {
        any_n |= c.has_normals();
        any_c |= c.has_colors();
        any_conf |= c.has_confidences();
        any_sc |= c.has_scales();
        any_vis |= c.has_visibility();
    }

    for (const auto& cloud : clouds) {
        for (size_t i = 0; i < cloud.size(); ++i) {
            auto key = detail::make_voxel_key(cloud.positions[i], inv_voxel);
            auto& d = voxel_map[key];
            d.pos_sum += cloud.positions[i];
            if (cloud.has_normals()) d.norm_sum += cloud.normals[i];
            if (cloud.has_colors()) d.color_sum += cloud.colors[i];
            if (cloud.has_confidences()) d.conf_sum += cloud.confidences[i];
            if (cloud.has_scales()) d.scale_sum += cloud.scales[i];
            if (cloud.has_visibility()) {
                d.all_views.insert(d.all_views.end(), cloud.visible_views[i].begin(), cloud.visible_views[i].end());
            }
            ++d.count;
        }
    }

    PointCloud result;
    result.positions.reserve(voxel_map.size());
    if (any_n) result.normals.reserve(voxel_map.size());
    if (any_c) result.colors.reserve(voxel_map.size());
    if (any_conf) result.confidences.reserve(voxel_map.size());
    if (any_sc) result.scales.reserve(voxel_map.size());
    if (any_vis) result.visible_views.reserve(voxel_map.size());

    for (auto& [key, d] : voxel_map) {
        float inv = 1.0f / static_cast<float>(d.count);
        result.positions.emplace_back(d.pos_sum * inv);
        if (any_n) {
            core::Vec3f n = d.norm_sum * inv;
            float len = n.norm();
            if (len > 1e-6f) n /= len;
            result.normals.push_back(n);
        }
        if (any_c) result.colors.emplace_back(d.color_sum * inv);
        if (any_conf) result.confidences.push_back(d.conf_sum * inv);
        if (any_sc) result.scales.push_back(d.scale_sum * inv);
        if (any_vis) {
            std::sort(d.all_views.begin(), d.all_views.end());
            d.all_views.erase(std::unique(d.all_views.begin(), d.all_views.end()), d.all_views.end());
            result.visible_views.push_back(std::move(d.all_views));
        }
    }
    return result;
}

PointCloudStats compute_stats(const PointCloud& cloud) {
    PointCloudStats stats;
    stats.num_points = cloud.size();
    if (cloud.empty()) return stats;

    core::Vec3f sum = core::Vec3f::Zero();
    stats.bbox_min = stats.bbox_max = cloud.positions[0];

    for (const auto& p : cloud.positions) {
        sum += p;
        stats.bbox_min = stats.bbox_min.cwiseMin(p);
        stats.bbox_max = stats.bbox_max.cwiseMax(p);
    }
    stats.centroid = sum / static_cast<float>(cloud.size());

    if (cloud.has_confidences()) {
        float s = 0.0f;
        for (float c : cloud.confidences) s += c;
        stats.avg_confidence = s / static_cast<float>(cloud.size());
    }
    if (cloud.has_scales()) {
        float s = 0.0f;
        for (float sc : cloud.scales) s += sc;
        stats.avg_scale = s / static_cast<float>(cloud.size());
    }
    if (cloud.has_visibility()) {
        size_t s = 0;
        for (const auto& v : cloud.visible_views) s += v.size();
        stats.avg_visible_views = static_cast<float>(s) / static_cast<float>(cloud.size());
    }

    core::Vec3f ext = stats.bbox_max - stats.bbox_min;
    float vol = ext.x() * ext.y() * ext.z();
    if (vol > 1e-9f) stats.point_density = static_cast<float>(cloud.size()) / vol;

    return stats;
}

}  // namespace inf::pc
