#include "point_cloud.hpp"
#include <cmath>

namespace inf::pc {

namespace {
core::Unexpected<core::Error> err(const char* msg) {
    return core::unexpected(core::Error{core::ErrorCode::kInvalidArgument, msg});
}
}

core::Vec3f compute_centroid(const PointCloud& cloud) {
    if (cloud.empty()) return core::Vec3f::Zero();
    core::Vec3f sum = core::Vec3f::Zero();
    for (const auto& p : cloud.positions) sum += p;
    return sum / static_cast<float>(cloud.size());
}

PointCloud extract_by_indices(const PointCloud& cloud, const std::vector<size_t>& indices) {
    PointCloud result;
    result.positions.reserve(indices.size());

    const bool has_n = cloud.has_normals(), has_c = cloud.has_colors();
    const bool has_conf = cloud.has_confidences(), has_sc = cloud.has_scales();
    const bool has_src = cloud.has_source_images(), has_vis = cloud.has_visibility();

    if (has_n) result.normals.reserve(indices.size());
    if (has_c) result.colors.reserve(indices.size());
    if (has_conf) result.confidences.reserve(indices.size());
    if (has_sc) result.scales.reserve(indices.size());
    if (has_src) result.source_images.reserve(indices.size());
    if (has_vis) result.visible_views.reserve(indices.size());

    for (size_t idx : indices) {
        if (idx >= cloud.size()) continue;
        result.positions.push_back(cloud.positions[idx]);
        if (has_n) result.normals.push_back(cloud.normals[idx]);
        if (has_c) result.colors.push_back(cloud.colors[idx]);
        if (has_conf) result.confidences.push_back(cloud.confidences[idx]);
        if (has_sc) result.scales.push_back(cloud.scales[idx]);
        if (has_src) result.source_images.push_back(cloud.source_images[idx]);
        if (has_vis) result.visible_views.push_back(cloud.visible_views[idx]);
    }
    return result;
}

PointCloud remove_nan_points(const PointCloud& cloud) {
    std::vector<size_t> valid;
    valid.reserve(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud.positions[i];
        if (std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z())) {
            valid.push_back(i);
        }
    }
    return extract_by_indices(cloud, valid);
}

core::Result<PointCloud> transform(const PointCloud& cloud, const core::Mat4f& transformation) {
    if (cloud.empty()) return err("empty point cloud");

    PointCloud result;
    result.positions.reserve(cloud.size());

    const core::Mat3f R = transformation.block<3, 3>(0, 0);
    const core::Vec3f t = transformation.block<3, 1>(0, 3);

    for (const auto& p : cloud.positions) {
        result.positions.push_back(R * p + t);
    }

    if (cloud.has_normals()) {
        result.normals.reserve(cloud.size());
        const core::Mat3f N = R.inverse().transpose();
        for (const auto& n : cloud.normals) {
            core::Vec3f tn = N * n;
            tn.normalize();
            result.normals.push_back(tn);
        }
    }

    if (cloud.has_colors()) result.colors = cloud.colors;
    if (cloud.has_confidences()) result.confidences = cloud.confidences;
    if (cloud.has_scales()) result.scales = cloud.scales;
    if (cloud.has_source_images()) result.source_images = cloud.source_images;
    if (cloud.has_visibility()) result.visible_views = cloud.visible_views;

    return result;
}

}  // namespace inf::pc
