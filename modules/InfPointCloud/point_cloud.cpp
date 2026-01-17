#include "point_cloud.hpp"

namespace inf::pc {

void PointCloud::reserve(size_t n) {
    positions.reserve(n);
    if (!normals.empty()) normals.reserve(n);
    if (!colors.empty()) colors.reserve(n);
    if (!confidences.empty()) confidences.reserve(n);
    if (!scales.empty()) scales.reserve(n);
    if (!source_images.empty()) source_images.reserve(n);
    if (!visible_views.empty()) visible_views.reserve(n);
}

void PointCloud::clear() noexcept {
    positions.clear();
    normals.clear();
    colors.clear();
    confidences.clear();
    scales.clear();
    source_images.clear();
    visible_views.clear();
}

void PointCloud::shrink_to_fit() {
    positions.shrink_to_fit();
    normals.shrink_to_fit();
    colors.shrink_to_fit();
    confidences.shrink_to_fit();
    scales.shrink_to_fit();
    source_images.shrink_to_fit();
    visible_views.shrink_to_fit();
}

std::pair<core::Vec3f, core::Vec3f> PointCloud::bounding_box() const {
    if (empty()) return {core::Vec3f::Zero(), core::Vec3f::Zero()};
    core::Vec3f min_pt = positions[0], max_pt = positions[0];
    for (size_t i = 1; i < positions.size(); ++i) {
        min_pt = min_pt.cwiseMin(positions[i]);
        max_pt = max_pt.cwiseMax(positions[i]);
    }
    return {min_pt, max_pt};
}

void PointCloud::append(PointCloud&& other) {
    const size_t new_size = size() + other.size();
    positions.reserve(new_size);
    positions.insert(positions.end(), std::make_move_iterator(other.positions.begin()), std::make_move_iterator(other.positions.end()));

    if (has_normals() && other.has_normals()) {
        normals.reserve(new_size);
        normals.insert(normals.end(), std::make_move_iterator(other.normals.begin()), std::make_move_iterator(other.normals.end()));
    }
    if (has_colors() && other.has_colors()) {
        colors.reserve(new_size);
        colors.insert(colors.end(), std::make_move_iterator(other.colors.begin()), std::make_move_iterator(other.colors.end()));
    }
    if (has_confidences() && other.has_confidences()) {
        confidences.reserve(new_size);
        confidences.insert(confidences.end(), std::make_move_iterator(other.confidences.begin()), std::make_move_iterator(other.confidences.end()));
    }
    if (has_scales() && other.has_scales()) {
        scales.reserve(new_size);
        scales.insert(scales.end(), std::make_move_iterator(other.scales.begin()), std::make_move_iterator(other.scales.end()));
    }
    if (has_source_images() && other.has_source_images()) {
        source_images.reserve(new_size);
        source_images.insert(source_images.end(), std::make_move_iterator(other.source_images.begin()), std::make_move_iterator(other.source_images.end()));
    }
    if (has_visibility() && other.has_visibility()) {
        visible_views.reserve(new_size);
        visible_views.insert(visible_views.end(), std::make_move_iterator(other.visible_views.begin()), std::make_move_iterator(other.visible_views.end()));
    }
    other.clear();
}

void PointCloud::append(const PointCloud& other) {
    const size_t new_size = size() + other.size();
    positions.reserve(new_size);
    positions.insert(positions.end(), other.positions.begin(), other.positions.end());

    if (has_normals() && other.has_normals()) {
        normals.reserve(new_size);
        normals.insert(normals.end(), other.normals.begin(), other.normals.end());
    }
    if (has_colors() && other.has_colors()) {
        colors.reserve(new_size);
        colors.insert(colors.end(), other.colors.begin(), other.colors.end());
    }
    if (has_confidences() && other.has_confidences()) {
        confidences.reserve(new_size);
        confidences.insert(confidences.end(), other.confidences.begin(), other.confidences.end());
    }
    if (has_scales() && other.has_scales()) {
        scales.reserve(new_size);
        scales.insert(scales.end(), other.scales.begin(), other.scales.end());
    }
    if (has_source_images() && other.has_source_images()) {
        source_images.reserve(new_size);
        source_images.insert(source_images.end(), other.source_images.begin(), other.source_images.end());
    }
    if (has_visibility() && other.has_visibility()) {
        visible_views.reserve(new_size);
        visible_views.insert(visible_views.end(), other.visible_views.begin(), other.visible_views.end());
    }
}

}  // namespace inf::pc
