#include "point_cloud.hpp"
#include "pcl_adapter.hpp"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <limits>

namespace inf::pc {

namespace {
core::Unexpected<core::Error> err(const char* msg) {
    return core::unexpected(core::Error{core::ErrorCode::InvalidArgument, msg});
}
}

core::Result<PointCloud> statistical_outlier_removal(const PointCloud& cloud, int k_neighbors, double std_ratio) {
    if (cloud.empty()) return err("empty point cloud");
    if (k_neighbors <= 0) return err("k_neighbors must be positive");

    auto pcl_cloud = detail::to_pcl_xyz(cloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud);
    sor.setMeanK(k_neighbors);
    sor.setStddevMulThresh(std_ratio);

    pcl::Indices inliers;
    sor.filter(inliers);
    return extract_by_indices(cloud, std::vector<size_t>(inliers.begin(), inliers.end()));
}

core::Result<PointCloud> radius_outlier_removal(const PointCloud& cloud, float search_radius, int min_neighbors) {
    if (cloud.empty()) return err("empty point cloud");
    if (search_radius <= 0.0f) return err("search_radius must be positive");

    auto pcl_cloud = detail::to_pcl_xyz(cloud);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(pcl_cloud);
    ror.setRadiusSearch(search_radius);
    ror.setMinNeighborsInRadius(min_neighbors);

    pcl::Indices inliers;
    ror.filter(inliers);
    return extract_by_indices(cloud, std::vector<size_t>(inliers.begin(), inliers.end()));
}

core::Result<PointCloud> passthrough_filter(const PointCloud& cloud, int axis, float min_val, float max_val, bool keep_organized) {
    if (cloud.empty()) return err("empty point cloud");
    if (axis < 0 || axis > 2) return err("axis must be 0, 1, or 2");

    if (keep_organized) {
        PointCloud result = cloud;
        constexpr float nan_val = std::numeric_limits<float>::quiet_NaN();
        for (size_t i = 0; i < result.size(); ++i) {
            float val = result.positions[i][axis];
            if (val < min_val || val > max_val) {
                result.positions[i] = core::Vec3f(nan_val, nan_val, nan_val);
                if (result.has_normals()) result.normals[i] = core::Vec3f(nan_val, nan_val, nan_val);
            }
        }
        return result;
    }

    std::vector<size_t> indices;
    indices.reserve(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        float val = cloud.positions[i][axis];
        if (val >= min_val && val <= max_val) indices.push_back(i);
    }
    return extract_by_indices(cloud, indices);
}

core::Result<PointCloud> crop_box(const PointCloud& cloud, const core::Vec3f& min_bound, const core::Vec3f& max_bound) {
    if (cloud.empty()) return err("empty point cloud");

    std::vector<size_t> indices;
    indices.reserve(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud.positions[i];
        if (p.x() >= min_bound.x() && p.x() <= max_bound.x() &&
            p.y() >= min_bound.y() && p.y() <= max_bound.y() &&
            p.z() >= min_bound.z() && p.z() <= max_bound.z()) {
            indices.push_back(i);
        }
    }
    return extract_by_indices(cloud, indices);
}

}  // namespace inf::pc
