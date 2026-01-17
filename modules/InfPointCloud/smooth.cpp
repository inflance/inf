#include "point_cloud.hpp"
#include "pcl_adapter.hpp"
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>

namespace inf::pc {

namespace {
core::Unexpected<core::Error> err(const char* msg) {
    return core::unexpected(core::Error{core::ErrorCode::kInvalidArgument, msg});
}
}

core::Result<PointCloud> bilateral_smooth(const PointCloud& cloud, float sigma_s, float sigma_n) {
    if (cloud.empty()) return err("empty point cloud");
    if (!cloud.has_normals()) return err("bilateral smooth requires normals");

    auto pcl_cloud = detail::to_pcl_xyz(cloud);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(pcl_cloud);

    PointCloud result;
    result.positions.reserve(cloud.size());
    result.normals = cloud.normals;
    if (cloud.has_colors()) result.colors = cloud.colors;

    const float sigma_s_sq = sigma_s * sigma_s;
    const float sigma_n_sq = sigma_n * sigma_n;
    const float search_radius = 3.0f * sigma_s;

    std::vector<int> indices;
    std::vector<float> distances;

    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud.positions[i];
        const auto& n = cloud.normals[i];
        tree.radiusSearch(pcl_cloud->points[i], search_radius, indices, distances);

        core::Vec3f weighted_pos = core::Vec3f::Zero();
        float weight_sum = 0.0f;

        for (size_t k = 0; k < indices.size(); ++k) {
            size_t j = indices[k];
            const auto& q = cloud.positions[j];
            const auto& nq = cloud.normals[j];

            float dist_sq = (p - q).squaredNorm();
            float ws = std::exp(-dist_sq / (2.0f * sigma_s_sq));
            float normal_diff = 1.0f - n.dot(nq);
            float wn = std::exp(-normal_diff * normal_diff / (2.0f * sigma_n_sq));
            float w = ws * wn;

            weighted_pos += w * q;
            weight_sum += w;
        }
        result.positions.push_back(weight_sum > 1e-6f ? weighted_pos / weight_sum : p);
    }
    return result;
}

core::Result<PointCloud> mls_smooth(const PointCloud& cloud, float search_radius, int polynomial_order, bool compute_normals) {
    if (cloud.empty()) return err("empty point cloud");
    if (search_radius <= 0.0f) return err("search_radius must be positive");

    auto pcl_cloud = detail::to_pcl_xyz(cloud);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(compute_normals);
    mls.setInputCloud(pcl_cloud);
    mls.setPolynomialOrder(polynomial_order);
    mls.setSearchMethod(pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
    mls.setSearchRadius(search_radius);

    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
    mls.process(*mls_points);

    PointCloud result = detail::from_pcl_normal(*mls_points);
    if (cloud.has_colors() && result.size() == cloud.size()) result.colors = cloud.colors;
    return result;
}

}  // namespace inf::pc
