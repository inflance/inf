#include "point_cloud.hpp"
#include "pcl_adapter.hpp"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

namespace inf::pc {

namespace {
core::Unexpected<core::Error> err(const char* msg) {
    return core::unexpected(core::Error{core::ErrorCode::InvalidArgument, msg});
}
}

core::Result<void> estimate_normals(PointCloud& cloud, const NormalEstimationParams& params) {
    if (cloud.empty()) return err("empty point cloud");

    auto pcl_cloud = detail::to_pcl_xyz(cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pcl_cloud);
    ne.setSearchMethod(pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());

    switch (params.method) {
        case NormalEstimationMethod::KNN: ne.setKSearch(params.k_neighbors); break;
        case NormalEstimationMethod::Radius: ne.setRadiusSearch(params.search_radius); break;
        case NormalEstimationMethod::Hybrid:
            ne.setKSearch(params.k_neighbors);
            ne.setRadiusSearch(params.search_radius);
            break;
    }

    if (params.viewpoint) {
        const auto& vp = *params.viewpoint;
        ne.setViewPoint(vp.x(), vp.y(), vp.z());
    }

    ne.compute(*normals);

    cloud.normals.resize(cloud.size());
    for (size_t i = 0; i < normals->size(); ++i) {
        const auto& n = normals->points[i];
        if (std::isfinite(n.normal_x) && std::isfinite(n.normal_y) && std::isfinite(n.normal_z)) {
            cloud.normals[i] = core::Vec3f(n.normal_x, n.normal_y, n.normal_z);
        } else {
            cloud.normals[i] = core::Vec3f(0, 0, 1);
        }
    }

    if (params.orient_normals && !params.viewpoint) {
        (void)orient_normals_consistently(cloud, params.k_neighbors);
    }
    return {};
}

void orient_normals_towards_viewpoint(PointCloud& cloud, const core::Vec3f& viewpoint) {
    if (!cloud.has_normals()) return;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (cloud.normals[i].dot(viewpoint - cloud.positions[i]) < 0) {
            cloud.normals[i] = -cloud.normals[i];
        }
    }
}

core::Result<void> orient_normals_consistently(PointCloud& cloud, int k_neighbors) {
    if (!cloud.has_normals()) return err("point cloud has no normals");
    if (cloud.size() < 3) return err("need at least 3 points");

    size_t top_idx = 0;
    float max_z = cloud.positions[0].z();
    for (size_t i = 1; i < cloud.size(); ++i) {
        if (cloud.positions[i].z() > max_z) {
            max_z = cloud.positions[i].z();
            top_idx = i;
        }
    }
    if (cloud.normals[top_idx].z() < 0) cloud.normals[top_idx] = -cloud.normals[top_idx];

    auto pcl_cloud = detail::to_pcl_xyz(cloud);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(pcl_cloud);

    std::vector<bool> visited(cloud.size(), false);
    std::vector<size_t> queue;
    queue.reserve(cloud.size());
    queue.push_back(top_idx);
    visited[top_idx] = true;

    std::vector<int> indices(k_neighbors);
    std::vector<float> distances(k_neighbors);

    while (!queue.empty()) {
        size_t cur = queue.back();
        queue.pop_back();
        tree.nearestKSearch(pcl_cloud->points[cur], k_neighbors, indices, distances);
        for (int idx : indices) {
            if (idx < 0 || static_cast<size_t>(idx) >= cloud.size() || visited[idx]) continue;
            if (cloud.normals[cur].dot(cloud.normals[idx]) < 0) cloud.normals[idx] = -cloud.normals[idx];
            visited[idx] = true;
            queue.push_back(idx);
        }
    }
    return {};
}

}  // namespace inf::pc
