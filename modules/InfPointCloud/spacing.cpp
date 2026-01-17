#include "point_cloud.hpp"
#include "pcl_adapter.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace inf::pc {

namespace {
core::Unexpected<core::Error> err(const char* msg) {
    return core::unexpected(core::Error{core::ErrorCode::InvalidArgument, msg});
}

[[nodiscard]] std::vector<int> reservoir_sample_indices(
    const size_t n,
    const size_t sample_count,
    std::mt19937& gen) {

    const size_t m = (sample_count == 0 || sample_count >= n) ? n : sample_count;
    std::vector<int> sample;
    sample.reserve(m);

    // reservoir sampling: O(n) time, O(m) memory
    for (size_t i = 0; i < n; ++i) {
        if (i < m) {
            sample.push_back(static_cast<int>(i));
            continue;
        }
        std::uniform_int_distribution<size_t> dist(0, i);
        const size_t j = dist(gen);
        if (j < m) {
            sample[static_cast<size_t>(j)] = static_cast<int>(i);
        }
    }
    return sample;
}
}  // namespace

core::Result<float> compute_average_neighbor_distance(
    const PointCloud& cloud,
    const AverageNeighborDistanceParams& params) {

    if (cloud.empty()) return err("empty point cloud");
    if (cloud.size() < 2) return err("point cloud too small");
    if (params.k_neighbors < 1) return err("k_neighbors must be >= 1");

    const int k_query = std::min<int>(
        static_cast<int>(cloud.size()),
        params.k_neighbors + 1 /* +self */);
    if (k_query < 2) return err("k_neighbors too large for this cloud");

    // build KD tree
    auto pcl_cloud = detail::to_pcl_xyz(cloud);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pcl_cloud);

    std::mt19937 gen(params.seed == 0 ? std::random_device{}() : params.seed);
    const auto sample = reservoir_sample_indices(cloud.size(), params.sample_count, gen);

    std::vector<int> nn_indices(k_query);
    std::vector<float> nn_dist_sq(k_query);

    double sum = 0.0;
    size_t cnt = 0;

    for (const int idx : sample) {
        if (idx < 0) continue;
        const auto& q = pcl_cloud->points[static_cast<size_t>(idx)];
        const int found = kdtree.nearestKSearch(q, k_query, nn_indices, nn_dist_sq);
        if (found < 2) continue;

        const int use_k = std::min(params.k_neighbors, found - 1);
        for (int j = 0; j < use_k; ++j) {
            const float d2 = nn_dist_sq[static_cast<size_t>(j + 1)];  // skip self
            if (!std::isfinite(d2) || d2 < 0.0f) continue;
            sum += std::sqrt(static_cast<double>(d2));
            ++cnt;
        }
    }

    if (cnt == 0) return err("failed to compute neighbor distances");
    return static_cast<float>(sum / static_cast<double>(cnt));
}

}  // namespace inf::pc

