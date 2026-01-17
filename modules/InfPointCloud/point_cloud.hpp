#pragma once

#if defined(_WIN32)
  #ifdef INFPOINTCLOUD_EXPORTS
    #define INFPOINTCLOUD_API __declspec(dllexport)
  #else
    #define INFPOINTCLOUD_API __declspec(dllimport)
  #endif
#else
  #define INFPOINTCLOUD_API __attribute__((visibility("default")))
#endif

#include "InfCore.hpp"
#include <vector>
#include <optional>
#include <cstdint>

namespace inf::pc {

struct PointCloud {
    std::vector<core::Vec3f> positions;
    std::vector<core::Vec3f> normals;
    std::vector<core::Vec3f> colors;
    std::vector<float> confidences;
    std::vector<float> scales;
    std::vector<uint32_t> source_images;
    std::vector<std::vector<uint32_t>> visible_views;

    [[nodiscard]] size_t size() const noexcept { return positions.size(); }
    [[nodiscard]] bool empty() const noexcept { return positions.empty(); }
    [[nodiscard]] bool has_normals() const noexcept { return normals.size() == positions.size(); }
    [[nodiscard]] bool has_colors() const noexcept { return colors.size() == positions.size(); }
    [[nodiscard]] bool has_confidences() const noexcept { return confidences.size() == positions.size(); }
    [[nodiscard]] bool has_scales() const noexcept { return scales.size() == positions.size(); }
    [[nodiscard]] bool has_source_images() const noexcept { return source_images.size() == positions.size(); }
    [[nodiscard]] bool has_visibility() const noexcept { return visible_views.size() == positions.size(); }
    [[nodiscard]] bool is_mvs_cloud() const noexcept { return has_confidences() || has_visibility(); }

    INFPOINTCLOUD_API void reserve(size_t n);
    INFPOINTCLOUD_API void clear() noexcept;
    INFPOINTCLOUD_API void shrink_to_fit();
    [[nodiscard]] INFPOINTCLOUD_API std::pair<core::Vec3f, core::Vec3f> bounding_box() const;
    INFPOINTCLOUD_API void append(PointCloud&& other);
    INFPOINTCLOUD_API void append(const PointCloud& other);
};

enum class NormalEstimationMethod : uint8_t { KNN, Radius, Hybrid };

struct NormalEstimationParams {
    NormalEstimationMethod method = NormalEstimationMethod::KNN;
    int k_neighbors = 30;
    float search_radius = 0.05f;
    bool orient_normals = true;
    std::optional<core::Vec3f> viewpoint;
};

struct PointCloudStats {
    size_t num_points = 0;
    core::Vec3f centroid = core::Vec3f::Zero();
    core::Vec3f bbox_min = core::Vec3f::Zero();
    core::Vec3f bbox_max = core::Vec3f::Zero();
    float avg_confidence = 0.0f;
    float avg_scale = 0.0f;
    float avg_visible_views = 0.0f;
    float point_density = 0.0f;
};

[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> voxel_downsample(const PointCloud& cloud, float voxel_size);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> random_downsample(const PointCloud& cloud, size_t target_count, uint32_t seed = 0);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> uniform_downsample(const PointCloud& cloud, float min_distance);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> grid_downsample(const PointCloud& cloud, size_t step);

[[nodiscard]] INFPOINTCLOUD_API core::Result<void> estimate_normals(PointCloud& cloud, const NormalEstimationParams& params = {});
INFPOINTCLOUD_API void orient_normals_towards_viewpoint(PointCloud& cloud, const core::Vec3f& viewpoint);
[[nodiscard]] INFPOINTCLOUD_API core::Result<void> orient_normals_consistently(PointCloud& cloud, int k_neighbors = 20);

[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> statistical_outlier_removal(const PointCloud& cloud, int k_neighbors = 20, double std_ratio = 2.0);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> radius_outlier_removal(const PointCloud& cloud, float search_radius, int min_neighbors = 2);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> passthrough_filter(const PointCloud& cloud, int axis, float min_val, float max_val, bool keep_organized = false);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> crop_box(const PointCloud& cloud, const core::Vec3f& min_bound, const core::Vec3f& max_bound);

[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> bilateral_smooth(const PointCloud& cloud, float sigma_s = 0.1f, float sigma_n = 0.1f);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> mls_smooth(const PointCloud& cloud, float search_radius, int polynomial_order = 2, bool compute_normals = true);

[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> filter_by_confidence(const PointCloud& cloud, float min_confidence);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> filter_by_visibility(const PointCloud& cloud, size_t min_views);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> from_sparse_model(const core::SparseModel& model, size_t min_track_length = 3);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> merge_clouds(const std::vector<PointCloud>& clouds, float merge_distance = 0.0f);
[[nodiscard]] INFPOINTCLOUD_API PointCloudStats compute_stats(const PointCloud& cloud);

[[nodiscard]] INFPOINTCLOUD_API core::Vec3f compute_centroid(const PointCloud& cloud);
[[nodiscard]] INFPOINTCLOUD_API PointCloud extract_by_indices(const PointCloud& cloud, const std::vector<size_t>& indices);
[[nodiscard]] INFPOINTCLOUD_API PointCloud remove_nan_points(const PointCloud& cloud);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> transform(const PointCloud& cloud, const core::Mat4f& transformation);

}  // namespace inf::pc
