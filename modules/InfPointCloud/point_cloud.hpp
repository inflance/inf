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
    [[nodiscard]] bool has_normals() const noexcept { return !normals.empty() && normals.size() == positions.size(); }
    [[nodiscard]] bool has_colors() const noexcept { return !colors.empty() && colors.size() == positions.size(); }
    [[nodiscard]] bool has_confidences() const noexcept { return !confidences.empty() && confidences.size() == positions.size(); }
    [[nodiscard]] bool has_scales() const noexcept { return !scales.empty() && scales.size() == positions.size(); }
    [[nodiscard]] bool has_source_images() const noexcept { return !source_images.empty() && source_images.size() == positions.size(); }
    [[nodiscard]] bool has_visibility() const noexcept { return !visible_views.empty() && visible_views.size() == positions.size(); }
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
/// @brief 体素降采样到逼近目标点数（通过二分查找 voxel_size）
/// @param cloud 输入点云
/// @param target_count 目标点数（<= cloud.size()）
/// @param voxel_size_out 输出实际使用的 voxel_size（可为 nullptr）
/// @param rel_tol 相对误差容忍度（例如 0.05 表示允许 5% 偏差提前停止）
/// @param max_iters 二分查找最大迭代次数
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> voxel_downsample_to_target(
    const PointCloud& cloud,
    size_t target_count,
    float* voxel_size_out = nullptr,
    float rel_tol = 0.05f,
    int max_iters = 20);
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

struct AverageNeighborDistanceParams {
    int k_neighbors = 1;            // 平均最近的 k 个邻居（不包含自身），>=1
    size_t sample_count = 50000;    // 抽样点数（0 或 >= cloud.size() 表示全量）
    uint32_t seed = 1;              // 0=随机种子
};

/// @brief 计算点云的平均邻域距离（kNN，默认抽样）
/// @note 返回的是“平均最近邻距离”（单位与点云坐标一致），常用于估计点间距/设置 epsilon 等。
[[nodiscard]] INFPOINTCLOUD_API core::Result<float> compute_average_neighbor_distance(
    const PointCloud& cloud,
    const AverageNeighborDistanceParams& params = {});

[[nodiscard]] INFPOINTCLOUD_API core::Vec3f compute_centroid(const PointCloud& cloud);
[[nodiscard]] INFPOINTCLOUD_API PointCloud extract_by_indices(const PointCloud& cloud, const std::vector<size_t>& indices);
[[nodiscard]] INFPOINTCLOUD_API PointCloud remove_nan_points(const PointCloud& cloud);
[[nodiscard]] INFPOINTCLOUD_API core::Result<PointCloud> transform(const PointCloud& cloud, const core::Mat4f& transformation);

}  // namespace inf::pc
