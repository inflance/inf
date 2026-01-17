#pragma once
/// @file plane_detector.hpp
/// @brief 点云/网格平面检测器 (CGAL RANSAC / RegionGrowing)

#include "point_cloud.hpp"
#include <functional>

namespace inf::pc {

// ============================================================================
// 平面数据结构
// ============================================================================
struct Plane {
    core::Vec3f normal = core::Vec3f::UnitZ();
    float d = 0.0f;  // ax + by + cz + d = 0

    [[nodiscard]] float signed_distance(const core::Vec3f& p) const noexcept {
        return normal.dot(p) + d;
    }
    [[nodiscard]] float distance(const core::Vec3f& p) const noexcept {
        return std::abs(signed_distance(p));
    }
};

// ============================================================================
// 检测方法
// ============================================================================
enum class PlaneDetectionMethod : uint8_t {
    RANSAC,
    RegionGrowing
};

// ============================================================================
// 检测结果
// ============================================================================
constexpr int32_t kUnlabeledPatch = -1;

struct PlaneDetectionResult {
    std::vector<Plane> planes;
    std::vector<int32_t> point_labels;    // 每点的 patch 标签, -1 = 未分类
    std::vector<std::vector<size_t>> plane_point_indices;  // 每个平面包含的点索引

    [[nodiscard]] size_t num_planes() const noexcept { return planes.size(); }
    [[nodiscard]] size_t num_unlabeled() const noexcept {
        size_t count = 0;
        for (auto label : point_labels) {
            if (label == kUnlabeledPatch) ++count;
        }
        return count;
    }
};

/// @brief 进度回调函数类型
/// @param done 已完成的 leaf 数
/// @param total 总 leaf 数
/// @param partial 当前累积的检测结果（未合并相似平面）
using PlaneProgressCallback = std::function<void(
    size_t done, size_t total, const PlaneDetectionResult& partial)>;

// ============================================================================
// 检测参数
// ============================================================================
struct PlaneDetectionParams {
    PlaneDetectionMethod method = PlaneDetectionMethod::RANSAC;

    // RANSAC 参数
    float distance_threshold = 0.02f;     // 点到平面的最大距离阈值
    float normal_threshold = 0.8f;        // 法向量余弦阈值 (cos of max angle ~36 degrees)
    size_t min_points = 100;              // 平面最少点数
    float probability = 0.01f;            // RANSAC 最小概率 (越小，检测越完整)
    float cluster_epsilon_factor = 3.0f;  // cluster_epsilon = distance_threshold * factor

    // Octree + 多线程 (RANSAC/RegionGrowing)
    bool enable_octree = false;           // 对点云做八叉树分割
    size_t octree_max_points = 200000;    // 叶子最大点数
    int octree_max_depth = 8;             // 最大深度
    int max_threads = 0;                  // 0=auto

    // RegionGrowing 参数
    int k_neighbors = 30;                 // KNN 邻域大小
    float smoothness_threshold = 10.0f;   // 平滑度阈值 (度)
    float curvature_threshold = 1.0f;     // 曲率阈值

    // 进度回调（仅 Octree 模式有效）
    PlaneProgressCallback on_progress = nullptr;    // 每完成一个 leaf 调用一次
};

// ============================================================================
// 平面检测 API
// ============================================================================

/// @brief 从点云检测平面 patch
/// @param cloud 输入点云 (需要有法向量用于 RegionGrowing)
/// @param params 检测参数
/// @return 检测结果，包含平面列表和点的 patch 标签
[[nodiscard]] INFPOINTCLOUD_API core::Result<PlaneDetectionResult> detect_planes(
    const PointCloud& cloud,
    const PlaneDetectionParams& params = {});

/// @brief 根据检测结果提取指定平面的点云
/// @param cloud 原始点云
/// @param result 检测结果
/// @param plane_index 平面索引
/// @return 该平面的点云子集
[[nodiscard]] INFPOINTCLOUD_API PointCloud extract_plane_cloud(
    const PointCloud& cloud,
    const PlaneDetectionResult& result,
    size_t plane_index);

/// @brief 提取未分类点的点云
/// @param cloud 原始点云
/// @param result 检测结果
/// @return 未分类点的点云
[[nodiscard]] INFPOINTCLOUD_API PointCloud extract_unlabeled_cloud(
    const PointCloud& cloud,
    const PlaneDetectionResult& result);

/// @brief 根据平面标签为点云着色 (用于可视化)
/// @param cloud 点云
/// @param result 检测结果
/// @return 着色后的点云副本
[[nodiscard]] INFPOINTCLOUD_API PointCloud colorize_by_planes(
    const PointCloud& cloud,
    const PlaneDetectionResult& result);

}  // namespace inf::pc
