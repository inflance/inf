#include "plane_detector.hpp"
#include "../InfIO/macro.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Point_set.h>
#include <CGAL/Orthtree.h>
#include <CGAL/Orthtree_traits_point.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <numeric>
#include <unordered_map>
#include <utility>

namespace inf::pc {

namespace {

// CGAL 类型定义
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using FT = Kernel::FT;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Plane_3 = Kernel::Plane_3;
using PointWithNormal = std::pair<Point_3, Vector_3>;
using PointList = std::vector<PointWithNormal>;
using PointMap = CGAL::First_of_pair_property_map<PointWithNormal>;
using NormalMap = CGAL::Second_of_pair_property_map<PointWithNormal>;

// RANSAC 类型
using RansacTraits = CGAL::Shape_detection::Efficient_RANSAC_traits<
    Kernel, PointList, PointMap, NormalMap>;
using EfficientRansac = CGAL::Shape_detection::Efficient_RANSAC<RansacTraits>;
using CgalPlane = CGAL::Shape_detection::Plane<RansacTraits>;

// Region Growing 需要能处理 iterator 的 property map
// 创建间接 property map：iterator -> dereferenced value -> point/normal
using PointIterator = PointList::const_iterator;

// 间接 property map: 解引用 iterator 再取第一个元素
struct IteratorPointMap {
    using key_type = PointIterator;
    using value_type = Point_3;
    using reference = const Point_3&;
    using category = boost::readable_property_map_tag;
    
    friend reference get(const IteratorPointMap&, key_type it) {
        return it->first;
    }
};

// 间接 property map: 解引用 iterator 再取第二个元素
struct IteratorNormalMap {
    using key_type = PointIterator;
    using value_type = Vector_3;
    using reference = const Vector_3&;
    using category = boost::readable_property_map_tag;
    
    friend reference get(const IteratorNormalMap&, key_type it) {
        return it->second;
    }
};

// Region Growing 类型 (使用正确的 property map)
using RgNeighborQuery = CGAL::Shape_detection::Point_set::K_neighbor_query<
    Kernel, PointIterator, IteratorPointMap>;
using RgRegionType = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<
    Kernel, PointIterator, IteratorPointMap, IteratorNormalMap>;
using RegionGrowing = CGAL::Shape_detection::Region_growing<RgNeighborQuery, RgRegionType>;

core::Unexpected<core::Error> make_error(const char* msg) {
    return core::unexpected(core::Error{core::ErrorCode::InvalidArgument, msg});
}

// 转换点云到 CGAL 格式
PointList to_cgal_points(const PointCloud& cloud) {
    PointList points;
    points.reserve(cloud.size());
    const bool has_normals = cloud.has_normals();
    size_t invalid_normals = 0;
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud.positions[i];
        Vector_3 n(0, 0, 1);
        if (has_normals) {
            const auto& normal = cloud.normals[i];
            const double nx = static_cast<double>(normal.x());
            const double ny = static_cast<double>(normal.y());
            const double nz = static_cast<double>(normal.z());
            const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (std::isfinite(len) && len > 1e-12) {
                n = Vector_3(nx / len, ny / len, nz / len);
            } else {
                ++invalid_normals;
            }
        }
        points.emplace_back(Point_3(p.x(), p.y(), p.z()), n);
    }
    if (has_normals && invalid_normals > 0) {
        INF_WARN("to_cgal_points: {} / {} invalid normals replaced with (0,0,1)",
                 invalid_normals, cloud.size());
    }
    return points;
}

// CGAL Plane_3 转换为 Plane 结构
Plane cgal_plane_to_plane(const Plane_3& cgal_plane) {
    Plane plane;
    double a = CGAL::to_double(cgal_plane.a());
    double b = CGAL::to_double(cgal_plane.b());
    double c = CGAL::to_double(cgal_plane.c());
    double d = CGAL::to_double(cgal_plane.d());
    double len = std::sqrt(a * a + b * b + c * c);
    if (len > 1e-9) {
        plane.normal = core::Vec3f(
            static_cast<float>(a / len),
            static_cast<float>(b / len),
            static_cast<float>(c / len));
        plane.d = static_cast<float>(d / len);
    }
    return plane;
}

core::Result<PlaneDetectionResult> detect_planes_ransac(
    const PointCloud& cloud,
    const PlaneDetectionParams& params);
core::Result<PlaneDetectionResult> detect_planes_region_growing(
    const PointCloud& cloud,
    const PlaneDetectionParams& params);

struct IndexPointMap {
    using key_type = size_t;
    using value_type = Point_3;
    using reference = const Point_3&;
    using category = boost::readable_property_map_tag;
    const std::vector<Point_3>* points = nullptr;

    friend reference get(const IndexPointMap& map, key_type idx) {
        return (*map.points)[idx];
    }
};

using IndexList = std::vector<size_t>;
using OctreeTraits = CGAL::Orthtree_traits_point<Kernel, IndexList, IndexPointMap, false, 3>;
using CgalOctree = CGAL::Orthtree<OctreeTraits>;

struct LocalDetectionResult {
    PlaneDetectionResult result;
    std::vector<size_t> indices;
};

// 判断两个平面是否可以合并（法向相近、距离相近）
bool can_merge_planes(const Plane& a, const Plane& b,
                      float normal_thresh = 0.98f,   // cos(~11°)
                      float dist_thresh = 0.05f) {
    const float dot = std::abs(a.normal.dot(b.normal));
    if (dot < normal_thresh) return false;
    // 平面距离差异（两个平面原点到对方的距离）
    const float d_diff = std::abs(a.d - b.d);
    return d_diff < dist_thresh;
}

// 合并相似平面，返回合并后的结果
PlaneDetectionResult merge_similar_planes(
    PlaneDetectionResult& input,
    float normal_thresh = 0.98f,
    float dist_thresh = 0.05f) {

    if (input.planes.empty()) return std::move(input);

    const size_t n_planes = input.planes.size();
    std::vector<int> parent(n_planes);
    std::iota(parent.begin(), parent.end(), 0);

    // Union-Find with path compression
    std::function<int(int)> find = [&](int x) {
        return parent[x] == x ? x : (parent[x] = find(parent[x]));
    };
    auto unite = [&](int a, int b) {
        a = find(a); b = find(b);
        if (a != b) parent[b] = a;
    };

    // 合并相似平面
    for (size_t i = 0; i < n_planes; ++i) {
        for (size_t j = i + 1; j < n_planes; ++j) {
            if (can_merge_planes(input.planes[i], input.planes[j],
                                 normal_thresh, dist_thresh)) {
                unite(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }

    // 收集合并组
    std::unordered_map<int, std::vector<size_t>> groups;
    for (size_t i = 0; i < n_planes; ++i) {
        groups[find(static_cast<int>(i))].push_back(i);
    }

    PlaneDetectionResult merged;
    merged.point_labels.resize(input.point_labels.size(), kUnlabeledPatch);
    merged.planes.reserve(groups.size());
    merged.plane_point_indices.reserve(groups.size());

    for (auto& [root, members] : groups) {
        // 合并所有点
        std::vector<size_t> all_indices;
        for (size_t m : members) {
            all_indices.insert(all_indices.end(),
                               input.plane_point_indices[m].begin(),
                               input.plane_point_indices[m].end());
        }
        if (all_indices.empty()) continue;

        // 用点数最多的平面作为代表（或者可以重新拟合）
        size_t best = members[0];
        for (size_t m : members) {
            if (input.plane_point_indices[m].size() >
                input.plane_point_indices[best].size()) {
                best = m;
            }
        }
        Plane merged_plane = input.planes[best];

        const int32_t new_label = static_cast<int32_t>(merged.planes.size());
        for (size_t idx : all_indices) {
            merged.point_labels[idx] = new_label;
        }
        merged.planes.push_back(merged_plane);
        merged.plane_point_indices.push_back(std::move(all_indices));
    }

    INF_INFO("Plane merge: {} -> {} planes", n_planes, merged.planes.size());
    return merged;
}

LocalDetectionResult detect_planes_ransac_local(
    const PointCloud& cloud,
    const PlaneDetectionParams& params,
    const std::vector<size_t>& indices) {

    LocalDetectionResult local;
    local.indices = indices;
    if (indices.size() < params.min_points) {
        return local;
    }
    PointCloud sub = extract_by_indices(cloud, indices);
    auto sub_result = detect_planes_ransac(sub, params);
    if (sub_result.has_value()) {
        local.result = std::move(sub_result.value());
    }
    return local;
}

LocalDetectionResult detect_planes_region_growing_local(
    const PointCloud& cloud,
    const PlaneDetectionParams& params,
    const std::vector<size_t>& indices) {

    LocalDetectionResult local;
    local.indices = indices;
    if (indices.size() < params.min_points) {
        return local;
    }
    PointCloud sub = extract_by_indices(cloud, indices);
    auto sub_result = detect_planes_region_growing(sub, params);
    if (sub_result.has_value()) {
        local.result = std::move(sub_result.value());
    }
    return local;
}

core::Result<PlaneDetectionResult> detect_planes_ransac_octree(
    const PointCloud& cloud,
    const PlaneDetectionParams& params) {

    INF_INFO("Octree RANSAC start: {} points, max_points={}, max_depth={}",
             cloud.size(), params.octree_max_points, params.octree_max_depth);

    // 构建 Point_3 用于 Octree（必须，因为 CGAL Octree 需要 Point_3）
    std::vector<Point_3> points;
    points.reserve(cloud.size());
    for (const auto& p : cloud.positions) {
        points.emplace_back(p.x(), p.y(), p.z());
    }

    IndexList indices(cloud.size());
    std::iota(indices.begin(), indices.end(), 0);

    IndexPointMap point_map;
    point_map.points = &points;
    CgalOctree octree(indices, point_map);
    INF_INFO("Octree RANSAC: building octree");
    octree.refine(CGAL::Orthtrees::Maximum_depth_and_maximum_contained_elements(
        params.octree_max_depth, params.octree_max_points));

    // 收集叶子节点的索引范围（只存 begin/end 迭代器位置，不复制）
    using LeafRange = std::pair<size_t, size_t>;  // [begin, end) in indices
    std::vector<std::vector<size_t>> leaves;
    leaves.reserve(256);
    for (auto node : octree.traverse<CGAL::Orthtrees::Leaves_traversal<CgalOctree>>()) {
        const auto& data = octree.data(node);
        if (data.begin() != data.end()) {
            std::vector<size_t> leaf_indices(data.begin(), data.end());
            leaves.push_back(std::move(leaf_indices));
        }
    }
    INF_INFO("Octree RANSAC: {} leaves", leaves.size());

    PlaneDetectionResult merged;
    merged.point_labels.resize(cloud.size(), kUnlabeledPatch);
    if (leaves.empty()) {
        INF_INFO("Octree RANSAC detected 0 planes, {} unlabeled points, 0 leaves",
                 merged.num_unlabeled());
        return merged;
    }

    // 合并单个 leaf 结果到 merged 的 lambda
    auto merge_local = [&merged](const LocalDetectionResult& local) {
        const size_t base = merged.planes.size();
        merged.planes.insert(
            merged.planes.end(),
            local.result.planes.begin(),
            local.result.planes.end());
        merged.plane_point_indices.resize(merged.planes.size());

        for (size_t p = 0; p < local.result.plane_point_indices.size(); ++p) {
            const size_t global_plane = base + p;
            auto& global_indices = merged.plane_point_indices[global_plane];
            const auto& local_indices = local.result.plane_point_indices[p];
            global_indices.reserve(local_indices.size());
            for (size_t local_idx : local_indices) {
                if (local_idx >= local.indices.size()) continue;
                const size_t global_idx = local.indices[local_idx];
                merged.point_labels[global_idx] = static_cast<int32_t>(global_plane);
                global_indices.push_back(global_idx);
            }
        }
    };

    const size_t total_leaves = leaves.size();

    // 如果设置了回调，顺序处理并在每个 leaf 后回调
    if (params.on_progress) {
        INF_INFO("Octree RANSAC: sequential mode (callback enabled)");
        for (size_t i = 0; i < leaves.size(); ++i) {
            auto local = detect_planes_ransac_local(cloud, params, leaves[i]);
            merge_local(local);
            INF_INFO("Octree RANSAC: leaf {}/{}, {} planes so far",
                     i + 1, total_leaves, merged.num_planes());
            params.on_progress(i + 1, total_leaves, merged);
        }
    } else {
        // 并行处理
        const int hw = INFMVS_OMP_MAX_THREADS();
        const int max_threads = (params.max_threads > 0)
            ? std::max(1, std::min(params.max_threads, hw))
            : std::max(1, hw);
        const int thread_count = std::min<int>(max_threads, static_cast<int>(leaves.size()));
        INF_INFO("Octree RANSAC: threads={}", thread_count);

        const size_t log_step = std::max<size_t>(1, total_leaves / 20);
        std::atomic<size_t> processed{0};
        std::atomic<size_t> next_log{log_step};

        std::vector<LocalDetectionResult> locals(leaves.size());

        INFMVS_OMP_SET_NUM_THREADS(thread_count);
        INFMVS_OMP_PARALLEL_FOR
        for (int i = 0; i < static_cast<int>(leaves.size()); ++i) {
            locals[static_cast<size_t>(i)] = detect_planes_ransac_local(
                cloud, params, leaves[static_cast<size_t>(i)]);
            const size_t done = processed.fetch_add(1) + 1;
            size_t expected = next_log.load();
            if (done >= expected) {
                const size_t new_next = expected + log_step;
                if (next_log.compare_exchange_strong(expected, new_next)) {
                    const int percent = static_cast<int>((100.0 * done) / total_leaves);
                    INF_INFO("Octree RANSAC progress: {}% ({}/{})", percent, done, total_leaves);
                }
            }
        }

        // 合并所有叶子的检测结果
        for (auto& local : locals) {
            merge_local(local);
        }
    }

    INF_INFO("Octree RANSAC before merge: {} planes", merged.num_planes());

    // 合并相似平面（跨 leaf 的同一平面会被切碎，需要合并）
    merged = merge_similar_planes(merged, 0.98f, params.distance_threshold * 2.0f);

    INF_INFO("Octree RANSAC detected {} planes, {} unlabeled points, {} leaves",
             merged.num_planes(), merged.num_unlabeled(), leaves.size());
    return merged;
}

core::Result<PlaneDetectionResult> detect_planes_region_growing_octree(
    const PointCloud& cloud,
    const PlaneDetectionParams& params) {

    INF_INFO("Octree RegionGrowing start: {} points, max_points={}, max_depth={}",
             cloud.size(), params.octree_max_points, params.octree_max_depth);

    // 构建 Point_3 用于 Octree
    std::vector<Point_3> points;
    points.reserve(cloud.size());
    for (const auto& p : cloud.positions) {
        points.emplace_back(p.x(), p.y(), p.z());
    }

    IndexList indices(cloud.size());
    std::iota(indices.begin(), indices.end(), 0);

    IndexPointMap point_map;
    point_map.points = &points;
    CgalOctree octree(indices, point_map);
    INF_INFO("Octree RegionGrowing: building octree");
    octree.refine(CGAL::Orthtrees::Maximum_depth_and_maximum_contained_elements(
        params.octree_max_depth, params.octree_max_points));

    // 收集叶子
    std::vector<std::vector<size_t>> leaves;
    leaves.reserve(256);
    for (auto node : octree.traverse<CGAL::Orthtrees::Leaves_traversal<CgalOctree>>()) {
        const auto& data = octree.data(node);
        if (data.begin() != data.end()) {
            std::vector<size_t> leaf_indices(data.begin(), data.end());
            leaves.push_back(std::move(leaf_indices));
        }
    }
    INF_INFO("Octree RegionGrowing: {} leaves", leaves.size());

    PlaneDetectionResult merged;
    merged.point_labels.resize(cloud.size(), kUnlabeledPatch);
    if (leaves.empty()) {
        INF_INFO("Octree RegionGrowing detected 0 planes, {} unlabeled points, 0 leaves",
                 merged.num_unlabeled());
        return merged;
    }

    // 合并单个 leaf 结果到 merged 的 lambda
    auto merge_local = [&merged](const LocalDetectionResult& local) {
        const size_t base = merged.planes.size();
        merged.planes.insert(
            merged.planes.end(),
            local.result.planes.begin(),
            local.result.planes.end());
        merged.plane_point_indices.resize(merged.planes.size());

        for (size_t p = 0; p < local.result.plane_point_indices.size(); ++p) {
            const size_t global_plane = base + p;
            auto& global_indices = merged.plane_point_indices[global_plane];
            const auto& local_indices = local.result.plane_point_indices[p];
            global_indices.reserve(local_indices.size());
            for (const size_t local_idx : local_indices) {
                if (local_idx >= local.indices.size()) continue;
                const size_t global_idx = local.indices[local_idx];
                merged.point_labels[global_idx] = static_cast<int32_t>(global_plane);
                global_indices.push_back(global_idx);
            }
        }
    };

    const size_t total_leaves = leaves.size();

    // 如果设置了回调，顺序处理并在每个 leaf 后回调
    if (params.on_progress) {
        INF_INFO("Octree RegionGrowing: sequential mode (callback enabled)");
        for (size_t i = 0; i < leaves.size(); ++i) {
            auto local = detect_planes_region_growing_local(cloud, params, leaves[i]);
            merge_local(local);
            INF_INFO("Octree RegionGrowing: leaf {}/{}, {} planes so far",
                     i + 1, total_leaves, merged.num_planes());
            params.on_progress(i + 1, total_leaves, merged);
        }
    } else {
        // 并行处理
        const int hw = INFMVS_OMP_MAX_THREADS();
        const int max_threads = (params.max_threads > 0)
            ? std::max(1, std::min(params.max_threads, hw))
            : std::max(1, hw);
        const int thread_count = std::min<int>(max_threads, static_cast<int>(leaves.size()));
        INF_INFO("Octree RegionGrowing: threads={}", thread_count);

        const size_t log_step = std::max<size_t>(1, total_leaves / 20);
        std::atomic<size_t> processed{0};
        std::atomic<size_t> next_log{log_step};

        std::vector<LocalDetectionResult> locals(leaves.size());

        INFMVS_OMP_SET_NUM_THREADS(thread_count);
        INFMVS_OMP_PARALLEL_FOR
        for (int i = 0; i < static_cast<int>(leaves.size()); ++i) {
            locals[static_cast<size_t>(i)] = detect_planes_region_growing_local(
                cloud, params, leaves[static_cast<size_t>(i)]);
            const size_t done = processed.fetch_add(1) + 1;
            size_t expected = next_log.load();
            if (done >= expected) {
                const size_t new_next = expected + log_step;
                if (next_log.compare_exchange_strong(expected, new_next)) {
                    const int percent = static_cast<int>((100.0 * done) / total_leaves);
                    INF_INFO("Octree RegionGrowing progress: {}% ({}/{})", percent, done, total_leaves);
                }
            }
        }

        // 合并所有叶子的检测结果
        for (auto& local : locals) {
            merge_local(local);
        }
    }

    INF_INFO("Octree RegionGrowing before merge: {} planes", merged.num_planes());

    // 合并相似平面
    merged = merge_similar_planes(merged, 0.98f, params.distance_threshold * 2.0f);

    INF_INFO("Octree RegionGrowing detected {} planes, {} unlabeled points, {} leaves",
             merged.num_planes(), merged.num_unlabeled(), leaves.size());
    return merged;
}

// RANSAC 实现
core::Result<PlaneDetectionResult> detect_planes_ransac(
    const PointCloud& cloud,
    const PlaneDetectionParams& params) {
    
    INF_DEBUG("RANSAC plane detection: {} points, distance_threshold={}", 
              cloud.size(), params.distance_threshold);

    auto points = to_cgal_points(cloud);
    
    EfficientRansac ransac;
    ransac.set_input(points);
    ransac.add_shape_factory<CgalPlane>();

    EfficientRansac::Parameters cgal_params;
    cgal_params.probability = params.probability;
    cgal_params.min_points = params.min_points;
    cgal_params.epsilon = params.distance_threshold;
    cgal_params.cluster_epsilon = params.distance_threshold * params.cluster_epsilon_factor;
    cgal_params.normal_threshold = params.normal_threshold;

    ransac.detect(cgal_params);

    PlaneDetectionResult result;
    result.point_labels.resize(cloud.size(), kUnlabeledPatch);

    const auto& shapes = ransac.shapes();
    result.planes.reserve(shapes.size());
    result.plane_point_indices.reserve(shapes.size());

    int32_t plane_idx = 0;
    for (const auto& shape : shapes) {
        if (auto* plane_shape = dynamic_cast<CgalPlane*>(shape.get())) {
            // 使用 Plane 的 conversion operator 转换为 Plane_3
            Plane_3 plane_3 = *plane_shape;
            result.planes.push_back(cgal_plane_to_plane(plane_3));

            std::vector<size_t> indices;
            indices.reserve(shape->indices_of_assigned_points().size());
            for (size_t idx : shape->indices_of_assigned_points()) {
                indices.push_back(idx);
                result.point_labels[idx] = plane_idx;
            }
            result.plane_point_indices.push_back(std::move(indices));
            ++plane_idx;
        }
    }

    INF_INFO("RANSAC detected {} planes, {} unlabeled points",
             result.num_planes(), result.num_unlabeled());

    return result;
}

// Region Growing 实现
core::Result<PlaneDetectionResult> detect_planes_region_growing(
    const PointCloud& cloud,
    const PlaneDetectionParams& params) {

    if (!cloud.has_normals()) {
        return make_error("RegionGrowing requires normals");
    }

    INF_INFO("RegionGrowing plane detection: {} points, k={}, dist={}, angle={}deg, min_pts={}",
              cloud.size(), params.k_neighbors, params.distance_threshold,
              params.smoothness_threshold, params.min_points);

    auto points = to_cgal_points(cloud);
    
    // 验证法线有效性
    size_t valid_normals = 0;
    for (const auto& pn : points) {
        const auto& n = pn.second;
        double len = std::sqrt(CGAL::to_double(n.squared_length()));
        if (len > 0.5 && len < 1.5) ++valid_normals;  // 接近单位长度
    }
    INF_INFO("RegionGrowing: {} / {} valid normals ({:.1f}%)",
             valid_normals, points.size(),
             100.0 * valid_normals / std::max<size_t>(1, points.size()));

    // 使用 named parameters 和自定义 property maps
    RgNeighborQuery neighbor_query(
        points,
        CGAL::parameters::k_neighbors(static_cast<std::size_t>(params.k_neighbors))
            .point_map(IteratorPointMap()));

    RgRegionType region_type(
        CGAL::parameters::maximum_distance(static_cast<FT>(params.distance_threshold))
            // CGAL 的 maximum_angle 单位是“度”（不是弧度）
            // 参考: CGAL Shape_detection::Point_set::Least_squares_plane_fit_region
            .maximum_angle(static_cast<FT>(params.smoothness_threshold))
            .minimum_region_size(params.min_points)
            .point_map(IteratorPointMap())
            .normal_map(IteratorNormalMap()));

    RegionGrowing region_growing(points, neighbor_query, region_type);

    // 检测区域
    using PrimitiveAndRegion = typename RegionGrowing::Primitive_and_region;
    std::vector<PrimitiveAndRegion> regions;
    region_growing.detect(std::back_inserter(regions));

    INF_INFO("RegionGrowing raw: {} regions detected", regions.size());

    // 统计区域大小分布
    size_t skipped = 0;
    size_t max_region_size = 0;
    for (const auto& pr : regions) {
        max_region_size = std::max(max_region_size, pr.second.size());
        if (pr.second.size() < params.min_points) ++skipped;
    }
    INF_INFO("RegionGrowing: max_region_size={}, skipped {} (< min_points={})",
             max_region_size, skipped, params.min_points);

    PlaneDetectionResult result;
    result.point_labels.resize(cloud.size(), kUnlabeledPatch);
    result.planes.reserve(regions.size());
    result.plane_point_indices.reserve(regions.size());

    int32_t plane_idx = 0;
    for (const auto& pr : regions) {
        const auto& region = pr.second;
        if (region.size() < params.min_points) continue;

        // 从检测到的 primitive 获取平面
        const Plane_3& plane_3 = pr.first;
        result.planes.push_back(cgal_plane_to_plane(plane_3));

        // 提取点索引
        std::vector<size_t> indices;
        indices.reserve(region.size());
        for (const auto& it : region) {
            size_t idx = static_cast<size_t>(std::distance(points.cbegin(), it));
            indices.push_back(idx);
            result.point_labels[idx] = plane_idx;
        }
        result.plane_point_indices.push_back(std::move(indices));
        ++plane_idx;
    }

    INF_INFO("RegionGrowing detected {} planes, {} unlabeled points",
             result.num_planes(), result.num_unlabeled());

    return result;
}

// 生成随机颜色 (确定性，基于 plane index)
// 输出 0-255 范围，用于 PLY 文件
core::Vec3f generate_plane_color(int32_t plane_idx) {
    // golden ratio 生成均匀分布的色相
    constexpr float kGoldenRatio = 0.618033988749895f;
    float hue = std::fmod(plane_idx * kGoldenRatio, 1.0f);
    
    // HSV to RGB (S=0.8, V=0.95) - 鲜艳的颜色
    constexpr float S = 0.8f;
    constexpr float V = 0.95f;
    float h = hue * 6.0f;
    int i = static_cast<int>(h);
    float f = h - static_cast<float>(i);
    float p = V * (1.0f - S);
    float q = V * (1.0f - S * f);
    float t = V * (1.0f - S * (1.0f - f));

    float r, g, b;
    switch (i % 6) {
        case 0: r = V; g = t; b = p; break;
        case 1: r = q; g = V; b = p; break;
        case 2: r = p; g = V; b = t; break;
        case 3: r = p; g = q; b = V; break;
        case 4: r = t; g = p; b = V; break;
        default: r = V; g = p; b = q; break;
    }
    // 转换到 0-255 范围
    return {r * 255.0f, g * 255.0f, b * 255.0f};
}

}  // namespace

core::Result<PlaneDetectionResult> detect_planes(
    const PointCloud& cloud,
    const PlaneDetectionParams& params) {

    if (cloud.empty()) {
        return make_error("empty point cloud");
    }
    if (cloud.size() < params.min_points) {
        return make_error("point cloud too small for plane detection");
    }

    switch (params.method) {
        case PlaneDetectionMethod::RANSAC:
            if (params.enable_octree) {
                return detect_planes_ransac_octree(cloud, params);
            }
            return detect_planes_ransac(cloud, params);
        case PlaneDetectionMethod::RegionGrowing:
            if (params.enable_octree) {
                return detect_planes_region_growing_octree(cloud, params);
            }
            return detect_planes_region_growing(cloud, params);
        default:
            return make_error("unknown detection method");
    }
}

PointCloud extract_plane_cloud(
    const PointCloud& cloud,
    const PlaneDetectionResult& result,
    size_t plane_index) {

    if (plane_index >= result.plane_point_indices.size()) {
        return {};
    }
    return extract_by_indices(cloud, result.plane_point_indices[plane_index]);
}

PointCloud extract_unlabeled_cloud(
    const PointCloud& cloud,
    const PlaneDetectionResult& result) {

    std::vector<size_t> indices;
    indices.reserve(result.num_unlabeled());
    for (size_t i = 0; i < result.point_labels.size(); ++i) {
        if (result.point_labels[i] == kUnlabeledPatch) {
            indices.push_back(i);
        }
    }
    return extract_by_indices(cloud, indices);
}

PointCloud colorize_by_planes(
    const PointCloud& cloud,
    const PlaneDetectionResult& result) {

    PointCloud colored = cloud;
    colored.colors.resize(cloud.size());

    const core::Vec3f unlabeled_color(128.0f, 128.0f, 128.0f);  // 灰色 (0-255)

    for (size_t i = 0; i < cloud.size(); ++i) {
        int32_t label = result.point_labels[i];
        if (label == kUnlabeledPatch) {
            colored.colors[i] = unlabeled_color;
        } else {
            colored.colors[i] = generate_plane_color(label);
        }
    }

    return colored;
}

}  // namespace inf::pc
