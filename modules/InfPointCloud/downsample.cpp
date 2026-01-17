#include "point_cloud.hpp"
#include "pcl_adapter.hpp"
#include <algorithm>
#include <numeric>
#include <random>
#include <unordered_map>
#include <unordered_set>

namespace inf::pc {

namespace {
core::Unexpected<core::Error> err(const char* msg) {
    return core::unexpected(core::Error{core::ErrorCode::InvalidArgument, msg});
}

[[nodiscard]] size_t voxel_cell_count(const PointCloud& cloud, float voxel_size) {
    if (cloud.empty() || voxel_size <= 0.0f) return 0;
    const float inv_voxel = 1.0f / voxel_size;
    std::unordered_set<detail::VoxelKey, detail::VoxelKeyHash> cells;
    cells.reserve(cloud.size() / 8);
    for (const auto& p : cloud.positions) {
        cells.insert(detail::make_voxel_key(p, inv_voxel));
    }
    return cells.size();
}

[[nodiscard]] float clamp_positive(float v, float min_v) {
    if (!std::isfinite(v)) return min_v;
    return std::max(v, min_v);
}
}

core::Result<PointCloud> voxel_downsample(const PointCloud& cloud, float voxel_size) {
    if (cloud.empty()) return err("empty point cloud");
    if (voxel_size <= 0.0f) return err("voxel_size must be positive");

    const float inv_voxel = 1.0f / voxel_size;

    struct VoxelData {
        core::Vec3f position_sum = core::Vec3f::Zero();
        core::Vec3f normal_sum = core::Vec3f::Zero();
        core::Vec3f color_sum = core::Vec3f::Zero();
        float confidence_sum = 0.0f, scale_sum = 0.0f;
        size_t count = 0;
    };

    std::unordered_map<detail::VoxelKey, VoxelData, detail::VoxelKeyHash> voxel_map;
    voxel_map.reserve(cloud.size() / 8);

    const bool has_n = cloud.has_normals(), has_c = cloud.has_colors();
    const bool has_conf = cloud.has_confidences(), has_sc = cloud.has_scales();

    for (size_t i = 0; i < cloud.size(); ++i) {
        auto key = detail::make_voxel_key(cloud.positions[i], inv_voxel);
        auto& d = voxel_map[key];
        d.position_sum += cloud.positions[i];
        if (has_n) d.normal_sum += cloud.normals[i];
        if (has_c) d.color_sum += cloud.colors[i];
        if (has_conf) d.confidence_sum += cloud.confidences[i];
        if (has_sc) d.scale_sum += cloud.scales[i];
        ++d.count;
    }

    PointCloud result;
    result.positions.reserve(voxel_map.size());
    if (has_n) result.normals.reserve(voxel_map.size());
    if (has_c) result.colors.reserve(voxel_map.size());
    if (has_conf) result.confidences.reserve(voxel_map.size());
    if (has_sc) result.scales.reserve(voxel_map.size());

    for (const auto& [key, d] : voxel_map) {
        const float inv = 1.0f / static_cast<float>(d.count);
        result.positions.emplace_back(d.position_sum * inv);
        if (has_n) {
            core::Vec3f n = d.normal_sum * inv;
            float len = n.norm();
            if (len > 1e-6f) n /= len;
            result.normals.push_back(n);
        }
        if (has_c) result.colors.emplace_back(d.color_sum * inv);
        if (has_conf) result.confidences.push_back(d.confidence_sum * inv);
        if (has_sc) result.scales.push_back(d.scale_sum * inv);
    }
    return result;
}

core::Result<PointCloud> voxel_downsample_to_target(
    const PointCloud& cloud,
    size_t target_count,
    float* voxel_size_out,
    float rel_tol,
    int max_iters) {

    if (cloud.empty()) return err("empty point cloud");
    if (target_count == 0) return err("target_count must be positive");
    if (target_count >= cloud.size()) {
        if (voxel_size_out) *voxel_size_out = 0.0f;
        return PointCloud(cloud);
    }
    if (!(rel_tol >= 0.0f && std::isfinite(rel_tol))) rel_tol = 0.05f;
    if (max_iters <= 0) max_iters = 20;

    const auto [bmin, bmax] = cloud.bounding_box();
    const core::Vec3f ext = (bmax - bmin).cwiseAbs();
    const float vol = std::max(1e-12f, ext.x() * ext.y() * ext.z());

    // 初始猜测：假设点在 bbox 内近似均匀，voxel^3 ≈ volume / target_count
    float guess = std::cbrt(vol / static_cast<float>(target_count));
    guess = clamp_positive(guess, 1e-6f);

    // 构造二分区间：lo/hi 对应 voxel_size（lo 小→点多，hi 大→点少）
    float lo = guess * 0.25f;
    float hi = guess * 4.0f;
    lo = clamp_positive(lo, 1e-6f);
    hi = clamp_positive(hi, lo * 2.0f);

    size_t cnt_lo = voxel_cell_count(cloud, lo);
    size_t cnt_hi = voxel_cell_count(cloud, hi);

    // 扩展区间直到覆盖 target_count（或达到上限）
    for (int k = 0; k < 16 && (cnt_lo < target_count || cnt_hi > target_count); ++k) {
        if (cnt_lo < target_count) {
            // lo 太大（点太少）→ 减小 lo
            lo *= 0.5f;
            lo = clamp_positive(lo, 1e-6f);
            cnt_lo = voxel_cell_count(cloud, lo);
        }
        if (cnt_hi > target_count) {
            // hi 太小（点太多）→ 增大 hi
            hi *= 2.0f;
            hi = clamp_positive(hi, lo * 2.0f);
            cnt_hi = voxel_cell_count(cloud, hi);
        }
    }

    float best_voxel = guess;
    size_t best_cnt = voxel_cell_count(cloud, best_voxel);
    auto consider = [&](float v, size_t c) {
        const size_t best_diff = (best_cnt > target_count) ? (best_cnt - target_count) : (target_count - best_cnt);
        const size_t diff = (c > target_count) ? (c - target_count) : (target_count - c);
        if (diff < best_diff) {
            best_voxel = v;
            best_cnt = c;
        }
    };
    consider(lo, cnt_lo);
    consider(hi, cnt_hi);

    const float target_f = static_cast<float>(target_count);
    for (int it = 0; it < max_iters; ++it) {
        const float mid = 0.5f * (lo + hi);
        const size_t cnt_mid = voxel_cell_count(cloud, mid);
        consider(mid, cnt_mid);

        const float rel_err = std::abs(static_cast<float>(cnt_mid) - target_f) / target_f;
        if (rel_err <= rel_tol) {
            best_voxel = mid;
            best_cnt = cnt_mid;
            break;
        }

        if (cnt_mid > target_count) {
            // 点仍然太多 → 增大 voxel
            lo = mid;
        } else {
            // 点太少 → 减小 voxel
            hi = mid;
        }
    }

    if (voxel_size_out) *voxel_size_out = best_voxel;
    return voxel_downsample(cloud, best_voxel);
}

core::Result<PointCloud> random_downsample(const PointCloud& cloud, size_t target_count, uint32_t seed) {
    if (cloud.empty()) return err("empty point cloud");
    if (target_count == 0) return err("target_count must be positive");
    if (target_count >= cloud.size()) return PointCloud(cloud);

    std::vector<size_t> indices(cloud.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::mt19937 gen(seed == 0 ? std::random_device{}() : seed);
    std::shuffle(indices.begin(), indices.end(), gen);
    indices.resize(target_count);
    std::sort(indices.begin(), indices.end());
    return extract_by_indices(cloud, indices);
}

core::Result<PointCloud> uniform_downsample(const PointCloud& cloud, float min_distance) {
    if (cloud.empty()) return err("empty point cloud");
    if (min_distance <= 0.0f) return err("min_distance must be positive");

    const float inv_voxel = 1.0f / min_distance;
    const float min_dist_sq = min_distance * min_distance;

    std::unordered_map<detail::VoxelKey, std::vector<size_t>, detail::VoxelKeyHash> voxel_map;
    voxel_map.reserve(cloud.size() / 4);

    for (size_t i = 0; i < cloud.size(); ++i) {
        voxel_map[detail::make_voxel_key(cloud.positions[i], inv_voxel)].push_back(i);
    }

    std::vector<bool> selected(cloud.size(), false);
    std::vector<size_t> result_indices;
    result_indices.reserve(cloud.size() / 8);

    for (size_t i = 0; i < cloud.size(); ++i) {
        if (selected[i]) continue;
        const auto& p = cloud.positions[i];
        auto key = detail::make_voxel_key(p, inv_voxel);

        bool too_close = false;
        for (int dx = -1; dx <= 1 && !too_close; ++dx) {
            for (int dy = -1; dy <= 1 && !too_close; ++dy) {
                for (int dz = -1; dz <= 1 && !too_close; ++dz) {
                    detail::VoxelKey nk{key.x + dx, key.y + dy, key.z + dz};
                    auto it = voxel_map.find(nk);
                    if (it == voxel_map.end()) continue;
                    for (size_t j : it->second) {
                        if (!selected[j] || j == i) continue;
                        if ((cloud.positions[j] - p).squaredNorm() < min_dist_sq) {
                            too_close = true;
                            break;
                        }
                    }
                }
            }
        }
        if (!too_close) {
            selected[i] = true;
            result_indices.push_back(i);
        }
    }
    return extract_by_indices(cloud, result_indices);
}

core::Result<PointCloud> grid_downsample(const PointCloud& cloud, size_t step) {
    if (cloud.empty()) return err("empty point cloud");
    if (step == 0) return err("step must be positive");
    if (step == 1) return PointCloud(cloud);

    std::vector<size_t> indices;
    indices.reserve((cloud.size() + step - 1) / step);
    for (size_t i = 0; i < cloud.size(); i += step) indices.push_back(i);
    return extract_by_indices(cloud, indices);
}

}  // namespace inf::pc
