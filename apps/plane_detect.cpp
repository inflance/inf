/// @file plane_detect.cpp
/// @brief 平面检测命令行工具

#include "InfPointCloud.hpp"
#include "InfIO.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>

namespace
{
	void log_elapsed(const char* tag, std::chrono::steady_clock::time_point start)
	{
		const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - start).count();
		INF_INFO("[timing] {}: {} ms", tag, ms);
	}

	inf::pc::PointCloud colorize_by_normals(const inf::pc::PointCloud& cloud)
	{
		if (!cloud.has_normals())
		{
			return cloud;
		}
		inf::pc::PointCloud colored = cloud;
		colored.colors.resize(cloud.size());
		size_t invalid_count = 0;
		size_t valid_count = 0;
		for (size_t i = 0; i < cloud.size(); ++i)
		{
			const inf::core::Vec3f n = cloud.normals[i];
			const float len = n.norm();
			if (!std::isfinite(len) || len < 1e-6f ||
				!std::isfinite(n.x()) || !std::isfinite(n.y()) || !std::isfinite(n.z()))
			{
				colored.colors[i] = inf::core::Vec3f(128.0f, 128.0f, 128.0f);
				++invalid_count;
				continue;
			}
			const inf::core::Vec3f nn = n / len;
			const inf::core::Vec3f mapped = (nn.array() * 0.5f + 0.5f).matrix();
			colored.colors[i] = mapped * 255.0f;
			++valid_count;
		}
		INF_INFO("Normal visualization: {} valid / {} invalid normals", valid_count, invalid_count);
		return colored;
	}

	inf::pc::Plane fit_plane_least_squares(
		const inf::pc::PointCloud& cloud,
		const std::vector<size_t>& indices,
		const inf::pc::Plane& fallback)
	{
		if (indices.size() < 3)
		{
			return fallback;
		}

		inf::core::Vec3f centroid = inf::core::Vec3f::Zero();
		for (size_t idx : indices)
		{
			centroid += cloud.positions[idx];
		}
		centroid /= static_cast<float>(indices.size());

		inf::core::Mat3f cov = inf::core::Mat3f::Zero();
		for (size_t idx : indices)
		{
			const inf::core::Vec3f d = cloud.positions[idx] - centroid;
			cov += d * d.transpose();
		}

		Eigen::SelfAdjointEigenSolver<inf::core::Mat3f> solver(cov);
		inf::core::Vec3f normal = solver.eigenvectors().col(0);
		normal.normalize();

		inf::pc::Plane plane;
		plane.normal = normal;
		plane.d = -normal.dot(centroid);
		return plane;
	}

	inf::pc::PlaneDetectionResult assign_planes_to_full_cloud(
		const inf::pc::PointCloud& cloud,
		const inf::pc::PlaneDetectionResult& ransac_result,
		float distance_threshold)
	{
		inf::pc::PlaneDetectionResult result;
		result.planes = ransac_result.planes;
		result.point_labels.assign(cloud.size(), inf::pc::kUnlabeledPatch);
		result.plane_point_indices.resize(result.planes.size());

		for (size_t i = 0; i < cloud.size(); ++i)
		{
			float best_dist = std::numeric_limits<float>::max();
			int32_t best_idx = inf::pc::kUnlabeledPatch;
			for (size_t p = 0; p < result.planes.size(); ++p)
			{
				const float d = result.planes[p].distance(cloud.positions[i]);
				if (d < best_dist)
				{
					best_dist = d;
					best_idx = static_cast<int32_t>(p);
				}
			}
			if (best_idx != inf::pc::kUnlabeledPatch && best_dist <= distance_threshold)
			{
				result.point_labels[i] = best_idx;
				result.plane_point_indices[best_idx].push_back(i);
			}
		}

		for (size_t p = 0; p < result.planes.size(); ++p)
		{
			result.planes[p] = fit_plane_least_squares(
				cloud, result.plane_point_indices[p], result.planes[p]);
		}

		return result;
	}

	void print_usage(const char* prog)
	{
		std::cerr << "Usage: " << prog << " <input.ply> [output_dir] [--no-backfill]\n";
		std::cerr << "\n";
		std::cerr << "Options:\n";
		std::cerr << "  input.ply     Input point cloud (PLY format)\n";
		std::cerr << "  output_dir    Output directory (default: same as input)\n";
		std::cerr << "  --no-backfill Do not assign planes to full cloud (save only detect cloud)\n";
		std::cerr << "\n";
		std::cerr << "Output files:\n";
		std::cerr << "  <name>_normals.ply  Normal visualization\n";
		std::cerr << "  <name>_planes.ply   Plane detection result\n";
	}
} // namespace

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		print_usage(argv[0]);
		return 1;
	}

	bool no_backfill = false;
	std::filesystem::path input_path;
	std::filesystem::path output_dir;

	for (int i = 1; i < argc; ++i)
	{
		const std::string_view arg(argv[i]);
		if (arg == "--no-backfill")
		{
			no_backfill = true;
			continue;
		}
		if (input_path.empty())
		{
			input_path = argv[i];
			continue;
		}
		if (output_dir.empty())
		{
			output_dir = argv[i];
			continue;
		}
	}

	if (input_path.empty())
	{
		print_usage(argv[0]);
		return 1;
	}
	if (output_dir.empty())
	{
		output_dir = input_path.parent_path();
	}

	// 如果输出目录为空（文件在当前目录），使用当前目录
	if (output_dir.empty())
	{
		output_dir = ".";
	}

	if (!std::filesystem::exists(input_path))
	{
		std::cerr << "Error: File not found: " << input_path << "\n";
		return 1;
	}

	// 确保输出目录存在
	if (!std::filesystem::exists(output_dir))
	{
		std::filesystem::create_directories(output_dir);
	}

	std::string stem = input_path.stem().string();

	// 读取点云
	INF_INFO("Loading point cloud: {}", input_path.string());
	auto t_read = std::chrono::steady_clock::now();
	auto cloud_result = inf::io::read_point_cloud(input_path);
	if (!cloud_result.has_value())
	{
		std::cerr << "Error: Failed to read point cloud\n";
		return 1;
	}
	log_elapsed("read_point_cloud", t_read);

	auto cloud_full = std::move(cloud_result.value());
	INF_INFO("Loaded point cloud: {} points, has_normals={}",
	         cloud_full.size(), cloud_full.has_normals());

	// 体素采样（逼近目标点数）：先降采样再做单块检测（更快更稳）
	const size_t target_points = std::min<size_t>(1000000, cloud_full.size());
	inf::pc::PointCloud sampled_cloud;
	bool use_sampled = false;
	const inf::pc::PointCloud* detect_cloud = &cloud_full;
	float voxel_size = 0.0f;
	if (target_points < cloud_full.size())
	{
		INF_INFO("Voxel downsample to target: {}", target_points);
		auto t_ds = std::chrono::steady_clock::now();
		auto ds = inf::pc::voxel_downsample_to_target(
			cloud_full, target_points, &voxel_size, 0.05f, 18);
		if (!ds.has_value())
		{
			std::cerr << "Error: voxel_downsample_to_target failed\n";
			return 1;
		}
		sampled_cloud = std::move(ds.value());
		use_sampled = true;
		detect_cloud = &sampled_cloud;
		log_elapsed("voxel_downsample_to_target", t_ds);
		INF_INFO("Downsampled: {} -> {} (voxel_size={:.6f})",
		         cloud_full.size(), detect_cloud->size(), voxel_size);
	}

	// 法线处理（优先在采样后的点云上做，避免 1000 万点直接估计法线）
	if (detect_cloud->has_normals())
	{
		INF_INFO("Estimating normals on detect cloud ({} points)...", detect_cloud->size());
		auto t_normals = std::chrono::steady_clock::now();
		inf::pc::NormalEstimationParams normal_params;
		normal_params.k_neighbors = 30;
		auto normal_result = use_sampled
			                     ? inf::pc::estimate_normals(sampled_cloud, normal_params)
			                     : inf::pc::estimate_normals(cloud_full, normal_params);
		if (!normal_result.has_value())
		{
			std::cerr << "Error: Failed to estimate normals\n";
			return 1;
		}
		log_elapsed("estimate_normals", t_normals);
	}
	else
	{
		INF_INFO("Using existing normals");
	}

	// 法线可视化并保存
	{
		auto t_norm_vis = std::chrono::steady_clock::now();
		auto normal_colored = colorize_by_normals(*detect_cloud);
		std::filesystem::path normals_path = output_dir / (stem + "_normals.ply");
		if (auto normals_write = inf::io::write_point_cloud(normals_path, normal_colored, true); normals_write.
			has_value())
		{
			INF_INFO("Saved normals visualization to: {}", normals_path.string());
		}
		else
		{
			INF_WARN("Failed to save normals visualization");
		}
		log_elapsed("write_normals_ply", t_norm_vis);
	}

	// 平面检测参数：单块检测（不做 leaf 分块），输入是采样后的点云
	inf::pc::PlaneDetectionParams params;
	params.method = inf::pc::PlaneDetectionMethod::RegionGrowing;
	params.distance_threshold = 0.1f; // 放宽一点，单位是米
	params.normal_threshold = 0.9f;
	params.min_points = 100;
	params.k_neighbors = 30;
	params.smoothness_threshold = 15.0f; // 放宽角度阈值（度）
	params.enable_octree = false; // 单块处理
	params.max_threads = 0;

	// 中间结果输出路径
	std::filesystem::path partial_path = output_dir / (stem + "_partial.ply");
	std::filesystem::path output_path = output_dir / (stem + "_planes.ply");

	INF_INFO("Detecting planes with params:");
	INF_INFO("  method: RegionGrowing");
	INF_INFO("  distance_threshold: {}", params.distance_threshold);
	INF_INFO("  min_points: {}", params.min_points);
	INF_INFO("  detect_points: {}", detect_cloud->size());
	if (use_sampled)
	{
		INF_INFO("  voxel_size: {:.6f}", voxel_size);
	}

	auto t_detect = std::chrono::steady_clock::now();
	auto detect_result = inf::pc::detect_planes(*detect_cloud, params);
	if (!detect_result.has_value())
	{
		std::cerr << "Error: Plane detection failed\n";
		return 1;
	}
	log_elapsed("detect_planes", t_detect);

	inf::pc::PlaneDetectionResult result;
	const inf::pc::PointCloud* output_cloud = &cloud_full;
	if (no_backfill)
	{
		// 不回填：直接输出检测点云的结果
		result = std::move(detect_result.value());
		output_cloud = detect_cloud;
		INF_INFO("Backfill disabled: output detect cloud only ({} points)", output_cloud->size());
	}
	else
	{
		// 回填到全量点云并最小二乘精拟合
		auto t_refit = std::chrono::steady_clock::now();
		result = assign_planes_to_full_cloud(cloud_full, *detect_result, params.distance_threshold);
		log_elapsed("assign_and_refit", t_refit);
	}

	INF_INFO("Detected {} planes, {} unlabeled points",
	         result.num_planes(), result.num_unlabeled());

	// 打印每个平面的信息
	for (size_t i = 0; i < result.num_planes() && i < 20; ++i)
	{
		const auto& plane = result.planes[i];
		INF_INFO("  Plane {}: normal=({:.3f}, {:.3f}, {:.3f}), d={:.3f}, {} points",
		         i, plane.normal.x(), plane.normal.y(), plane.normal.z(),
		         plane.d, result.plane_point_indices[i].size());
	}
	if (result.num_planes() > 20)
	{
		INF_INFO("  ... and {} more planes", result.num_planes() - 20);
	}

	// 保存最终结果
	auto colored = inf::pc::colorize_by_planes(*output_cloud, result);
	auto write_result = inf::io::write_point_cloud(output_path, colored, true);

	if (write_result.has_value())
	{
		INF_INFO("Saved final result to: {}", output_path.string());
	}
	else
	{
		std::cerr << "Error: Failed to save output\n";
		return 1;
	}

	// 删除中间文件
	std::filesystem::remove(partial_path);

	INF_INFO("Done!");
	return 0;
}
