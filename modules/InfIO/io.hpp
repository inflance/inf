#pragma once
/// @file io.hpp
/// @brief InfIO 内部实现

#if defined(_WIN32)
  #ifdef INFIO_EXPORTS
    #define INFIO_API __declspec(dllexport)
  #else
    #define INFIO_API __declspec(dllimport)
  #endif
#else
  #define INFIO_API __attribute__((visibility("default")))
#endif

#include "InfCore.hpp"
#include "InfMesh.hpp"
#include "InfPointCloud.hpp"
#include <cstdint>
#include <filesystem>

namespace inf::io {

// ============================================================================
// Format 类型
// ============================================================================

enum class IoFormat : uint8_t {
    Unknown = 0,
    Mesh,
    PointCloud,
    SparseModel
};

/// @brief 通过后缀匹配 Format
[[nodiscard]] INFIO_API IoFormat format_from_extension(
    const std::filesystem::path& path);

/// @brief 通过路径匹配 Format（目录检测空三模型）
[[nodiscard]] INFIO_API IoFormat format_from_path(
    const std::filesystem::path& path);

// ============================================================================
// COLMAP I/O 函数
// ============================================================================

/// @brief 读取 COLMAP 模型 (自动检测 text/binary 格式)
/// @param model_dir 包含 cameras.{txt,bin}, images.{txt,bin}, points3D.{txt,bin} 的目录
/// @return SparseModel 或错误
[[nodiscard]] INFIO_API core::Result<core::SparseModel> read_colmap(
    const std::filesystem::path& model_dir);

/// @brief 读取 COLMAP Text 格式
/// @param model_dir 包含 cameras.txt, images.txt, points3D.txt 的目录
[[nodiscard]] INFIO_API core::Result<core::SparseModel> read_colmap_text(
    const std::filesystem::path& model_dir);

/// @brief 读取 COLMAP Binary 格式
/// @param model_dir 包含 cameras.bin, images.bin, points3D.bin 的目录
[[nodiscard]] INFIO_API core::Result<core::SparseModel> read_colmap_binary(
    const std::filesystem::path& model_dir);

// ============================================================================
// Mesh / Point Cloud I/O 函数
// ============================================================================

/// @brief 读取 PLY 网格
[[nodiscard]] INFIO_API core::Result<mesh::Mesh> read_ply_mesh(
    const std::filesystem::path& path);

/// @brief 写入 PLY 网格（仅三角形）
[[nodiscard]] INFIO_API core::Result<void> write_ply_mesh(
    const std::filesystem::path& path,
    const mesh::Mesh& mesh,
    bool binary = true);

/// @brief 读取点云（支持 .ply/.pcd）
[[nodiscard]] INFIO_API core::Result<pc::PointCloud> read_point_cloud(
    const std::filesystem::path& path);

/// @brief 写入点云（支持 .ply/.pcd）
[[nodiscard]] INFIO_API core::Result<void> write_point_cloud(
    const std::filesystem::path& path,
    const pc::PointCloud& cloud,
    bool binary = true);

}  // namespace inf::io
