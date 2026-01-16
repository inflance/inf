#pragma once
/// @file pipeline.hpp
/// @brief InfMeshPipeline 内部实现

#if defined(_WIN32)
  #ifdef INFPIPELINE_EXPORTS
    #define INFPIPELINE_API __declspec(dllexport)
  #else
    #define INFPIPELINE_API __declspec(dllimport)
  #endif
#else
  #define INFPIPELINE_API __attribute__((visibility("default")))
#endif

#include "InfCore.hpp"
#include "InfIO.hpp"
#include "InfMesh.hpp"
#include "InfTexture.hpp"
#include <filesystem>

namespace inf::pipeline {

// ============================================================================
// 流水线配置
// ============================================================================
struct PipelineConfig {
    float mesh_simplify_ratio = 0.5f;
    uint32_t texture_resolution = 2048;
    bool compute_normals = true;
};

// ============================================================================
// 流水线结果
// ============================================================================
struct PipelineResult {
    mesh::Mesh mesh;
    tex::TextureAtlas texture;
    tex::UVMapping uv;
};

// ============================================================================
// 流水线函数
// ============================================================================

/// @brief 运行完整重建流水线
[[nodiscard]] INFPIPELINE_API core::Result<PipelineResult> run(
    const std::filesystem::path& colmap_dir,
    const PipelineConfig& config = {});

}  // namespace inf::pipeline
