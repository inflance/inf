#pragma once
/// @file mesh.hpp
/// @brief InfMesh 内部实现

#if defined(_WIN32)
  #ifdef INFMESH_EXPORTS
    #define INFMESH_API __declspec(dllexport)
  #else
    #define INFMESH_API __declspec(dllimport)
  #endif
#else
  #define INFMESH_API __attribute__((visibility("default")))
#endif

#include "InfCore.hpp"
#include <vector>
#include <cstdint>

namespace inf::mesh {

// ============================================================================
// 网格数据结构
// ============================================================================
struct Mesh {
    std::vector<core::Vec3f> vertices;
    std::vector<core::Vec3f> normals;
    std::vector<uint32_t>    indices;  // 三角形索引
};

// ============================================================================
// 网格操作
// ============================================================================

/// @brief 网格简化
[[nodiscard]] INFMESH_API Mesh simplify(const Mesh& mesh, float ratio);

/// @brief 计算法向量
INFMESH_API void compute_normals(Mesh& mesh);

}  // namespace inf::mesh
