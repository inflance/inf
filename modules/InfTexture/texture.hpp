#pragma once
/// @file texture.hpp
/// @brief InfTexture 内部实现

#if defined(_WIN32)
  #ifdef INFTEXTURE_EXPORTS
    #define INFTEXTURE_API __declspec(dllexport)
  #else
    #define INFTEXTURE_API __declspec(dllimport)
  #endif
#else
  #define INFTEXTURE_API __attribute__((visibility("default")))
#endif

#include "InfCore.hpp"
#include "InfMesh.hpp"
#include <vector>

namespace inf::tex {

// ============================================================================
// 纹理数据结构
// ============================================================================
struct TextureAtlas {
    std::vector<uint8_t> data;
    uint32_t width  = 0;
    uint32_t height = 0;
    uint32_t channels = 4;  // RGBA
};

struct UVMapping {
    std::vector<core::Vec2f> uvs;  // 每个顶点的 UV 坐标
};

// ============================================================================
// 纹理操作
// ============================================================================

/// @brief UV 展开
[[nodiscard]] INFTEXTURE_API UVMapping unwrap_uv(const mesh::Mesh& mesh);

/// @brief 生成纹理图集
[[nodiscard]] INFTEXTURE_API TextureAtlas generate_atlas(
    const mesh::Mesh& mesh,
    const UVMapping& uv,
    uint32_t resolution = 2048);

}  // namespace inf::tex
