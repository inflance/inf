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
#include <filesystem>
#include <vector>

namespace inf::io {

// ============================================================================
// COLMAP 数据结构
// ============================================================================
struct Camera {
    uint32_t    id = 0;
    std::string model;
    uint32_t    width = 0, height = 0;
    std::vector<double> params;
};

struct Image {
    uint32_t       id = 0;
    core::Quatd    rotation;
    core::Vec3d    translation;
    uint32_t       camera_id = 0;
    std::string    name;
};

struct Point3D {
    uint64_t    id = 0;
    core::Vec3d position;
    core::Vec3f color;
    double      error = 0.0;
};

struct ColmapModel {
    std::vector<Camera>  cameras;
    std::vector<Image>   images;
    std::vector<Point3D> points3d;
};

// ============================================================================
// I/O 函数
// ============================================================================
[[nodiscard]] INFIO_API core::Result<ColmapModel> read_colmap_text(
    const std::filesystem::path& model_dir);

}  // namespace inf::io
