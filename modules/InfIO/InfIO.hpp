#pragma once
/// @file InfIO.hpp
/// @brief InfIO 公开接口

#include "io.hpp"
#include "ply.hpp"

// 对外暴露：
// - COLMAP I/O: read_colmap(), read_colmap_text(), read_colmap_binary()
// - PLY I/O: PlyFile (Ply), PlyHeader, PlyElement, PlyProperty
// - Mesh/PointCloud I/O: read_ply_mesh(), write_ply_mesh(), read_point_cloud(), write_point_cloud()
