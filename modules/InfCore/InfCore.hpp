#pragma once
/// @file InfCore.hpp
/// @brief InfCore 公开接口 - 只暴露必要的 API

#include "core.hpp"
#include "camera.hpp"
#include "log.hpp"

// 枚举工具 (基于 magic_enum) 和 去畸变工具 (基于 OpenCV)
// 注意: CUDA 代码中不要包含这些头文件
#if !defined(__CUDACC__) && !defined(__CUDA_ARCH__)
    #include "enum_utils.hpp"
    #include "undistort.hpp"
#endif

// 对外暴露：
// - 基础类型别名 (Vec3f, Mat4f, Quatd, ...)
// - 错误处理 (Error, Result<T>)
// - 相机模型与 SFM 数据结构 (Camera, Image, Point3D, SparseModel)
// - 日志系统 (Log, INF_INFO, ...)
// - 枚举工具 (enum_name, enum_cast, ...) [非 CUDA]
// - version()
