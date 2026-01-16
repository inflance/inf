#pragma once
/// @file core.hpp
/// @brief InfCore 内部实现

// ============================================================================
// DLL 导出宏
// ============================================================================
#if defined(_WIN32)
  #ifdef INFCORE_EXPORTS
    #define INFCORE_API __declspec(dllexport)
  #else
    #define INFCORE_API __declspec(dllimport)
  #endif
#else
  #define INFCORE_API __attribute__((visibility("default")))
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <string_view>
#include <expected>

namespace inf::core {

// ============================================================================
// 类型别名
// ============================================================================
using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Vector4f;
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Mat3f = Eigen::Matrix3f;
using Mat4f = Eigen::Matrix4f;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Quatf = Eigen::Quaternionf;
using Quatd = Eigen::Quaterniond;

// ============================================================================
// 错误处理
// ============================================================================
enum class ErrorCode : uint32_t {
    kSuccess = 0,
    kFileNotFound,
    kParseError,
    kInvalidArgument,
    kOutOfMemory,
    kUnsupportedFormat,
};

struct Error {
    ErrorCode   code = ErrorCode::kSuccess;
    std::string message;
    [[nodiscard]] bool ok() const noexcept { return code == ErrorCode::kSuccess; }
};

template <typename T>
using Result = std::expected<T, Error>;

// ============================================================================
// 版本
// ============================================================================
[[nodiscard]] INFCORE_API std::string_view version() noexcept;

}  // namespace inf::core
