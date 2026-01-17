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
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>
#include <unordered_map>

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
    Success = 0,
    FileNotFound,
    ParseError,
    InvalidArgument,
    OutOfMemory,
    UnsupportedFormat,
};

struct Error {
    ErrorCode   code = ErrorCode::Success;
    std::string message;
    [[nodiscard]] bool ok() const noexcept { return code == ErrorCode::Success; }
};

// ============================================================================
// C++17 兼容的 Expected 实现
// ============================================================================
template <typename E>
struct Unexpected {
    E error;
    explicit Unexpected(E e) : error(std::move(e)) {}
};

template <typename E>
Unexpected<std::decay_t<E>> unexpected(E&& e) {
    return Unexpected<std::decay_t<E>>(std::forward<E>(e));
}

template <typename T, typename E>
class Expected {
    std::variant<T, E> data_;
    
public:
    Expected(T value) : data_(std::move(value)) {}
    Expected(Unexpected<E> err) : data_(std::move(err.error)) {}
    
    [[nodiscard]] bool has_value() const noexcept { return std::holds_alternative<T>(data_); }
    [[nodiscard]] explicit operator bool() const noexcept { return has_value(); }
    
    [[nodiscard]] T& value() & { return std::get<T>(data_); }
    [[nodiscard]] const T& value() const& { return std::get<T>(data_); }
    [[nodiscard]] T&& value() && { return std::get<T>(std::move(data_)); }
    
    [[nodiscard]] E& error() & { return std::get<E>(data_); }
    [[nodiscard]] const E& error() const& { return std::get<E>(data_); }
    [[nodiscard]] E&& error() && { return std::get<E>(std::move(data_)); }
    
    [[nodiscard]] T& operator*() & { return value(); }
    [[nodiscard]] const T& operator*() const& { return value(); }
    [[nodiscard]] T* operator->() { return &value(); }
    [[nodiscard]] const T* operator->() const { return &value(); }
};

// void 特化：用于只返回成功/失败的函数
template <typename E>
class Expected<void, E> {
    std::optional<E> error_;
    
public:
    Expected() = default;
    Expected(Unexpected<E> err) : error_(std::move(err.error)) {}
    
    [[nodiscard]] bool has_value() const noexcept { return !error_.has_value(); }
    [[nodiscard]] explicit operator bool() const noexcept { return has_value(); }
    
    [[nodiscard]] E& error() & { return *error_; }
    [[nodiscard]] const E& error() const& { return *error_; }
};

template <typename T>
using Result = Expected<T, Error>;

// ============================================================================
// 版本
// ============================================================================
[[nodiscard]] INFCORE_API std::string_view version() noexcept;

}  // namespace inf::core
