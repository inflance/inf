#pragma once
/// @file enum_utils.hpp
/// @brief 枚举工具 (基于 magic_enum)
/// @note CUDA 代码中不要包含此头文件

// CUDA 编译器不支持 magic_enum
#if defined(__CUDACC__) || defined(__CUDA_ARCH__)
    #error "enum_utils.hpp cannot be used in CUDA code. Use raw enum values instead."
#endif

#include <magic_enum/magic_enum.hpp>
#include <optional>
#include <string_view>

namespace inf::core {

// ============================================================================
// 通用枚举工具
// ============================================================================

/// @brief 枚举转字符串
template <typename E>
[[nodiscard]] constexpr std::string_view enum_name(E value) noexcept {
    return magic_enum::enum_name(value);
}

/// @brief 字符串转枚举 (返回 optional)
template <typename E>
[[nodiscard]] constexpr std::optional<E> enum_cast(std::string_view name) noexcept {
    return magic_enum::enum_cast<E>(name);
}

/// @brief 字符串转枚举 (带默认值)
template <typename E>
[[nodiscard]] constexpr E enum_cast_or(std::string_view name, E default_value) noexcept {
    return magic_enum::enum_cast<E>(name).value_or(default_value);
}

/// @brief 枚举转整数
template <typename E>
[[nodiscard]] constexpr auto enum_integer(E value) noexcept {
    return magic_enum::enum_integer(value);
}

/// @brief 整数转枚举 (返回 optional)
template <typename E, typename I>
[[nodiscard]] constexpr std::optional<E> enum_cast(I value) noexcept {
    return magic_enum::enum_cast<E>(value);
}

/// @brief 获取枚举值数量
template <typename E>
[[nodiscard]] constexpr std::size_t enum_count() noexcept {
    return magic_enum::enum_count<E>();
}

/// @brief 获取所有枚举值
template <typename E>
[[nodiscard]] constexpr auto enum_values() noexcept {
    return magic_enum::enum_values<E>();
}

/// @brief 获取所有枚举名称
template <typename E>
[[nodiscard]] constexpr auto enum_names() noexcept {
    return magic_enum::enum_names<E>();
}

/// @brief 检查值是否为有效枚举
template <typename E, typename I>
[[nodiscard]] constexpr bool enum_contains(I value) noexcept {
    return magic_enum::enum_contains<E>(value);
}

}  // namespace inf::core
