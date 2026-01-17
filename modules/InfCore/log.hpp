#pragma once
/// @file log.hpp
/// @brief InfMVS 日志系统 (基于 spdlog)

#include "core.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <memory>

namespace inf::core {

// ============================================================================
// 日志级别
// ============================================================================
enum class LogLevel : uint8_t {
    kTrace = 0,
    kDebug,
    kInfo,
    kWarn,
    kError,
    kCritical,
    kOff,
};

// ============================================================================
// 日志类
// ============================================================================
class INFCORE_API Log {
public:
    /// @brief 初始化日志系统
    /// @param name 日志器名称
    /// @param level 日志级别
    /// @param pattern 日志格式 (spdlog pattern)
    static void init(
        std::string_view name = "InfMVS",
        LogLevel level = LogLevel::kInfo,
        std::string_view pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

    /// @brief 设置日志级别
    static void set_level(LogLevel level);

    /// @brief 获取日志级别
    [[nodiscard]] static LogLevel level();

    /// @brief 获取 spdlog logger (高级用法)
    [[nodiscard]] static std::shared_ptr<spdlog::logger>& logger();

    // 便捷日志函数
    template <typename... Args>
    static void trace(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        logger()->trace(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void debug(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        logger()->debug(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void info(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        logger()->info(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void warn(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        logger()->warn(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void error(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        logger()->error(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void critical(spdlog::format_string_t<Args...> fmt, Args&&... args) {
        logger()->critical(fmt, std::forward<Args>(args)...);
    }

private:
    static std::shared_ptr<spdlog::logger> logger_;
};

// ============================================================================
// 便捷宏 (可选，带源码位置)
// ============================================================================
#define INF_TRACE(...)    ::inf::core::Log::trace(__VA_ARGS__)
#define INF_DEBUG(...)    ::inf::core::Log::debug(__VA_ARGS__)
#define INF_INFO(...)     ::inf::core::Log::info(__VA_ARGS__)
#define INF_WARN(...)     ::inf::core::Log::warn(__VA_ARGS__)
#define INF_ERROR(...)    ::inf::core::Log::error(__VA_ARGS__)
#define INF_CRITICAL(...) ::inf::core::Log::critical(__VA_ARGS__)

}  // namespace inf::core
