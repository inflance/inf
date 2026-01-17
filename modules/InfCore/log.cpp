/// @file log.cpp
#include "log.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>

namespace inf::core {

std::shared_ptr<spdlog::logger> Log::logger_ = nullptr;

namespace {

spdlog::level::level_enum to_spdlog_level(LogLevel level) {
    switch (level) {
        case LogLevel::kTrace:    return spdlog::level::trace;
        case LogLevel::kDebug:    return spdlog::level::debug;
        case LogLevel::kInfo:     return spdlog::level::info;
        case LogLevel::kWarn:     return spdlog::level::warn;
        case LogLevel::kError:    return spdlog::level::err;
        case LogLevel::kCritical: return spdlog::level::critical;
        case LogLevel::kOff:      return spdlog::level::off;
        default:                  return spdlog::level::info;
    }
}

LogLevel from_spdlog_level(spdlog::level::level_enum level) {
    switch (level) {
        case spdlog::level::trace:    return LogLevel::kTrace;
        case spdlog::level::debug:    return LogLevel::kDebug;
        case spdlog::level::info:     return LogLevel::kInfo;
        case spdlog::level::warn:     return LogLevel::kWarn;
        case spdlog::level::err:      return LogLevel::kError;
        case spdlog::level::critical: return LogLevel::kCritical;
        case spdlog::level::off:      return LogLevel::kOff;
        default:                      return LogLevel::kInfo;
    }
}

}  // namespace

void Log::init(std::string_view name, LogLevel level, std::string_view pattern) {
    // 创建彩色控制台 sink
    logger_ = spdlog::stdout_color_mt(std::string(name));
    logger_->set_level(to_spdlog_level(level));
    logger_->set_pattern(std::string(pattern));
}

void Log::set_level(LogLevel level) {
    logger()->set_level(to_spdlog_level(level));
}

LogLevel Log::level() {
    return from_spdlog_level(logger()->level());
}

std::shared_ptr<spdlog::logger>& Log::logger() {
    if (!logger_) {
        // 懒初始化
        init();
    }
    return logger_;
}

}  // namespace inf::core
