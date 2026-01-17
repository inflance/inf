/// @file pipeline.cpp
#include "pipeline.hpp"

namespace inf::pipeline {

core::Result<PipelineResult> run(
    const std::filesystem::path& colmap_dir, const PipelineConfig& config) {
    // TODO: 实现完整流水线
    (void)colmap_dir; (void)config;
    return core::unexpected(core::Error{core::ErrorCode::UnsupportedFormat, "Not implemented"});
}

}  // namespace inf::pipeline
