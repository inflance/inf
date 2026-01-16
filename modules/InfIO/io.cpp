/// @file io.cpp
#include "io.hpp"

namespace inf::io {

core::Result<ColmapModel> read_colmap_text(const std::filesystem::path& model_dir) {
    // TODO: 实现 COLMAP Text 格式读取
    (void)model_dir;
    return std::unexpected(core::Error{core::ErrorCode::kUnsupportedFormat, "Not implemented"});
}

}  // namespace inf::io
