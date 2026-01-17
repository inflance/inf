#pragma once
/// @file ply.hpp
/// @brief PLY 文件格式读写类

#include "InfCore.hpp"
#include "InfMesh.hpp"
#include "InfPointCloud.hpp"
#include <filesystem>
#include <string>
#include <vector>
#include <optional>

namespace inf::io {


enum class PlyFormat : uint8_t {
    Ascii = 0,
    BinaryLittleEndian,
    BinaryBigEndian,
};

enum class PlyType : uint8_t {
    Char, UChar,
    Short, UShort,
    Int, UInt,
    Float, Double,
    Invalid,
};

struct PlyProperty {
    std::string name;
    PlyType type = PlyType::Invalid;
    bool is_list = false;
    PlyType count_type = PlyType::Invalid;  // 列表计数类型
};

struct PlyElement {
    std::string name;
    size_t count = 0;
    std::vector<PlyProperty> properties;

    [[nodiscard]] bool has_property(std::string_view prop_name) const;
    [[nodiscard]] const PlyProperty* find_property(std::string_view prop_name) const;
};

struct PlyHeader {
    PlyFormat format = PlyFormat::Ascii;
    std::vector<PlyElement> elements;
    std::vector<std::string> comments;

    [[nodiscard]] const PlyElement* find_element(std::string_view name) const;
    [[nodiscard]] size_t vertex_count() const;
    [[nodiscard]] size_t face_count() const;
    [[nodiscard]] bool has_normals() const;
    [[nodiscard]] bool has_colors() const;
    [[nodiscard]] bool is_binary() const { return format != PlyFormat::Ascii; }
};

class PlyFile {
public:
    PlyFile() = default;

    [[nodiscard]] static core::Result<PlyHeader> read_header(const std::filesystem::path& path);

    [[nodiscard]] static core::Result<mesh::Mesh> read_mesh(const std::filesystem::path& path);

    [[nodiscard]] static core::Result<pc::PointCloud> read_pointcloud(const std::filesystem::path& path);

    [[nodiscard]] static core::Result<void> write_mesh(
        const std::filesystem::path& path,
        const mesh::Mesh& mesh,
        bool binary = true);

    [[nodiscard]] static core::Result<void> write_pointcloud(
        const std::filesystem::path& path,
        const pc::PointCloud& cloud,
        bool binary = true);

    [[nodiscard]] static size_t type_size(PlyType type);

    [[nodiscard]] static PlyType type_from_string(std::string_view str);
};

// ============================================================================
// 便捷类型别名
// ============================================================================

using Ply = PlyFile;

}  // namespace inf::io
