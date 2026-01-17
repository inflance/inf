/// @file io.cpp
#include "io.hpp"
#include "ply.hpp"
#include "macro.hpp"

#include <cmath>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <charconv>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>

namespace inf::io {

namespace {

[[nodiscard]] core::Error file_not_found_error(const std::filesystem::path& path) {
    return {core::ErrorCode::kFileNotFound, "File not found: " + path.string()};
}

[[nodiscard]] core::Error parse_error(const std::string& msg) {
    return {core::ErrorCode::kParseError, msg};
}

[[nodiscard]] core::Error invalid_argument_error(const std::string& msg) {
    return {core::ErrorCode::kInvalidArgument, msg};
}

[[nodiscard]] core::Error unsupported_format_error(const std::filesystem::path& path) {
    return {core::ErrorCode::kUnsupportedFormat, "Unsupported format: " + path.string()};
}

[[nodiscard]] bool is_comment_or_empty(const std::string& line) {
    if (line.empty()) return true;
    for (char c : line) {
        if (c == '#') return true;
        if (!std::isspace(static_cast<unsigned char>(c))) return false;
    }
    return true;
}

template <typename T>
[[nodiscard]] bool read_binary(std::istream& is, T& value) {
    return static_cast<bool>(is.read(reinterpret_cast<char*>(&value), sizeof(T)));
}

[[nodiscard]] bool read_binary_string(std::istream& is, std::string& str) {
    str.clear();
    char c;
    while (is.get(c) && c != '\0') {
        str.push_back(c);
    }
    return is.good() || is.eof();
}

[[nodiscard]] std::string to_lower(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

[[nodiscard]] const char* skip_whitespace(const char* p, const char* end) {
    while (p < end && (*p == ' ' || *p == '\t')) ++p;
    return p;
}

template <typename T>
[[nodiscard]] const char* parse_int(const char* p, const char* end, T& value) {
    p = skip_whitespace(p, end);
    auto result = std::from_chars(p, end, value);
    return (result.ec == std::errc{}) ? result.ptr : nullptr;
}

[[nodiscard]] const char* parse_double(const char* p, const char* end, double& value) {
    p = skip_whitespace(p, end);
    // std::from_chars for double 在某些编译器上不支持，使用 strtod
    char* endptr = nullptr;
    value = std::strtod(p, &endptr);
    return (endptr > p) ? endptr : nullptr;
}

[[nodiscard]] const char* parse_string(const char* p, const char* end, std::string& value) {
    p = skip_whitespace(p, end);
    const char* start = p;
    while (p < end && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') ++p;
    value.assign(start, p);
    return p;
}

[[nodiscard]] std::string file_extension_lower(const std::filesystem::path& path) {
    return to_lower(path.extension().string());
}

[[nodiscard]] bool has_field(const pcl::PCLPointCloud2& cloud, const std::string& name) {
    return pcl::getFieldIndex(cloud, name) >= 0;
}

[[nodiscard]] uint8_t clamp_color(float value) {
    const float clamped = std::clamp(value, 0.0f, 255.0f);
    return static_cast<uint8_t>(clamped);
}

[[nodiscard]] core::Result<std::unordered_map<uint32_t, core::Camera>>
read_cameras_text(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        return core::unexpected(file_not_found_error(path));
    }

    std::unordered_map<uint32_t, core::Camera> cameras;
    std::string line;

    while (std::getline(file, line)) {
        if (is_comment_or_empty(line)) continue;

        std::istringstream iss(line);
        core::Camera cam;
        std::string model_name;

        if (!(iss >> cam.id >> model_name >> cam.width >> cam.height)) {
            return core::unexpected(parse_error("Failed to parse camera: " + line));
        }

        cam.model = core::camera_model_from_string(model_name);
        const auto num_params = core::camera_model_num_params(cam.model);
        cam.params.resize(num_params);

        for (uint32_t i = 0; i < num_params; ++i) {
            if (!(iss >> cam.params[i])) {
                return core::unexpected(parse_error("Failed to parse camera params: " + line));
            }
        }

        cameras[cam.id] = std::move(cam);
    }

    return cameras;
}

[[nodiscard]] core::Result<std::unordered_map<uint32_t, core::Image>>
read_images_text(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        return core::unexpected(file_not_found_error(path));
    }

    std::unordered_map<uint32_t, core::Image> images;
    images.reserve(1000);  // 预分配
    std::string line;
    line.reserve(4096);

    while (std::getline(file, line)) {
        if (is_comment_or_empty(line)) continue;

        const char* p = line.data();
        const char* end = p + line.size();
        core::Image img;
        double qw, qx, qy, qz, tx, ty, tz;

        // IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        if (!(p = parse_int(p, end, img.id))) continue;
        if (!(p = parse_double(p, end, qw))) continue;
        if (!(p = parse_double(p, end, qx))) continue;
        if (!(p = parse_double(p, end, qy))) continue;
        if (!(p = parse_double(p, end, qz))) continue;
        if (!(p = parse_double(p, end, tx))) continue;
        if (!(p = parse_double(p, end, ty))) continue;
        if (!(p = parse_double(p, end, tz))) continue;
        if (!(p = parse_int(p, end, img.camera_id))) continue;
        if (!(p = parse_string(p, end, img.name))) continue;

        img.rotation = core::Quatd(qw, qx, qy, qz);
        img.translation = core::Vec3d(tx, ty, tz);

        // 第二行: POINTS2D[] as (X, Y, POINT3D_ID)
        if (!std::getline(file, line)) {
            return core::unexpected(parse_error("Missing points2d line for image: " + std::to_string(img.id)));
        }

        if (!is_comment_or_empty(line)) {
            p = line.data();
            end = p + line.size();
            img.points2d.reserve(500);  // 预分配
            double x, y;
            int64_t point3d_id;
            while ((p = parse_double(p, end, x)) && 
                   (p = parse_double(p, end, y)) && 
                   (p = parse_int(p, end, point3d_id))) {
                img.points2d.push_back({core::Vec2d(x, y), point3d_id});
            }
        }

        images[img.id] = std::move(img);
    }

    return images;
}

/// @brief 解析 points3D.txt (优化版)
[[nodiscard]] core::Result<std::unordered_map<uint64_t, core::Point3D>>
read_points3d_text(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        return core::unexpected(file_not_found_error(path));
    }

    std::unordered_map<uint64_t, core::Point3D> points3d;
    points3d.reserve(10000);  // 预分配
    std::string line;
    line.reserve(1024);

    while (std::getline(file, line)) {
        if (is_comment_or_empty(line)) continue;

        const char* p = line.data();
        const char* end = p + line.size();
        core::Point3D pt;
        double x, y, z;
        int r, g, b;

        // POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[]
        if (!(p = parse_int(p, end, pt.id))) continue;
        if (!(p = parse_double(p, end, x))) continue;
        if (!(p = parse_double(p, end, y))) continue;
        if (!(p = parse_double(p, end, z))) continue;
        if (!(p = parse_int(p, end, r))) continue;
        if (!(p = parse_int(p, end, g))) continue;
        if (!(p = parse_int(p, end, b))) continue;
        if (!(p = parse_double(p, end, pt.error))) continue;

        pt.position = core::Vec3d(x, y, z);
        pt.color = core::Vec3f(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b));

        // 解析 track
        pt.track.reserve(8);
        uint32_t image_id, point2d_idx;
        while ((p = parse_int(p, end, image_id)) && (p = parse_int(p, end, point2d_idx))) {
            pt.track.push_back({image_id, point2d_idx});
        }

        points3d[pt.id] = std::move(pt);
    }

    return points3d;
}

// ============================================================================
// Binary 格式解析
// ============================================================================

/// @brief 解析 cameras.bin
[[nodiscard]] core::Result<std::unordered_map<uint32_t, core::Camera>>
read_cameras_binary(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return core::unexpected(file_not_found_error(path));
    }

    uint64_t num_cameras;
    if (!read_binary(file, num_cameras)) {
        return core::unexpected(parse_error("Failed to read num_cameras"));
    }

    std::unordered_map<uint32_t, core::Camera> cameras;
    cameras.reserve(static_cast<size_t>(num_cameras));

    for (uint64_t i = 0; i < num_cameras; ++i) {
        core::Camera cam;
        int32_t model_id;
        uint64_t width, height;  // COLMAP binary 用 uint64_t

        if (!read_binary(file, cam.id) ||
            !read_binary(file, model_id) ||
            !read_binary(file, width) ||
            !read_binary(file, height)) {
            return core::unexpected(parse_error("Failed to read camera header"));
        }

        cam.width = static_cast<uint32_t>(width);
        cam.height = static_cast<uint32_t>(height);
        cam.model = static_cast<core::CameraModel>(model_id);
        const auto num_params = core::camera_model_num_params(cam.model);
        cam.params.resize(num_params);

        for (uint32_t j = 0; j < num_params; ++j) {
            if (!read_binary(file, cam.params[j])) {
                return core::unexpected(parse_error("Failed to read camera params"));
            }
        }

        cameras[cam.id] = std::move(cam);
    }

    return cameras;
}

/// @brief 解析 images.bin
[[nodiscard]] core::Result<std::unordered_map<uint32_t, core::Image>>
read_images_binary(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return core::unexpected(file_not_found_error(path));
    }

    uint64_t num_images;
    if (!read_binary(file, num_images)) {
        return core::unexpected(parse_error("Failed to read num_images"));
    }

    std::unordered_map<uint32_t, core::Image> images;
    images.reserve(static_cast<size_t>(num_images));

    for (uint64_t i = 0; i < num_images; ++i) {
        core::Image img;
        double qw, qx, qy, qz, tx, ty, tz;

        if (!read_binary(file, img.id) ||
            !read_binary(file, qw) ||
            !read_binary(file, qx) ||
            !read_binary(file, qy) ||
            !read_binary(file, qz) ||
            !read_binary(file, tx) ||
            !read_binary(file, ty) ||
            !read_binary(file, tz) ||
            !read_binary(file, img.camera_id)) {
            return core::unexpected(parse_error("Failed to read image header"));
        }

        img.rotation = core::Quatd(qw, qx, qy, qz);
        img.translation = core::Vec3d(tx, ty, tz);

        if (!read_binary_string(file, img.name)) {
            return core::unexpected(parse_error("Failed to read image name"));
        }

        uint64_t num_points2d;
        if (!read_binary(file, num_points2d)) {
            return core::unexpected(parse_error("Failed to read num_points2d"));
        }

        img.points2d.reserve(static_cast<size_t>(num_points2d));
        for (uint64_t j = 0; j < num_points2d; ++j) {
            core::Point2D pt;
            double x, y;
            uint64_t point3d_id;

            if (!read_binary(file, x) ||
                !read_binary(file, y) ||
                !read_binary(file, point3d_id)) {
                return core::unexpected(parse_error("Failed to read point2d"));
            }

            pt.xy = core::Vec2d(x, y);
            // COLMAP 使用 max uint64_t 表示无效 ID，转换为 -1
            pt.point3d_id = (point3d_id == std::numeric_limits<uint64_t>::max()) 
                          ? -1 
                          : static_cast<int64_t>(point3d_id);
            img.points2d.push_back(pt);
        }

        images[img.id] = std::move(img);
    }

    return images;
}

/// @brief 解析 points3D.bin
[[nodiscard]] core::Result<std::unordered_map<uint64_t, core::Point3D>>
read_points3d_binary(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return core::unexpected(file_not_found_error(path));
    }

    uint64_t num_points3d;
    if (!read_binary(file, num_points3d)) {
        return core::unexpected(parse_error("Failed to read num_points3d"));
    }

    std::unordered_map<uint64_t, core::Point3D> points3d;
    points3d.reserve(static_cast<size_t>(num_points3d));

    for (uint64_t i = 0; i < num_points3d; ++i) {
        core::Point3D pt;
        double x, y, z;
        uint8_t r, g, b;

        if (!read_binary(file, pt.id) ||
            !read_binary(file, x) ||
            !read_binary(file, y) ||
            !read_binary(file, z) ||
            !read_binary(file, r) ||
            !read_binary(file, g) ||
            !read_binary(file, b) ||
            !read_binary(file, pt.error)) {
            return core::unexpected(parse_error("Failed to read point3d header"));
        }

        pt.position = core::Vec3d(x, y, z);
        pt.color = core::Vec3f(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b));

        uint64_t track_length;
        if (!read_binary(file, track_length)) {
            return core::unexpected(parse_error("Failed to read track_length"));
        }

        pt.track.reserve(static_cast<size_t>(track_length));
        for (uint64_t j = 0; j < track_length; ++j) {
            core::TrackElement elem;
            if (!read_binary(file, elem.image_id) ||
                !read_binary(file, elem.point2d_idx)) {
                return core::unexpected(parse_error("Failed to read track element"));
            }
            pt.track.push_back(elem);
        }

        points3d[pt.id] = std::move(pt);
    }

    return points3d;
}

}  // namespace

// ============================================================================
// 公共 API 实现
// ============================================================================

core::Result<core::SparseModel> read_colmap_text(const std::filesystem::path& model_dir) {
    namespace fs = std::filesystem;

    const auto cameras_path  = model_dir / "cameras.txt";
    const auto images_path   = model_dir / "images.txt";
    const auto points3d_path = model_dir / "points3D.txt";

    // 解析 cameras
    auto cameras_result = read_cameras_text(cameras_path);
    if (!cameras_result) {
        return core::unexpected(cameras_result.error());
    }

    // 解析 images
    auto images_result = read_images_text(images_path);
    if (!images_result) {
        return core::unexpected(images_result.error());
    }

    // 解析 points3D
    auto points3d_result = read_points3d_text(points3d_path);
    if (!points3d_result) {
        return core::unexpected(points3d_result.error());
    }

    core::SparseModel model;
    model.cameras  = std::move(*cameras_result);
    model.images   = std::move(*images_result);
    model.points3d = std::move(*points3d_result);

    return model;
}

core::Result<core::SparseModel> read_colmap_binary(const std::filesystem::path& model_dir) {
    namespace fs = std::filesystem;

    const auto cameras_path  = model_dir / "cameras.bin";
    const auto images_path   = model_dir / "images.bin";
    const auto points3d_path = model_dir / "points3D.bin";

    // 解析 cameras
    auto cameras_result = read_cameras_binary(cameras_path);
    if (!cameras_result) {
        return core::unexpected(cameras_result.error());
    }

    // 解析 images
    auto images_result = read_images_binary(images_path);
    if (!images_result) {
        return core::unexpected(images_result.error());
    }

    // 解析 points3D
    auto points3d_result = read_points3d_binary(points3d_path);
    if (!points3d_result) {
        return core::unexpected(points3d_result.error());
    }

    core::SparseModel model;
    model.cameras  = std::move(*cameras_result);
    model.images   = std::move(*images_result);
    model.points3d = std::move(*points3d_result);

    return model;
}

core::Result<core::SparseModel> read_colmap(const std::filesystem::path& model_dir) {
    namespace fs = std::filesystem;

    // 优先尝试 binary 格式
    if (fs::exists(model_dir / "cameras.bin") &&
        fs::exists(model_dir / "images.bin") &&
        fs::exists(model_dir / "points3D.bin")) {
        return read_colmap_binary(model_dir);
    }

    // 回退到 text 格式
    if (fs::exists(model_dir / "cameras.txt") &&
        fs::exists(model_dir / "images.txt") &&
        fs::exists(model_dir / "points3D.txt")) {
        return read_colmap_text(model_dir);
    }

    return core::unexpected(core::Error{
        core::ErrorCode::kFileNotFound,
        "No valid COLMAP model found in: " + model_dir.string()
    });
}

IoFormat format_from_extension(const std::filesystem::path& path) {
    const auto ext = file_extension_lower(path);
    if (ext == ".ply") {
        return IoFormat::Mesh;
    }
    if (ext == ".pcd") {
        return IoFormat::PointCloud;
    }
    return IoFormat::Unknown;
}

IoFormat format_from_path(const std::filesystem::path& path) {
    namespace fs = std::filesystem;
    if (fs::is_directory(path)) {
        const bool has_bin = fs::exists(path / "cameras.bin") &&
                             fs::exists(path / "images.bin") &&
                             fs::exists(path / "points3D.bin");
        const bool has_txt = fs::exists(path / "cameras.txt") &&
                             fs::exists(path / "images.txt") &&
                             fs::exists(path / "points3D.txt");
        return (has_bin || has_txt) ? IoFormat::SparseModel : IoFormat::Unknown;
    }
    return format_from_extension(path);
}

// ============================================================================
// PLY I/O (委托给 PlyFile 类)
// ============================================================================

core::Result<mesh::Mesh> read_ply_mesh(const std::filesystem::path& path) {
    return PlyFile::read_mesh(path);
}

core::Result<void> write_ply_mesh(
    const std::filesystem::path& path, const mesh::Mesh& mesh, bool binary) {
    return PlyFile::write_mesh(path, mesh, binary);
}

// ============================================================================
// Point Cloud I/O
// ============================================================================

core::Result<pc::PointCloud> read_point_cloud(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        return core::unexpected(file_not_found_error(path));
    }

    const std::string ext = file_extension_lower(path);

    // PLY: 使用 PlyFile 类
    if (ext == ".ply") {
        return PlyFile::read_pointcloud(path);
    }

    // PCD: 使用 PCL
    if (ext == ".pcd") {
        pcl::PCLPointCloud2 cloud_blob;
        if (pcl::io::loadPCDFile(path.string(), cloud_blob) < 0) {
            return core::unexpected(parse_error("Failed to read PCD: " + path.string()));
        }

        const bool has_rgb = has_field(cloud_blob, "rgb") || has_field(cloud_blob, "rgba");
        const bool has_normals = has_field(cloud_blob, "normal_x");

        pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_cloud;
        pcl::fromPCLPointCloud2(cloud_blob, pcl_cloud);

        pc::PointCloud result;
        result.positions.reserve(pcl_cloud.points.size());
        if (has_normals) result.normals.reserve(pcl_cloud.points.size());
        if (has_rgb) result.colors.reserve(pcl_cloud.points.size());

        for (const auto& pt : pcl_cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            result.positions.emplace_back(pt.x, pt.y, pt.z);
            if (has_normals) result.normals.emplace_back(pt.normal_x, pt.normal_y, pt.normal_z);
            if (has_rgb) result.colors.emplace_back(static_cast<float>(pt.r), static_cast<float>(pt.g), static_cast<float>(pt.b));
        }
        return result;
    }

    return core::unexpected(unsupported_format_error(path));
}

core::Result<void> write_point_cloud(
    const std::filesystem::path& path, const pc::PointCloud& cloud, bool binary) {
    const std::string ext = file_extension_lower(path);

    // PLY: 使用 PlyFile 类
    if (ext == ".ply") {
        return PlyFile::write_pointcloud(path, cloud, binary);
    }

    // PCD: 使用 PCL
    if (ext == ".pcd") {
        if (cloud.positions.empty()) {
            return core::unexpected(invalid_argument_error("Point cloud has no points"));
        }

        const bool has_colors = !cloud.colors.empty();
        const bool has_normals = !cloud.normals.empty();

        pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_cloud;
        pcl_cloud.points.resize(cloud.positions.size());
        pcl_cloud.width = static_cast<uint32_t>(cloud.positions.size());
        pcl_cloud.height = 1;

        INFMVS_OMP_PARALLEL_FOR
        for (int64_t i = 0; i < static_cast<int64_t>(cloud.positions.size()); ++i) {
            auto& pt = pcl_cloud.points[i];
            const auto& p = cloud.positions[i];
            pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
            if (has_normals) {
                const auto& n = cloud.normals[i];
                pt.normal_x = n.x(); pt.normal_y = n.y(); pt.normal_z = n.z();
            }
            if (has_colors) {
                const auto& c = cloud.colors[i];
                pt.r = clamp_color(c.x()); pt.g = clamp_color(c.y()); pt.b = clamp_color(c.z());
            }
        }

        int result = binary ? pcl::io::savePCDFileBinary(path.string(), pcl_cloud)
                            : pcl::io::savePCDFileASCII(path.string(), pcl_cloud);
        if (result < 0) {
            return core::unexpected(parse_error("Failed to write PCD: " + path.string()));
        }
        return {};
    }

    return core::unexpected(unsupported_format_error(path));
}

}  // namespace inf::io
