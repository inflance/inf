/// @file io.cpp
#include "io.hpp"
#include <cmath>
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
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
#include <memory>
#include <sstream>

namespace inf::io {

namespace {

// ============================================================================
// 辅助函数
// ============================================================================

/// @brief 创建文件未找到错误
[[nodiscard]] core::Error file_not_found_error(const std::filesystem::path& path) {
    return {core::ErrorCode::kFileNotFound, "File not found: " + path.string()};
}

/// @brief 创建解析错误
[[nodiscard]] core::Error parse_error(const std::string& msg) {
    return {core::ErrorCode::kParseError, msg};
}

[[nodiscard]] core::Error invalid_argument_error(const std::string& msg) {
    return {core::ErrorCode::kInvalidArgument, msg};
}

[[nodiscard]] core::Error unsupported_format_error(const std::filesystem::path& path) {
    return {core::ErrorCode::kUnsupportedFormat, "Unsupported format: " + path.string()};
}

/// @brief 判断行是否为注释或空行
[[nodiscard]] bool is_comment_or_empty(const std::string& line) {
    if (line.empty()) return true;
    for (char c : line) {
        if (c == '#') return true;
        if (!std::isspace(static_cast<unsigned char>(c))) return false;
    }
    return true;
}

/// @brief 小端序读取
template <typename T>
[[nodiscard]] bool read_binary(std::istream& is, T& value) {
    return static_cast<bool>(is.read(reinterpret_cast<char*>(&value), sizeof(T)));
}

/// @brief 读取 null-terminated 字符串
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

[[nodiscard]] std::string file_extension_lower(const std::filesystem::path& path) {
    return to_lower(path.extension().string());
}

struct AssimpSceneDeleter {
    void operator()(aiScene* scene) const {
        if (!scene) return;
        if (scene->mMeshes) {
            for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
                delete scene->mMeshes[i];
            }
            delete[] scene->mMeshes;
        }
        if (scene->mMaterials) {
            for (unsigned int i = 0; i < scene->mNumMaterials; ++i) {
                delete scene->mMaterials[i];
            }
            delete[] scene->mMaterials;
        }
        if (scene->mRootNode) {
            delete[] scene->mRootNode->mMeshes;
            delete scene->mRootNode;
        }
        delete scene;
    }
};

[[nodiscard]] bool has_field(const pcl::PCLPointCloud2& cloud, const std::string& name) {
    return pcl::getFieldIndex(cloud, name) >= 0;
}

[[nodiscard]] uint8_t clamp_color(float value) {
    const float clamped = std::clamp(value, 0.0f, 255.0f);
    return static_cast<uint8_t>(clamped);
}

// ============================================================================
// Text 格式解析
// ============================================================================

/// @brief 解析 cameras.txt
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

/// @brief 解析 images.txt
[[nodiscard]] core::Result<std::unordered_map<uint32_t, core::Image>>
read_images_text(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        return core::unexpected(file_not_found_error(path));
    }

    std::unordered_map<uint32_t, core::Image> images;
    std::string line;

    while (std::getline(file, line)) {
        if (is_comment_or_empty(line)) continue;

        // 第一行: IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        std::istringstream iss(line);
        core::Image img;
        double qw, qx, qy, qz;

        if (!(iss >> img.id >> qw >> qx >> qy >> qz 
                  >> img.translation.x() >> img.translation.y() >> img.translation.z()
                  >> img.camera_id >> img.name)) {
            return core::unexpected(parse_error("Failed to parse image: " + line));
        }

        img.rotation = core::Quatd(qw, qx, qy, qz);

        // 第二行: POINTS2D[] as (X, Y, POINT3D_ID)
        if (!std::getline(file, line)) {
            return core::unexpected(parse_error("Missing points2d line for image: " + std::to_string(img.id)));
        }

        if (!is_comment_or_empty(line)) {
            std::istringstream iss2(line);
            double x, y;
            int64_t point3d_id;
            while (iss2 >> x >> y >> point3d_id) {
                core::Point2D pt;
                pt.xy = core::Vec2d(x, y);
                pt.point3d_id = point3d_id;
                img.points2d.push_back(pt);
            }
        }

        images[img.id] = std::move(img);
    }

    return images;
}

/// @brief 解析 points3D.txt
[[nodiscard]] core::Result<std::unordered_map<uint64_t, core::Point3D>>
read_points3d_text(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        return core::unexpected(file_not_found_error(path));
    }

    std::unordered_map<uint64_t, core::Point3D> points3d;
    std::string line;

    while (std::getline(file, line)) {
        if (is_comment_or_empty(line)) continue;

        // POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
        std::istringstream iss(line);
        core::Point3D pt;
        int r, g, b;

        if (!(iss >> pt.id 
                  >> pt.position.x() >> pt.position.y() >> pt.position.z()
                  >> r >> g >> b >> pt.error)) {
            return core::unexpected(parse_error("Failed to parse point3d: " + line));
        }

        pt.color = core::Vec3f(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b));

        // 解析 track
        uint32_t image_id, point2d_idx;
        while (iss >> image_id >> point2d_idx) {
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

        if (!read_binary(file, cam.id) ||
            !read_binary(file, model_id) ||
            !read_binary(file, cam.width) ||
            !read_binary(file, cam.height)) {
            return core::unexpected(parse_error("Failed to read camera header"));
        }

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

core::Result<mesh::Mesh> read_ply_mesh(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        return core::unexpected(file_not_found_error(path));
    }

    Assimp::Importer importer;
    const unsigned int flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices;
    const aiScene* scene = importer.ReadFile(path.string(), flags);
    if (!scene || !scene->HasMeshes()) {
        return core::unexpected(parse_error("Failed to read PLY mesh: " + std::string(importer.GetErrorString())));
    }

    const aiMesh* ai_mesh = scene->mMeshes[0];
    mesh::Mesh result;
    result.vertices.reserve(ai_mesh->mNumVertices);
    if (ai_mesh->HasNormals()) {
        result.normals.reserve(ai_mesh->mNumVertices);
    }

    for (unsigned int i = 0; i < ai_mesh->mNumVertices; ++i) {
        const auto& v = ai_mesh->mVertices[i];
        result.vertices.emplace_back(v.x, v.y, v.z);
        if (ai_mesh->HasNormals()) {
            const auto& n = ai_mesh->mNormals[i];
            result.normals.emplace_back(n.x, n.y, n.z);
        }
    }

    result.indices.reserve(static_cast<size_t>(ai_mesh->mNumFaces) * 3);
    for (unsigned int i = 0; i < ai_mesh->mNumFaces; ++i) {
        const aiFace& face = ai_mesh->mFaces[i];
        if (face.mNumIndices != 3) {
            return core::unexpected(invalid_argument_error("Non-triangle face in mesh: " + path.string()));
        }
        result.indices.push_back(face.mIndices[0]);
        result.indices.push_back(face.mIndices[1]);
        result.indices.push_back(face.mIndices[2]);
    }

    return result;
}

core::Result<void> write_ply_mesh(
    const std::filesystem::path& path, const mesh::Mesh& mesh, bool binary) {
    if (mesh.indices.size() % 3 != 0) {
        return core::unexpected(invalid_argument_error("Mesh indices must be triangles"));
    }
    if (mesh.vertices.empty()) {
        return core::unexpected(invalid_argument_error("Mesh has no vertices"));
    }

    auto scene = std::unique_ptr<aiScene, AssimpSceneDeleter>(new aiScene());
    scene->mRootNode = new aiNode();
    scene->mRootNode->mNumMeshes = 1;
    scene->mRootNode->mMeshes = new unsigned int[1]{0};

    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh*[1];
    scene->mMeshes[0] = new aiMesh();
    aiMesh* ai_mesh = scene->mMeshes[0];

    ai_mesh->mMaterialIndex = 0;
    ai_mesh->mNumVertices = static_cast<unsigned int>(mesh.vertices.size());
    ai_mesh->mVertices = new aiVector3D[ai_mesh->mNumVertices];

    const bool has_normals = mesh.normals.size() == mesh.vertices.size();
    if (has_normals) {
        ai_mesh->mNormals = new aiVector3D[ai_mesh->mNumVertices];
    }

    for (unsigned int i = 0; i < ai_mesh->mNumVertices; ++i) {
        const auto& v = mesh.vertices[i];
        ai_mesh->mVertices[i] = aiVector3D(v.x(), v.y(), v.z());
        if (has_normals) {
            const auto& n = mesh.normals[i];
            ai_mesh->mNormals[i] = aiVector3D(n.x(), n.y(), n.z());
        }
    }

    ai_mesh->mNumFaces = static_cast<unsigned int>(mesh.indices.size() / 3);
    ai_mesh->mFaces = new aiFace[ai_mesh->mNumFaces];
    for (unsigned int i = 0; i < ai_mesh->mNumFaces; ++i) {
        aiFace& face = ai_mesh->mFaces[i];
        face.mNumIndices = 3;
        face.mIndices = new unsigned int[3];
        const size_t base = static_cast<size_t>(i) * 3;
        face.mIndices[0] = mesh.indices[base];
        face.mIndices[1] = mesh.indices[base + 1];
        face.mIndices[2] = mesh.indices[base + 2];
    }

    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial*[1];
    scene->mMaterials[0] = new aiMaterial();

    Assimp::Exporter exporter;
    const char* format_id = binary ? "plyb" : "ply";
    if (exporter.Export(scene.get(), format_id, path.string()) != AI_SUCCESS) {
        if (!binary || exporter.Export(scene.get(), "ply", path.string()) != AI_SUCCESS) {
            return core::unexpected(parse_error("Failed to write PLY mesh: " + std::string(exporter.GetErrorString())));
        }
    }

    return {};
}

core::Result<pc::PointCloud> read_point_cloud(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        return core::unexpected(file_not_found_error(path));
    }

    const std::string ext = file_extension_lower(path);
    pcl::PCLPointCloud2 cloud_blob;
    int load_result = -1;

    if (ext == ".ply") {
        load_result = pcl::io::loadPLYFile(path.string(), cloud_blob);
    } else if (ext == ".pcd") {
        load_result = pcl::io::loadPCDFile(path.string(), cloud_blob);
    } else {
        return core::unexpected(unsupported_format_error(path));
    }

    if (load_result < 0) {
        return core::unexpected(parse_error("Failed to read point cloud: " + path.string()));
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
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        result.positions.emplace_back(pt.x, pt.y, pt.z);
        if (has_normals) {
            result.normals.emplace_back(pt.normal_x, pt.normal_y, pt.normal_z);
        }
        if (has_rgb) {
            result.colors.emplace_back(static_cast<float>(pt.r),
                                       static_cast<float>(pt.g),
                                       static_cast<float>(pt.b));
        }
    }

    return result;
}

core::Result<void> write_point_cloud(
    const std::filesystem::path& path, const pc::PointCloud& cloud, bool binary) {
    const std::string ext = file_extension_lower(path);
    if (cloud.positions.empty()) {
        return core::unexpected(invalid_argument_error("Point cloud has no points"));
    }

    const bool has_colors = !cloud.colors.empty();
    const bool has_normals = !cloud.normals.empty();
    if (has_colors && cloud.colors.size() != cloud.positions.size()) {
        return core::unexpected(invalid_argument_error("Point cloud colors size mismatch"));
    }
    if (has_normals && cloud.normals.size() != cloud.positions.size()) {
        return core::unexpected(invalid_argument_error("Point cloud normals size mismatch"));
    }

    int save_result = -1;
    if (has_colors && has_normals) {
        pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_cloud;
        pcl_cloud.points.reserve(cloud.positions.size());
        for (size_t i = 0; i < cloud.positions.size(); ++i) {
            pcl::PointXYZRGBNormal pt;
            const auto& p = cloud.positions[i];
            const auto& n = cloud.normals[i];
            const auto& c = cloud.colors[i];
            pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
            pt.normal_x = n.x(); pt.normal_y = n.y(); pt.normal_z = n.z();
            pt.r = clamp_color(c.x());
            pt.g = clamp_color(c.y());
            pt.b = clamp_color(c.z());
            pcl_cloud.points.push_back(pt);
        }
        pcl_cloud.width = static_cast<uint32_t>(pcl_cloud.points.size());
        pcl_cloud.height = 1;

        if (ext == ".ply") {
            save_result = binary ? pcl::io::savePLYFileBinary(path.string(), pcl_cloud)
                                 : pcl::io::savePLYFileASCII(path.string(), pcl_cloud);
        } else if (ext == ".pcd") {
            save_result = binary ? pcl::io::savePCDFileBinary(path.string(), pcl_cloud)
                                 : pcl::io::savePCDFileASCII(path.string(), pcl_cloud);
        } else {
            return core::unexpected(unsupported_format_error(path));
        }
    } else if (has_colors) {
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl_cloud.points.reserve(cloud.positions.size());
        for (size_t i = 0; i < cloud.positions.size(); ++i) {
            pcl::PointXYZRGB pt;
            const auto& p = cloud.positions[i];
            const auto& c = cloud.colors[i];
            pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
            pt.r = clamp_color(c.x());
            pt.g = clamp_color(c.y());
            pt.b = clamp_color(c.z());
            pcl_cloud.points.push_back(pt);
        }
        pcl_cloud.width = static_cast<uint32_t>(pcl_cloud.points.size());
        pcl_cloud.height = 1;

        if (ext == ".ply") {
            save_result = binary ? pcl::io::savePLYFileBinary(path.string(), pcl_cloud)
                                 : pcl::io::savePLYFileASCII(path.string(), pcl_cloud);
        } else if (ext == ".pcd") {
            save_result = binary ? pcl::io::savePCDFileBinary(path.string(), pcl_cloud)
                                 : pcl::io::savePCDFileASCII(path.string(), pcl_cloud);
        } else {
            return core::unexpected(unsupported_format_error(path));
        }
    } else if (has_normals) {
        pcl::PointCloud<pcl::PointNormal> pcl_cloud;
        pcl_cloud.points.reserve(cloud.positions.size());
        for (size_t i = 0; i < cloud.positions.size(); ++i) {
            pcl::PointNormal pt;
            const auto& p = cloud.positions[i];
            const auto& n = cloud.normals[i];
            pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
            pt.normal_x = n.x(); pt.normal_y = n.y(); pt.normal_z = n.z();
            pcl_cloud.points.push_back(pt);
        }
        pcl_cloud.width = static_cast<uint32_t>(pcl_cloud.points.size());
        pcl_cloud.height = 1;

        if (ext == ".ply") {
            save_result = binary ? pcl::io::savePLYFileBinary(path.string(), pcl_cloud)
                                 : pcl::io::savePLYFileASCII(path.string(), pcl_cloud);
        } else if (ext == ".pcd") {
            save_result = binary ? pcl::io::savePCDFileBinary(path.string(), pcl_cloud)
                                 : pcl::io::savePCDFileASCII(path.string(), pcl_cloud);
        } else {
            return core::unexpected(unsupported_format_error(path));
        }
    } else {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.points.reserve(cloud.positions.size());
        for (const auto& p : cloud.positions) {
            pcl::PointXYZ pt;
            pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
            pcl_cloud.points.push_back(pt);
        }
        pcl_cloud.width = static_cast<uint32_t>(pcl_cloud.points.size());
        pcl_cloud.height = 1;

        if (ext == ".ply") {
            save_result = binary ? pcl::io::savePLYFileBinary(path.string(), pcl_cloud)
                                 : pcl::io::savePLYFileASCII(path.string(), pcl_cloud);
        } else if (ext == ".pcd") {
            save_result = binary ? pcl::io::savePCDFileBinary(path.string(), pcl_cloud)
                                 : pcl::io::savePCDFileASCII(path.string(), pcl_cloud);
        } else {
            return core::unexpected(unsupported_format_error(path));
        }
    }

    if (save_result < 0) {
        return core::unexpected(parse_error("Failed to write point cloud: " + path.string()));
    }

    return {};
}

}  // namespace inf::io
