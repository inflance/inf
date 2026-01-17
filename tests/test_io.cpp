/// @file test_io.cpp
/// @brief InfIO 单元测试
#include "InfIO.hpp"
#include <gtest/gtest.h>
#include <filesystem>

namespace {

// testdata 目录路径 (相对于构建目录)
const std::filesystem::path kTestDataDir = TESTDATA_DIR;

}  // namespace

// ============================================================================
// CameraModel 测试
// ============================================================================

TEST(InfCore, CameraModelFromString) {
    using inf::core::CameraModel;
    using inf::core::camera_model_from_string;
    
    EXPECT_EQ(camera_model_from_string("SIMPLE_PINHOLE"), CameraModel::SIMPLE_PINHOLE);
    EXPECT_EQ(camera_model_from_string("PINHOLE"), CameraModel::PINHOLE);
    EXPECT_EQ(camera_model_from_string("SIMPLE_RADIAL"), CameraModel::SIMPLE_RADIAL);
    EXPECT_EQ(camera_model_from_string("RADIAL"), CameraModel::RADIAL);
    EXPECT_EQ(camera_model_from_string("OPENCV"), CameraModel::OPENCV);
    EXPECT_EQ(camera_model_from_string("OPENCV_FISHEYE"), CameraModel::OPENCV_FISHEYE);
    EXPECT_EQ(camera_model_from_string("UNKNOWN_MODEL"), CameraModel::PINHOLE);  // 默认
}

TEST(InfCore, CameraModelToString) {
    using inf::core::CameraModel;
    using inf::core::camera_model_to_string;
    
    EXPECT_EQ(camera_model_to_string(CameraModel::SIMPLE_PINHOLE), "SIMPLE_PINHOLE");
    EXPECT_EQ(camera_model_to_string(CameraModel::PINHOLE), "PINHOLE");
    EXPECT_EQ(camera_model_to_string(CameraModel::OPENCV), "OPENCV");
}

TEST(InfCore, CameraModelNumParams) {
    using inf::core::CameraModel;
    using inf::core::camera_model_num_params;
    
    EXPECT_EQ(camera_model_num_params(CameraModel::SIMPLE_PINHOLE), 3);
    EXPECT_EQ(camera_model_num_params(CameraModel::PINHOLE), 4);
    EXPECT_EQ(camera_model_num_params(CameraModel::SIMPLE_RADIAL), 4);
    EXPECT_EQ(camera_model_num_params(CameraModel::RADIAL), 5);
    EXPECT_EQ(camera_model_num_params(CameraModel::OPENCV), 8);
    EXPECT_EQ(camera_model_num_params(CameraModel::FULL_OPENCV), 12);
}

// ============================================================================
// COLMAP Text 格式测试
// ============================================================================

TEST(InfIO, ReadColmapTextSuccess) {
    auto result = inf::io::read_colmap_text(kTestDataDir);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    
    const auto& model = *result;
    
    // 验证数量
    EXPECT_EQ(model.cameras.size(), 1);
    EXPECT_EQ(model.images.size(), 730);
    EXPECT_EQ(model.points3d.size(), 8631);
}

TEST(InfIO, ReadColmapTextCameras) {
    auto result = inf::io::read_colmap_text(kTestDataDir);
    ASSERT_TRUE(result.has_value());
    
    const auto& model = *result;
    
    // 验证相机 1
    ASSERT_TRUE(model.cameras.count(1));
    const auto& cam = model.cameras.at(1);
    
    EXPECT_EQ(cam.id, 1);
    EXPECT_EQ(cam.model, inf::core::CameraModel::PINHOLE);
    EXPECT_EQ(cam.width, 960);
    EXPECT_EQ(cam.height, 720);
    ASSERT_EQ(cam.params.size(), 4);
    EXPECT_NEAR(cam.params[0], 689.864, 0.001);  // fx
    EXPECT_NEAR(cam.params[1], 690.193, 0.001);  // fy
    EXPECT_NEAR(cam.params[2], 479.5, 0.001);    // cx
    EXPECT_NEAR(cam.params[3], 359.5, 0.001);    // cy
}

TEST(InfIO, ReadColmapTextImages) {
    auto result = inf::io::read_colmap_text(kTestDataDir);
    ASSERT_TRUE(result.has_value());
    
    const auto& model = *result;
    
    // 验证至少有一个图像有 points2d
    bool has_points2d = false;
    for (const auto& [id, img] : model.images) {
        if (!img.points2d.empty()) {
            has_points2d = true;
            EXPECT_EQ(img.camera_id, 1);  // 所有图像使用相机 1
            break;
        }
    }
    EXPECT_TRUE(has_points2d);
}

TEST(InfIO, ReadColmapTextPoints3D) {
    auto result = inf::io::read_colmap_text(kTestDataDir);
    ASSERT_TRUE(result.has_value());
    
    const auto& model = *result;
    
    // 验证第一个点 (ID: 4295034187)
    ASSERT_TRUE(model.points3d.count(4295034187));
    const auto& pt = model.points3d.at(4295034187);
    
    EXPECT_EQ(pt.id, 4295034187);
    EXPECT_NEAR(pt.position.x(), 2.923, 0.001);
    EXPECT_NEAR(pt.position.y(), -0.861, 0.001);
    EXPECT_NEAR(pt.position.z(), -2.304, 0.001);
    EXPECT_EQ(static_cast<int>(pt.color.x()), 33);
    EXPECT_EQ(static_cast<int>(pt.color.y()), 50);
    EXPECT_EQ(static_cast<int>(pt.color.z()), 14);
    EXPECT_FALSE(pt.track.empty());
}

TEST(InfIO, ReadColmapTextNotFound) {
    auto result = inf::io::read_colmap_text("nonexistent_dir");
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, inf::core::ErrorCode::kFileNotFound);
}

// ============================================================================
// COLMAP Binary 格式测试
// ============================================================================

TEST(InfIO, ReadColmapBinarySuccess) {
    auto result = inf::io::read_colmap_binary(kTestDataDir);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    
    const auto& model = *result;
    
    // 验证数量 (应该和 text 格式一致)
    EXPECT_EQ(model.cameras.size(), 1);
    EXPECT_EQ(model.images.size(), 730);
    EXPECT_EQ(model.points3d.size(), 8631);
}

TEST(InfIO, ReadColmapBinaryCameras) {
    auto result = inf::io::read_colmap_binary(kTestDataDir);
    ASSERT_TRUE(result.has_value());
    
    const auto& model = *result;
    
    // 验证相机 1
    ASSERT_TRUE(model.cameras.count(1));
    const auto& cam = model.cameras.at(1);
    
    EXPECT_EQ(cam.id, 1);
    EXPECT_EQ(cam.model, inf::core::CameraModel::PINHOLE);
    EXPECT_EQ(cam.width, 960);
    EXPECT_EQ(cam.height, 720);
}

TEST(InfIO, ReadColmapBinaryNotFound) {
    auto result = inf::io::read_colmap_binary("nonexistent_dir");
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, inf::core::ErrorCode::kFileNotFound);
}

// ============================================================================
// 自动检测格式测试
// ============================================================================

TEST(InfIO, ReadColmapAutoDetect) {
    auto result = inf::io::read_colmap(kTestDataDir);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    
    const auto& model = *result;
    
    // 验证数量
    EXPECT_EQ(model.cameras.size(), 1);
    EXPECT_EQ(model.images.size(), 730);
    EXPECT_EQ(model.points3d.size(), 8631);
}

TEST(InfIO, ReadColmapAutoDetectNotFound) {
    auto result = inf::io::read_colmap("nonexistent_dir");
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, inf::core::ErrorCode::kFileNotFound);
}

// ============================================================================
// Text 和 Binary 结果一致性测试
// ============================================================================

TEST(InfIO, TextBinaryConsistency) {
    auto text_result = inf::io::read_colmap_text(kTestDataDir);
    auto bin_result = inf::io::read_colmap_binary(kTestDataDir);
    
    ASSERT_TRUE(text_result.has_value());
    ASSERT_TRUE(bin_result.has_value());
    
    const auto& text_model = *text_result;
    const auto& bin_model = *bin_result;
    
    // 数量一致
    EXPECT_EQ(text_model.cameras.size(), bin_model.cameras.size());
    EXPECT_EQ(text_model.images.size(), bin_model.images.size());
    EXPECT_EQ(text_model.points3d.size(), bin_model.points3d.size());
    
    // 相机参数一致
    for (const auto& [id, text_cam] : text_model.cameras) {
        ASSERT_TRUE(bin_model.cameras.count(id));
        const auto& bin_cam = bin_model.cameras.at(id);
        
        EXPECT_EQ(text_cam.model, bin_cam.model);
        EXPECT_EQ(text_cam.width, bin_cam.width);
        EXPECT_EQ(text_cam.height, bin_cam.height);
        ASSERT_EQ(text_cam.params.size(), bin_cam.params.size());
        for (size_t i = 0; i < text_cam.params.size(); ++i) {
            EXPECT_NEAR(text_cam.params[i], bin_cam.params[i], 1e-6);
        }
    }
}

// ============================================================================
// Mesh 读写测试
// ============================================================================

TEST(InfIO, ReadPlyMesh) {
    const auto mesh_path = kTestDataDir / "ply" / "mesh.ply";
    auto result = inf::io::read_ply_mesh(mesh_path);
    ASSERT_TRUE(result.has_value()) << result.error().message;

    const auto& mesh = *result;
    EXPECT_GT(mesh.vertices.size(), 0);
    EXPECT_GT(mesh.indices.size(), 0);
    EXPECT_EQ(mesh.indices.size() % 3, 0);  // 三角形索引
}

TEST(InfIO, ReadPlyMeshNotFound) {
    auto result = inf::io::read_ply_mesh("nonexistent.ply");
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, inf::core::ErrorCode::kFileNotFound);
}

TEST(InfIO, WritePlyMeshBinary) {
    // 读取原始 mesh
    const auto mesh_path = kTestDataDir / "ply" / "mesh.ply";
    auto read_result = inf::io::read_ply_mesh(mesh_path);
    ASSERT_TRUE(read_result.has_value());
    const auto& original = *read_result;

    // 写入临时文件 (binary)
    const auto tmp_path = std::filesystem::temp_directory_path() / "test_mesh_bin.ply";
    auto write_result = inf::io::write_ply_mesh(tmp_path, original, true);
    ASSERT_TRUE(write_result.has_value()) << write_result.error().message;

    // 重新读取验证
    auto reread_result = inf::io::read_ply_mesh(tmp_path);
    ASSERT_TRUE(reread_result.has_value());
    const auto& reread = *reread_result;

    EXPECT_EQ(original.vertices.size(), reread.vertices.size());
    EXPECT_EQ(original.indices.size(), reread.indices.size());

    // 清理
    std::filesystem::remove(tmp_path);
}

TEST(InfIO, WritePlyMeshAscii) {
    // 创建一个小的测试 mesh，避免 ASCII 大文件测试太慢
    inf::mesh::Mesh small_mesh;
    small_mesh.vertices = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.5f, 1.0f, 0.0f},
        {0.5f, 0.5f, 1.0f}
    };
    small_mesh.indices = {0, 1, 2, 0, 1, 3, 1, 2, 3, 0, 2, 3};  // 4 triangles

    // 写入临时文件 (ascii)
    const auto tmp_path = std::filesystem::temp_directory_path() / "test_mesh_ascii.ply";
    auto write_result = inf::io::write_ply_mesh(tmp_path, small_mesh, false);
    ASSERT_TRUE(write_result.has_value()) << write_result.error().message;

    // 重新读取验证
    auto reread_result = inf::io::read_ply_mesh(tmp_path);
    ASSERT_TRUE(reread_result.has_value());
    const auto& reread = *reread_result;

    EXPECT_EQ(small_mesh.vertices.size(), reread.vertices.size());
    EXPECT_EQ(small_mesh.indices.size(), reread.indices.size());

    // 验证顶点数据近似相等
    for (size_t i = 0; i < small_mesh.vertices.size(); ++i) {
        EXPECT_NEAR(small_mesh.vertices[i].x(), reread.vertices[i].x(), 1e-5f);
        EXPECT_NEAR(small_mesh.vertices[i].y(), reread.vertices[i].y(), 1e-5f);
        EXPECT_NEAR(small_mesh.vertices[i].z(), reread.vertices[i].z(), 1e-5f);
    }

    // 清理
    std::filesystem::remove(tmp_path);
}

// ============================================================================
// PointCloud 读写测试
// ============================================================================

TEST(InfIO, ReadPointCloud) {
    const auto pc_path = kTestDataDir / "ply" / "point_cloud.ply";
    auto result = inf::io::read_point_cloud(pc_path);
    ASSERT_TRUE(result.has_value()) << result.error().message;

    const auto& cloud = *result;
    EXPECT_GT(cloud.positions.size(), 0);
}

TEST(InfIO, ReadPointCloudNotFound) {
    auto result = inf::io::read_point_cloud("nonexistent.ply");
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, inf::core::ErrorCode::kFileNotFound);
}

TEST(InfIO, WritePointCloudPlyBinary) {
    // 读取原始点云
    const auto pc_path = kTestDataDir / "ply" / "point_cloud.ply";
    auto read_result = inf::io::read_point_cloud(pc_path);
    ASSERT_TRUE(read_result.has_value());
    const auto& original = *read_result;

    // 写入临时文件 (binary ply)
    const auto tmp_path = std::filesystem::temp_directory_path() / "test_pc_bin.ply";
    auto write_result = inf::io::write_point_cloud(tmp_path, original, true);
    ASSERT_TRUE(write_result.has_value()) << write_result.error().message;

    // 重新读取验证
    auto reread_result = inf::io::read_point_cloud(tmp_path);
    ASSERT_TRUE(reread_result.has_value());
    const auto& reread = *reread_result;

    EXPECT_EQ(original.positions.size(), reread.positions.size());

    // 清理
    std::filesystem::remove(tmp_path);
}

TEST(InfIO, WritePointCloudPlyAscii) {
    // 读取原始点云
    const auto pc_path = kTestDataDir / "ply" / "point_cloud.ply";
    auto read_result = inf::io::read_point_cloud(pc_path);
    ASSERT_TRUE(read_result.has_value());
    const auto& original = *read_result;

    // 写入临时文件 (ascii ply)
    const auto tmp_path = std::filesystem::temp_directory_path() / "test_pc_ascii.ply";
    auto write_result = inf::io::write_point_cloud(tmp_path, original, false);
    ASSERT_TRUE(write_result.has_value()) << write_result.error().message;

    // 重新读取验证
    auto reread_result = inf::io::read_point_cloud(tmp_path);
    ASSERT_TRUE(reread_result.has_value());
    const auto& reread = *reread_result;

    EXPECT_EQ(original.positions.size(), reread.positions.size());

    // 清理
    std::filesystem::remove(tmp_path);
}

TEST(InfIO, WritePointCloudPcd) {
    // 读取原始点云
    const auto pc_path = kTestDataDir / "ply" / "point_cloud.ply";
    auto read_result = inf::io::read_point_cloud(pc_path);
    ASSERT_TRUE(read_result.has_value());
    const auto& original = *read_result;

    // 写入 PCD 格式
    const auto tmp_path = std::filesystem::temp_directory_path() / "test_pc.pcd";
    auto write_result = inf::io::write_point_cloud(tmp_path, original, true);
    ASSERT_TRUE(write_result.has_value()) << write_result.error().message;

    // 重新读取验证
    auto reread_result = inf::io::read_point_cloud(tmp_path);
    ASSERT_TRUE(reread_result.has_value());
    const auto& reread = *reread_result;

    EXPECT_EQ(original.positions.size(), reread.positions.size());

    // 清理
    std::filesystem::remove(tmp_path);
}

TEST(InfIO, FormatFromExtension) {
    EXPECT_EQ(inf::io::format_from_extension("test.ply"), inf::io::IoFormat::Mesh);
    EXPECT_EQ(inf::io::format_from_extension("test.pcd"), inf::io::IoFormat::PointCloud);
    EXPECT_EQ(inf::io::format_from_extension("test.xyz"), inf::io::IoFormat::Unknown);
}

TEST(InfIO, FormatFromPath) {
    EXPECT_EQ(inf::io::format_from_path(kTestDataDir), inf::io::IoFormat::SparseModel);
    EXPECT_EQ(inf::io::format_from_path(kTestDataDir / "ply" / "mesh.ply"), inf::io::IoFormat::Mesh);
}
