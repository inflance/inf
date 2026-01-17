#include "InfPointCloud.hpp"
#include <gtest/gtest.h>
#include <random>

namespace {

inf::pc::PointCloud make_test_cloud(size_t n, bool with_normals = false, bool with_colors = false) {
    inf::pc::PointCloud cloud;
    cloud.positions.reserve(n);
    std::mt19937 gen(42);
    std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
    for (size_t i = 0; i < n; ++i) {
        cloud.positions.emplace_back(dist(gen), dist(gen), dist(gen));
    }
    if (with_normals) {
        cloud.normals.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            inf::core::Vec3f normal(dist(gen), dist(gen), dist(gen));
            normal.normalize();
            cloud.normals.push_back(normal);
        }
    }
    if (with_colors) {
        cloud.colors.reserve(n);
        std::uniform_real_distribution<float> cdist(0.0f, 255.0f);
        for (size_t i = 0; i < n; ++i) {
            cloud.colors.emplace_back(cdist(gen), cdist(gen), cdist(gen));
        }
    }
    return cloud;
}

inf::pc::PointCloud make_grid_cloud(int side) {
    inf::pc::PointCloud cloud;
    cloud.positions.reserve(side * side * side);
    for (int x = 0; x < side; ++x) {
        for (int y = 0; y < side; ++y) {
            for (int z = 0; z < side; ++z) {
                cloud.positions.emplace_back(
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(z));
            }
        }
    }
    return cloud;
}

}  // namespace

TEST(PointCloud, BasicProperties) {
    inf::pc::PointCloud cloud;
    EXPECT_TRUE(cloud.empty());
    EXPECT_EQ(cloud.size(), 0);
    EXPECT_FALSE(cloud.has_normals());
    EXPECT_FALSE(cloud.has_colors());

    cloud = make_test_cloud(100);
    EXPECT_FALSE(cloud.empty());
    EXPECT_EQ(cloud.size(), 100);
    EXPECT_FALSE(cloud.has_normals());
    EXPECT_FALSE(cloud.has_colors());

    auto cloud2 = make_test_cloud(100, true, true);
    EXPECT_TRUE(cloud2.has_normals());
    EXPECT_TRUE(cloud2.has_colors());
}

TEST(PointCloud, BoundingBox) {
    auto cloud = make_grid_cloud(10);
    auto [min_pt, max_pt] = cloud.bounding_box();
    EXPECT_FLOAT_EQ(min_pt.x(), 0.0f);
    EXPECT_FLOAT_EQ(min_pt.y(), 0.0f);
    EXPECT_FLOAT_EQ(min_pt.z(), 0.0f);
    EXPECT_FLOAT_EQ(max_pt.x(), 9.0f);
    EXPECT_FLOAT_EQ(max_pt.y(), 9.0f);
    EXPECT_FLOAT_EQ(max_pt.z(), 9.0f);
}

TEST(PointCloud, Append) {
    auto cloud1 = make_test_cloud(50);
    auto cloud2 = make_test_cloud(30);
    size_t size1 = cloud1.size();
    cloud1.append(std::move(cloud2));
    EXPECT_EQ(cloud1.size(), size1 + 30);
    EXPECT_TRUE(cloud2.empty());

    auto c1 = make_test_cloud(50, true, true);
    auto c2 = make_test_cloud(30, true, true);
    c1.append(c2);
    EXPECT_EQ(c1.size(), 80);
    EXPECT_TRUE(c1.has_normals());
    EXPECT_TRUE(c1.has_colors());
}

TEST(PointCloud, ReserveClearShrink) {
    inf::pc::PointCloud cloud;
    cloud.reserve(1000);
    EXPECT_GE(cloud.positions.capacity(), 1000);

    cloud = make_test_cloud(100);
    cloud.clear();
    EXPECT_TRUE(cloud.empty());

    cloud = make_test_cloud(100);
    cloud.shrink_to_fit();
    EXPECT_EQ(cloud.positions.capacity(), cloud.positions.size());
}

TEST(VoxelDownsample, Basic) {
    auto cloud = make_grid_cloud(10);
    EXPECT_EQ(cloud.size(), 1000);

    auto result = inf::pc::voxel_downsample(cloud, 2.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->size(), cloud.size());
    EXPECT_GT(result->size(), 0);
}

TEST(VoxelDownsample, PreservesAttributes) {
    auto cloud = make_test_cloud(500, true, true);
    auto result = inf::pc::voxel_downsample(cloud, 5.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->has_normals());
    EXPECT_TRUE(result->has_colors());
}

TEST(VoxelDownsample, InvalidInput) {
    inf::pc::PointCloud empty;
    auto result = inf::pc::voxel_downsample(empty, 1.0f);
    EXPECT_FALSE(result.has_value());

    auto cloud = make_test_cloud(100);
    result = inf::pc::voxel_downsample(cloud, -1.0f);
    EXPECT_FALSE(result.has_value());
}

TEST(RandomDownsample, Basic) {
    auto cloud = make_test_cloud(1000);
    auto result = inf::pc::random_downsample(cloud, 100, 42);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 100);
}

TEST(RandomDownsample, Deterministic) {
    auto cloud = make_test_cloud(1000);
    auto r1 = inf::pc::random_downsample(cloud, 100, 42);
    auto r2 = inf::pc::random_downsample(cloud, 100, 42);
    ASSERT_TRUE(r1.has_value() && r2.has_value());
    EXPECT_EQ(r1->positions[0], r2->positions[0]);
}

TEST(RandomDownsample, TargetLargerThanCloud) {
    auto cloud = make_test_cloud(50);
    auto result = inf::pc::random_downsample(cloud, 100);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 50);
}

TEST(UniformDownsample, Basic) {
    auto cloud = make_grid_cloud(10);
    auto result = inf::pc::uniform_downsample(cloud, 2.5f);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->size(), cloud.size());
}

TEST(GridDownsample, Basic) {
    auto cloud = make_test_cloud(100);
    auto result = inf::pc::grid_downsample(cloud, 5);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 20);
}

TEST(GridDownsample, StepOne) {
    auto cloud = make_test_cloud(100);
    auto result = inf::pc::grid_downsample(cloud, 1);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 100);
}

TEST(EstimateNormals, Basic) {
    auto cloud = make_grid_cloud(5);
    inf::pc::NormalEstimationParams params;
    params.k_neighbors = 10;
    params.orient_normals = false;

    auto result = inf::pc::estimate_normals(cloud, params);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(cloud.has_normals());
    EXPECT_EQ(cloud.normals.size(), cloud.size());

    for (const auto& n : cloud.normals) {
        float len = n.norm();
        EXPECT_NEAR(len, 1.0f, 0.01f);
    }
}

TEST(EstimateNormals, EmptyCloud) {
    inf::pc::PointCloud empty;
    auto result = inf::pc::estimate_normals(empty);
    EXPECT_FALSE(result.has_value());
}

TEST(OrientNormals, TowardsViewpoint) {
    auto cloud = make_test_cloud(100, true);
    inf::core::Vec3f viewpoint(100.0f, 100.0f, 100.0f);

    inf::pc::orient_normals_towards_viewpoint(cloud, viewpoint);

    for (size_t i = 0; i < cloud.size(); ++i) {
        inf::core::Vec3f dir = viewpoint - cloud.positions[i];
        EXPECT_GE(cloud.normals[i].dot(dir), 0.0f);
    }
}

TEST(StatisticalOutlierRemoval, Basic) {
    auto cloud = make_grid_cloud(5);
    cloud.positions.emplace_back(100.0f, 100.0f, 100.0f);

    auto result = inf::pc::statistical_outlier_removal(cloud, 10, 1.0);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->size(), cloud.size());
}

TEST(StatisticalOutlierRemoval, PreservesAttributes) {
    auto cloud = make_test_cloud(200, true, true);
    auto result = inf::pc::statistical_outlier_removal(cloud, 20, 2.0);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->has_normals());
    EXPECT_TRUE(result->has_colors());
}

TEST(RadiusOutlierRemoval, Basic) {
    auto cloud = make_grid_cloud(5);
    cloud.positions.emplace_back(100.0f, 100.0f, 100.0f);

    auto result = inf::pc::radius_outlier_removal(cloud, 2.0f, 3);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->size(), cloud.size());
}

TEST(PassthroughFilter, Basic) {
    auto cloud = make_grid_cloud(10);
    auto result = inf::pc::passthrough_filter(cloud, 0, 2.0f, 7.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->size(), cloud.size());

    for (const auto& p : result->positions) {
        EXPECT_GE(p.x(), 2.0f);
        EXPECT_LE(p.x(), 7.0f);
    }
}

TEST(PassthroughFilter, InvalidAxis) {
    auto cloud = make_test_cloud(100);
    auto result = inf::pc::passthrough_filter(cloud, 5, 0.0f, 1.0f);
    EXPECT_FALSE(result.has_value());
}

TEST(CropBox, Basic) {
    auto cloud = make_grid_cloud(10);
    inf::core::Vec3f min_b(2.0f, 2.0f, 2.0f);
    inf::core::Vec3f max_b(5.0f, 5.0f, 5.0f);

    auto result = inf::pc::crop_box(cloud, min_b, max_b);
    ASSERT_TRUE(result.has_value());

    for (const auto& p : result->positions) {
        EXPECT_GE(p.x(), 2.0f); EXPECT_LE(p.x(), 5.0f);
        EXPECT_GE(p.y(), 2.0f); EXPECT_LE(p.y(), 5.0f);
        EXPECT_GE(p.z(), 2.0f); EXPECT_LE(p.z(), 5.0f);
    }
}

TEST(BilateralSmooth, RequiresNormals) {
    auto cloud = make_test_cloud(100);
    auto result = inf::pc::bilateral_smooth(cloud, 0.5f, 0.5f);
    EXPECT_FALSE(result.has_value());

    cloud = make_test_cloud(100, true);
    result = inf::pc::bilateral_smooth(cloud, 0.5f, 0.5f);
    EXPECT_TRUE(result.has_value());
}

TEST(MLSSmooth, Basic) {
    auto cloud = make_grid_cloud(5);
    auto result = inf::pc::mls_smooth(cloud, 2.0f, 2, true);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->has_normals());
}

TEST(ComputeCentroid, Basic) {
    auto cloud = make_grid_cloud(10);
    auto centroid = inf::pc::compute_centroid(cloud);
    EXPECT_NEAR(centroid.x(), 4.5f, 0.01f);
    EXPECT_NEAR(centroid.y(), 4.5f, 0.01f);
    EXPECT_NEAR(centroid.z(), 4.5f, 0.01f);
}

TEST(ExtractByIndices, Basic) {
    auto cloud = make_test_cloud(100, true, true);
    std::vector<size_t> indices = {0, 10, 20, 30};
    auto result = inf::pc::extract_by_indices(cloud, indices);
    EXPECT_EQ(result.size(), 4);
    EXPECT_TRUE(result.has_normals());
    EXPECT_TRUE(result.has_colors());
    EXPECT_EQ(result.positions[0], cloud.positions[0]);
    EXPECT_EQ(result.positions[1], cloud.positions[10]);
}

TEST(RemoveNanPoints, Basic) {
    inf::pc::PointCloud cloud;
    cloud.positions = {
        {1.0f, 2.0f, 3.0f},
        {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f},
        {4.0f, 5.0f, 6.0f},
        {0.0f, std::numeric_limits<float>::infinity(), 0.0f}
    };
    auto result = inf::pc::remove_nan_points(cloud);
    EXPECT_EQ(result.size(), 2);
}

TEST(Transform, Basic) {
    inf::pc::PointCloud cloud;
    cloud.positions = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
    cloud.normals = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};

    inf::core::Mat4f T = inf::core::Mat4f::Identity();
    T(0, 3) = 10.0f;

    auto result = inf::pc::transform(cloud, T);
    ASSERT_TRUE(result.has_value());
    EXPECT_FLOAT_EQ(result->positions[0].x(), 11.0f);
    EXPECT_FLOAT_EQ(result->positions[1].x(), 10.0f);
}

TEST(ComputeStats, Basic) {
    auto cloud = make_grid_cloud(10);
    auto stats = inf::pc::compute_stats(cloud);
    EXPECT_EQ(stats.num_points, 1000);
    EXPECT_NEAR(stats.centroid.x(), 4.5f, 0.01f);
    EXPECT_GT(stats.point_density, 0.0f);
}

TEST(FilterByConfidence, Basic) {
    inf::pc::PointCloud cloud = make_test_cloud(100);
    cloud.confidences.resize(100);
    for (size_t i = 0; i < 100; ++i) {
        cloud.confidences[i] = static_cast<float>(i) / 100.0f;
    }

    auto result = inf::pc::filter_by_confidence(cloud, 0.5f);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 50);
}

TEST(FilterByVisibility, Basic) {
    inf::pc::PointCloud cloud = make_test_cloud(100);
    cloud.visible_views.resize(100);
    for (size_t i = 0; i < 100; ++i) {
        cloud.visible_views[i].resize(i % 5);
    }

    auto result = inf::pc::filter_by_visibility(cloud, 3);
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->size(), cloud.size());
}

TEST(MergeClouds, Simple) {
    auto c1 = make_test_cloud(50);
    auto c2 = make_test_cloud(50);
    std::vector<inf::pc::PointCloud> clouds = {c1, c2};

    auto result = inf::pc::merge_clouds(clouds, 0.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 100);
}

TEST(MergeClouds, WithDedup) {
    auto c1 = make_grid_cloud(5);
    auto c2 = make_grid_cloud(5);
    std::vector<inf::pc::PointCloud> clouds = {c1, c2};

    auto result = inf::pc::merge_clouds(clouds, 0.5f);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 125);
}

TEST(MergeClouds, EmptyList) {
    std::vector<inf::pc::PointCloud> empty;
    auto result = inf::pc::merge_clouds(empty);
    EXPECT_FALSE(result.has_value());
}

TEST(MVSCloud, Properties) {
    inf::pc::PointCloud cloud = make_test_cloud(100);
    EXPECT_FALSE(cloud.is_mvs_cloud());

    cloud.confidences.resize(100, 0.8f);
    EXPECT_TRUE(cloud.is_mvs_cloud());
    EXPECT_TRUE(cloud.has_confidences());
}
