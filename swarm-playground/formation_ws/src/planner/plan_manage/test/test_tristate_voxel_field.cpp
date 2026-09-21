#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <starconvex_adapter/original_star_space.hpp>

namespace {

using starconvex_adapter::Vec3;
using starconvex_adapter::VoxelObstacleField;
using starconvex_adapter::GeometryOptions;
using starconvex_adapter::generateBoundaryCloud;

TEST(TriStateVoxelField, UnknownIsNotKnownFree) {
  pcl::PointCloud<pcl::PointXYZ> occupied;
  occupied.push_back(pcl::PointXYZ(1.0F, 0.0F, 1.0F));
  pcl::PointCloud<pcl::PointXYZ> known_free;
  for (int index = 0; index < 5; ++index)
    known_free.push_back(pcl::PointXYZ(index * 0.2F, 0.0F, 1.0F));

  const VoxelObstacleField field(occupied, known_free, 0.2, 0.0);
  EXPECT_TRUE(field.knownFree({0.4, 0.0, 1.0}));
  EXPECT_FALSE(field.knownFree({0.4, 1.0, 1.0}));
  EXPECT_TRUE(field.occupied({1.0, 0.0, 1.0}));
}

TEST(TriStateVoxelField, RayStopsAtFirstUnknownVoxel) {
  pcl::PointCloud<pcl::PointXYZ> occupied;
  occupied.push_back(pcl::PointXYZ(2.0F, 0.0F, 1.0F));
  pcl::PointCloud<pcl::PointXYZ> known_free;
  known_free.push_back(pcl::PointXYZ(0.0F, 0.0F, 1.0F));
  known_free.push_back(pcl::PointXYZ(0.2F, 0.0F, 1.0F));

  const VoxelObstacleField field(occupied, known_free, 0.2, 0.0);
  const Vec3 hit = field.firstUnsafe({0.0, 0.0, 1.0},
                                     {1.0, 0.0, 1.0}, 0.2);
  EXPECT_NEAR(hit.x, 0.4, 1e-9);
}

TEST(TriStateVoxelField, LegacyFieldRetainsBinaryBehavior) {
  pcl::PointCloud<pcl::PointXYZ> occupied;
  occupied.push_back(pcl::PointXYZ(1.0F, 0.0F, 1.0F));

  const VoxelObstacleField field(occupied, 0.2, 0.0);
  EXPECT_TRUE(field.knownFree({0.4, 1.0, 1.0}));
  EXPECT_FALSE(field.hasKnownFreeLayer());
}

TEST(StarConvexVerticalSampling, CoversZeroToThreeMeterFlightEnvelope) {
  pcl::PointCloud<pcl::PointXYZ> occupied;
  occupied.push_back(pcl::PointXYZ(100.0F, 100.0F, 100.0F));
  const VoxelObstacleField field(occupied, 0.2, 0.0);
  GeometryOptions options;
  options.sensor_range = 1.0;
  options.azimuth_step = 0.5;
  options.raycast_resolution = 0.2;
  options.vertical_resolution = 0.2;
  options.frontier_floor = 0.0;
  options.frontier_roof = 3.0;
  options.delta_floor = -0.5;
  options.delta_roof = 0.5;

  EXPECT_NO_THROW({
    const auto low = generateBoundaryCloud({0.0, 0.0, 0.0}, field, options);
    EXPECT_FALSE(low->empty());
  });
  EXPECT_NO_THROW({
    const auto high = generateBoundaryCloud({0.0, 0.0, 3.0}, field, options);
    EXPECT_FALSE(high->empty());
  });
  EXPECT_NO_THROW({
    const auto recorded_peak =
        generateBoundaryCloud({0.0, 0.0, 2.75}, field, options);
    EXPECT_FALSE(recorded_peak->empty());
  });
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
