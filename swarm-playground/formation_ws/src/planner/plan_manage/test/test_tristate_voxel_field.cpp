#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <starconvex_adapter/original_star_space.hpp>

namespace {

using starconvex_adapter::Vec3;
using starconvex_adapter::VoxelObstacleField;
using starconvex_adapter::GeometryOptions;
using starconvex_adapter::generateBoundaryCloud;
using starconvex_adapter::VoxelGridLayout;

TEST(PlannerVoxelField, PreservesFloorBoundariesAndDoesNotReinflate) {
  const VoxelGridLayout layout(0.1, {-12,-7,-0.01}, true);
  EXPECT_EQ(48, layout.indexOf({-2.2,-2.2,1.5}).y);
  pcl::PointCloud<pcl::PointXYZ> planner, free;
  planner.push_back(pcl::PointXYZ(-1.75F,-2.15F,1.74F));
  free.push_back(pcl::PointXYZ(-1.6F,-2.2F,1.8F));
  const VoxelObstacleField field(planner,layout,free,VoxelGridLayout(0.2),{24,14,3});
  // The recorded 5--6 wall-corner failure point is outside the planner's
  // inflated cells. The legacy extra inflation used to mark it occupied.
  const Vec3 recorded{-1.576658,-2.293396,1.744613};
  EXPECT_FALSE(field.occupied(recorded));
  EXPECT_TRUE(field.knownFree(recorded));
  EXPECT_TRUE(field.occupied({-1.75,-2.15,1.74}));
  EXPECT_FALSE(field.occupied({-1.65,-2.15,1.74}));
}

TEST(PlannerVoxelField, PlannerOccupancyOverridesCoarserObservedFree) {
  pcl::PointCloud<pcl::PointXYZ> planner, free;
  planner.push_back(pcl::PointXYZ(0.05F,0.05F,1.05F));
  free.push_back(pcl::PointXYZ(0,0,1));
  const VoxelObstacleField field(planner,VoxelGridLayout(0.1,{},true),
                                 free,VoxelGridLayout(0.2));
  EXPECT_TRUE(field.occupied({0.04,0.04,1.04}));
  EXPECT_FALSE(field.knownFree({0.04,0.04,1.04}));
  EXPECT_FALSE(field.knownFree({0.4,0.4,1.04}));
}

TEST(PlannerVoxelField, SegmentCannotBridgeUnknownOrOccupiedCells) {
  pcl::PointCloud<pcl::PointXYZ> planner, free;
  for (int i=0; i<=5; ++i) free.push_back(pcl::PointXYZ(i*0.2F,0,1));
  const VoxelGridLayout fine(0.1,{},true), observed(0.2);
  const VoxelObstacleField clear(planner,fine,free,observed);
  EXPECT_TRUE(clear.knownFreeSegment({0.01,0.01,1.04},{0.99,0.01,1.04}));
  EXPECT_TRUE(clear.knownFreeSegment({0.99,0.01,1.04},{0.01,0.01,1.04}));
  planner.push_back(pcl::PointXYZ(0.55F,0.05F,1.05F));
  const VoxelObstacleField blocked(planner,fine,free,observed);
  EXPECT_FALSE(blocked.knownFreeSegment({0.01,0.01,1.04},{0.99,0.01,1.04}));
  planner.clear();
  free.erase(free.begin()+2);
  const VoxelObstacleField unknown(planner,fine,free,observed);
  EXPECT_FALSE(unknown.knownFreeSegment({0.01,0.01,1.04},{0.99,0.01,1.04}));
}

TEST(PlannerVoxelField, SegmentDetectsThinDiagonalVoxelIntersection) {
  pcl::PointCloud<pcl::PointXYZ> planner, free;
  planner.push_back(pcl::PointXYZ(0.15F,0.15F,1.05F));
  for (int x=0; x<=6; ++x)
    for (int y=0; y<=6; ++y) free.push_back(pcl::PointXYZ(x*0.2F,y*0.2F,1));
  const VoxelObstacleField field(planner,VoxelGridLayout(0.1,{},true),
                                 free,VoxelGridLayout(0.2));
  // This segment only clips ~1.5 mm of the occupied cell. Fixed 5 cm samples
  // can skip it; voxel traversal must reject it in both directions.
  const Vec3 a{0.001,0.0995,1.04}, b{0.999,1.0975,1.04};
  EXPECT_TRUE(field.knownFree(a));
  EXPECT_TRUE(field.knownFree(b));
  EXPECT_FALSE(field.knownFreeSegment(a,b));
  EXPECT_FALSE(field.knownFreeSegment(b,a));
}

TEST(PlannerVoxelField, UnknownMapBoundaryAndHighObstaclesRemainUnsafe) {
  pcl::PointCloud<pcl::PointXYZ> planner, free;
  planner.push_back(pcl::PointXYZ(-2.15F,-2.05F,2.54F));
  free.push_back(pcl::PointXYZ(-2.2F,-2.0F,2.6F));
  free.push_back(pcl::PointXYZ(12,0,1));
  const VoxelObstacleField field(planner,VoxelGridLayout(0.1,{-12,-7,-0.01},true),
                                 free,VoxelGridLayout(0.2),{24,14,3});
  EXPECT_TRUE(field.occupied({-2.15,-2.05,2.54}));
  EXPECT_FALSE(field.knownFree({-2.15,-2.05,2.54}));
  EXPECT_FALSE(field.knownFree({12,0,1}));
  EXPECT_FALSE(field.knownFreeSegment({12,0,1},{12,0,1}));
  EXPECT_FALSE(field.knownFreeSegment(
      {std::numeric_limits<double>::quiet_NaN(),0,1},{0,0,1}));
}

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
