#include <array>

#include <gtest/gtest.h>

#include <path_searching/hungarian_assignment.hpp>

TEST(HungarianAssignment, FindsKnownMinimum)
{
  Eigen::MatrixXd cost(3, 3);
  cost << 4.0, 1.0, 3.0,
          2.0, 0.0, 5.0,
          3.0, 2.0, 2.0;
  const Eigen::VectorXi assignment = path_searching::hungarianAssignment(cost);
  EXPECT_EQ(1, assignment(0));
  EXPECT_EQ(0, assignment(1));
  EXPECT_EQ(2, assignment(2));
}

TEST(HungarianAssignment, FindsSixVehiclePermutation)
{
  Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(6, 6, 100.0);
  const std::array<int, 6> expected{{5, 3, 1, 4, 2, 0}};
  for (int drone = 0; drone < 6; ++drone)
    cost(drone, expected[drone]) = static_cast<double>(drone);

  const Eigen::VectorXi assignment = path_searching::hungarianAssignment(cost);
  for (int drone = 0; drone < 6; ++drone)
    EXPECT_EQ(expected[drone], assignment(drone));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(HungarianAssignment, SupportsNegativeCosts)
{
  Eigen::MatrixXd cost(2, 2);
  cost << -1.0, -4.0,
          -3.0, -2.0;
  const Eigen::VectorXi assignment = path_searching::hungarianAssignment(cost);
  EXPECT_EQ(1, assignment(0));
  EXPECT_EQ(0, assignment(1));
}

TEST(HungarianAssignment, RejectsNonSquareInput)
{
  EXPECT_THROW(path_searching::hungarianAssignment(Eigen::MatrixXd::Zero(2, 3)),
               std::invalid_argument);
}
