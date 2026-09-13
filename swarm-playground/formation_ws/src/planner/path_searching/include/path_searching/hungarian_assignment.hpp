#ifndef PATH_SEARCHING_HUNGARIAN_ASSIGNMENT_HPP
#define PATH_SEARCHING_HUNGARIAN_ASSIGNMENT_HPP

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace path_searching
{

// Minimum-cost one-to-one assignment. The returned vector maps each row
// (physical UAV) to one column (formation slot).
inline Eigen::VectorXi hungarianAssignment(const Eigen::MatrixXd &cost)
{
  if (cost.rows() == 0 || cost.rows() != cost.cols())
    throw std::invalid_argument("Hungarian assignment requires a non-empty square matrix");
  if (!cost.allFinite())
    throw std::invalid_argument("Hungarian assignment cost matrix must be finite");

  const int n = cost.rows();
  std::vector<double> row_potential(n + 1, 0.0);
  std::vector<double> col_potential(n + 1, 0.0);
  std::vector<int> matched_row(n + 1, 0);
  std::vector<int> previous_col(n + 1, 0);

  for (int row = 1; row <= n; ++row)
  {
    matched_row[0] = row;
    int current_col = 0;
    std::vector<double> min_reduced_cost(
        n + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(n + 1, false);

    do
    {
      used[current_col] = true;
      const int current_row = matched_row[current_col];
      double delta = std::numeric_limits<double>::infinity();
      int next_col = 0;
      for (int col = 1; col <= n; ++col)
      {
        if (used[col])
          continue;
        const double reduced_cost =
            cost(current_row - 1, col - 1) - row_potential[current_row] -
            col_potential[col];
        if (reduced_cost < min_reduced_cost[col])
        {
          min_reduced_cost[col] = reduced_cost;
          previous_col[col] = current_col;
        }
        // The strict comparison gives deterministic lower-slot tie breaking.
        if (min_reduced_cost[col] < delta)
        {
          delta = min_reduced_cost[col];
          next_col = col;
        }
      }
      if (!std::isfinite(delta))
        throw std::runtime_error("Hungarian assignment could not find a finite matching");
      for (int col = 0; col <= n; ++col)
      {
        if (used[col])
        {
          row_potential[matched_row[col]] += delta;
          col_potential[col] -= delta;
        }
        else
        {
          min_reduced_cost[col] -= delta;
        }
      }
      current_col = next_col;
    } while (matched_row[current_col] != 0);

    do
    {
      const int predecessor = previous_col[current_col];
      matched_row[current_col] = matched_row[predecessor];
      current_col = predecessor;
    } while (current_col != 0);
  }

  Eigen::VectorXi assignment = Eigen::VectorXi::Constant(n, -1);
  for (int col = 1; col <= n; ++col)
    assignment(matched_row[col] - 1) = col - 1;
  return assignment;
}

} // namespace path_searching

#endif
