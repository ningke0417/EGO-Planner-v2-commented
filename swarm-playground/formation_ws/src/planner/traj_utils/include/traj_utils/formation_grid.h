#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ego_planner {

// Rows run from front to rear along x; columns run from left to right along y.
// Edge ordering preserves the original 2x4 graph: horizontal edges, then vertical.
inline std::vector<std::array<int, 2>> rectangularFormationEdges(int rows, int columns) {
  if (rows < 1 || columns < 1 || rows > 16 || columns > 16 ||
      rows * columns < 2 || rows * columns > 16)
    throw std::invalid_argument("formation grid requires 2 to 16 members");
  std::vector<std::array<int, 2>> result;
  for (int row = 0; row < rows; ++row)
    for (int column = 0; column + 1 < columns; ++column)
      result.push_back({{row * columns + column, row * columns + column + 1}});
  for (int row = 0; row + 1 < rows; ++row)
    for (int column = 0; column < columns; ++column)
      result.push_back({{row * columns + column, (row + 1) * columns + column}});
  if (result.size() > 31)
    throw std::invalid_argument("formation grid exceeds the 31-edge protocol capacity");
  return result;
}

inline std::vector<std::array<double, 3>> rectangularFormationOffsets(
    int rows, int columns, double row_spacing = 1.0, double column_spacing = 1.0) {
  rectangularFormationEdges(rows, columns);
  if (!std::isfinite(row_spacing) || !std::isfinite(column_spacing) ||
      row_spacing <= 0 || column_spacing <= 0)
    throw std::invalid_argument("formation spacing must be finite and positive");
  std::vector<std::array<double, 3>> result;
  for (int row = 0; row < rows; ++row)
    for (int column = 0; column < columns; ++column)
      result.push_back({{(0.5 * (rows - 1) - row) * row_spacing,
                         (column - 0.5 * (columns - 1)) * column_spacing, 0.0}});
  return result;
}

}  // namespace ego_planner
