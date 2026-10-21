/**
 * @file cell_range.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Shared uniform grid cell math
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cmath>
#include <cstdint>

#include <glm/common.hpp>
#include <glm/vec3.hpp>

#include "cyberverse/geometry/bounds.hpp"

namespace cyberverse {
namespace spatial_index {
namespace detail {

/**
 * @brief Signed per-axis cell coordinate
 *
 */
using cell_vec = glm::vec<3, std::int64_t>;

inline bool cell_is_finite(const glm::vec3& value) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

inline glm::dvec3 cell_to_dvec3(const glm::vec3& value) noexcept {
  return glm::dvec3(static_cast<double>(value.x), static_cast<double>(value.y), static_cast<double>(value.z));
}

inline glm::dvec3 cell_to_dvec3(const cell_vec& value) noexcept {
  return glm::dvec3(static_cast<double>(value.x), static_cast<double>(value.y), static_cast<double>(value.z));
}

inline bool cell_double_to_i64(double value, std::int64_t& out) noexcept {
  // [-2^63, 2^63) is exactly representable as a lower / exclusive upper bound in double.
  constexpr double lower = -9223372036854775808.0;
  constexpr double upper = 9223372036854775808.0;
  if (!(value >= lower && value < upper)) {
    return false;
  }
  out = static_cast<std::int64_t>(value);
  return true;
}

/**
 * @brief Cell range overlapped by an AABB
 *
 *  Cells are half-open: a range ending exactly on a cell boundary does not include the next cell. A degenerate range
 *  (min == max) still includes the cell containing it.
 *
 * @param origin World position of cell (0, 0, 0)
 * @param unit Cell size
 * @param range Query range
 * @param lo Output first cell
 * @param hi Output last cell
 * @return bool, false for an invalid or non-finite range
 */
inline bool cell_range(
    const glm::vec3& origin, const glm::vec3& unit, const geometry::aabb& range, cell_vec& lo, cell_vec& hi) noexcept {
  if (!range.is_valid()) {
    return false;
  }

  const glm::dvec3 min_d = (cell_to_dvec3(range.min()) - cell_to_dvec3(origin)) / cell_to_dvec3(unit);
  const glm::dvec3 max_d = (cell_to_dvec3(range.max()) - cell_to_dvec3(origin)) / cell_to_dvec3(unit);
  if (!std::isfinite(min_d.x) || !std::isfinite(min_d.y) || !std::isfinite(min_d.z) || !std::isfinite(max_d.x) ||
      !std::isfinite(max_d.y) || !std::isfinite(max_d.z)) {
    return false;
  }

  glm::dvec3 lo_d = glm::floor(min_d);
  glm::dvec3 hi_d = glm::floor(max_d);
  for (int axis = 0; axis < 3; ++axis) {
    if (hi_d[axis] > lo_d[axis] && max_d[axis] == hi_d[axis]) {
      hi_d[axis] -= 1.0;
    }
  }

  return cell_double_to_i64(lo_d.x, lo.x) && cell_double_to_i64(lo_d.y, lo.y) && cell_double_to_i64(lo_d.z, lo.z) &&
         cell_double_to_i64(hi_d.x, hi.x) && cell_double_to_i64(hi_d.y, hi.y) && cell_double_to_i64(hi_d.z, hi.z);
}

/**
 * @brief World-space center of a cell
 *
 * @param origin World position of cell (0, 0, 0)
 * @param unit Cell size
 * @param cell Cell coordinate
 * @return glm::vec3
 */
inline glm::vec3 cell_center(const glm::vec3& origin, const glm::vec3& unit, const cell_vec& cell) noexcept {
  const glm::dvec3 center =
      cell_to_dvec3(origin) + (cell_to_dvec3(cell) + glm::dvec3(0.5, 0.5, 0.5)) * cell_to_dvec3(unit);
  return glm::vec3(center);
}

/**
 * @brief World-space box of a cell range
 *
 * @param origin World position of cell (0, 0, 0)
 * @param unit Cell size
 * @param cell First cell
 * @param span Number of cells per axis
 * @return geometry::aabb, invalid when the box is not finite
 */
inline geometry::aabb
cell_box(const glm::vec3& origin, const glm::vec3& unit, const cell_vec& cell, const cell_vec& span) noexcept {
  const glm::dvec3 origin_d = cell_to_dvec3(origin);
  const glm::dvec3 unit_d = cell_to_dvec3(unit);
  const glm::dvec3 min_d = origin_d + cell_to_dvec3(cell) * unit_d;
  const glm::dvec3 max_d = origin_d + (cell_to_dvec3(cell) + cell_to_dvec3(span)) * unit_d;
  const glm::vec3 min_f(min_d);
  const glm::vec3 max_f(max_d);
  if (!cell_is_finite(min_f) || !cell_is_finite(max_f)) {
    return {};
  }
  return geometry::aabb(min_f, max_f);
}

} // namespace detail
} // namespace spatial_index
} // namespace cyberverse
