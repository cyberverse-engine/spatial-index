/**
 * @file type_traits.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Type traits
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <concepts>
#include <utility>

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/geometry/intersect.hpp"

namespace cyberverse {
namespace spatial_index {

namespace detail {

/**
 * @brief Visitor used to check the query overload set of a partition index
 *
 */
struct query_probe {
  template <typename Value>
  bool operator()(Value&&, const geometry::aabb&) const noexcept {
    return true;
  }
};

} // namespace detail

/**
 * @brief Common interface of a coarse partition index
 *
 *  Covers the interface shared by every partition index (uniform grids today, spatial hashes and BVHs later). Storage
 *  occupancy and other implementation specific statistics are intentionally not part of the concept.
 *
 * @tparam T Candidate type
 */
template <typename T>
concept PartitionIndexType = requires(T t,
                                      const T& ct,
                                      const glm::vec3& pos,
                                      const geometry::aabb& box,
                                      const geometry::ray& ry,
                                      const geometry::frustum& fr,
                                      detail::query_probe probe) {
  typename T::value_type;

  { ct.get(pos) } noexcept -> std::same_as<const typename T::value_type*>;
  { t.get(pos) } noexcept -> std::same_as<typename T::value_type*>;
  { ct.contains(pos) } noexcept -> std::same_as<bool>;

  { t.ensure(pos) } -> std::same_as<typename T::value_type&>;
  { t.set(pos, std::declval<typename T::value_type>()) };
  { t.set(box, std::declval<const typename T::value_type&>()) };

  { t.erase(pos) } noexcept -> std::same_as<bool>;
  { t.erase(box) } noexcept -> std::same_as<bool>;
  { t.clear() } noexcept;
  { ct.empty() } noexcept -> std::same_as<bool>;

  { ct.query(ry, probe) };
  { t.query(ry, probe) };
  { ct.query(box, probe) };
  { t.query(box, probe) };
  { ct.query(fr, probe) };
  { t.query(fr, probe) };
};

} // namespace spatial_index
} // namespace cyberverse
