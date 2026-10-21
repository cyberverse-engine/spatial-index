/**
 * @file type_traits.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Spatial index concepts
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/geometry/intersect.hpp"

namespace cyberverse {
namespace spatial_index {

/**
 * @brief Entry handle requirements
 *
 *  A handle is an opaque, caller-owned key. The index never interprets its bits, never reuses it and never tracks a
 *  generation. The caller is responsible for uniqueness and for not reusing a stale handle.
 *
 *  Total ordering is required so k-nearest tie breaking is deterministic.
 *
 * @tparam H Candidate handle type
 */
template <typename H>
concept EntryHandleType = std::copyable<H> && std::default_initializable<H> && std::equality_comparable<H> &&
                          std::totally_ordered<H> && requires(const H& handle) {
                            { std::hash<H>{}(handle) } -> std::convertible_to<std::size_t>;
                          };

/**
 * @brief Ray hit returned by an entry index
 *
 * @tparam Handle Entry handle type
 */
template <EntryHandleType Handle>
struct entry_ray_hit {
  Handle handle;
  float distance = 0.0f;
};

namespace detail {

/**
 * @brief Visitor used to check cell query overloads
 *
 */
struct cell_query_probe {
  template <typename Value>
  bool operator()(const Value&, const geometry::aabb&) const noexcept {
    return true;
  }
};

/**
 * @brief Filter used to check entry query overloads
 *
 */
struct entry_filter_probe {
  template <typename Handle, typename Value>
  bool operator()(const Handle&, const Value&) const noexcept {
    return true;
  }
};

/**
 * @brief Visitor used to check entry query overloads
 *
 */
struct entry_visitor_probe {
  template <typename Handle, typename Value>
  bool operator()(const Handle&, const Value&, const geometry::aabb&) const noexcept {
    return true;
  }
};

} // namespace detail

/**
 * @brief Common local bucket interface of an entry index
 *
 *  This is a requirement subset of EntryIndexType: every type satisfying EntryIndexType also satisfies
 *  EntryBucketType. A bucket only needs local insert/erase/query operations; it does not need global bounds/value
 *  lookup or nearest/raycast.
 *
 * @tparam T Candidate type
 */
template <typename T>
concept EntryBucketType = requires(T t,
                                   const T& ct,
                                   typename T::entry_handle_type handle,
                                   const geometry::aabb& box,
                                   const geometry::bounding_sphere& sphere,
                                   const geometry::ray& ry,
                                   const geometry::frustum& fr,
                                   const typename T::entry_value_type& value,
                                   detail::entry_filter_probe filter,
                                   detail::entry_visitor_probe visitor) {
  typename T::entry_handle_type;
  typename T::entry_value_type;
  requires EntryHandleType<typename T::entry_handle_type>;

  { t.insert(handle, box, value) } -> std::same_as<bool>;
  { t.erase(handle) } -> std::same_as<bool>;
  { t.set_value(handle, value) } -> std::same_as<bool>;

  { ct.query(box, filter, visitor) };
  { ct.query(sphere, filter, visitor) };
  { ct.query(ry, filter, visitor) };
  { ct.query(fr, filter, visitor) };

  { ct.size() } -> std::same_as<std::size_t>;
  { ct.empty() } noexcept -> std::same_as<bool>;
  { t.clear() } noexcept;
};

/**
 * @brief Spatial index over (entry_handle, AABB, entry_value)
 *
 *  Every query is const and does not mutate the index. Implementations may use a caller-owned query scratch for
 *  handle deduplication, but the scratch is never stored in the index.
 *
 * @tparam T Candidate type
 */
template <typename T>
concept EntryIndexType = EntryBucketType<T> && requires(T t,
                                                        const T& ct,
                                                        typename T::entry_handle_type handle,
                                                        const geometry::aabb& box,
                                                        const glm::vec3& position,
                                                        const geometry::ray& ry,
                                                        detail::entry_filter_probe filter) {
  { t.update(handle, box) } -> std::same_as<bool>;
  { t.contains(handle) } -> std::same_as<bool>;
  { ct.bounds(handle) } -> std::same_as<const geometry::aabb*>;
  { ct.value(handle) } -> std::same_as<const typename T::entry_value_type*>;

  { ct.nearest(position, filter, float{}) } -> std::same_as<std::optional<typename T::entry_handle_type>>;
  {
    ct.k_nearest(position, std::size_t{}, filter, float{})
  } -> std::same_as<std::vector<typename T::entry_handle_type>>;
  { ct.raycast(ry, filter, float{}) } -> std::same_as<std::optional<entry_ray_hit<typename T::entry_handle_type>>>;
};

/**
 * @brief Cell-based partition index
 *
 *  A cell index stores one user value per integer cell. It does not require the value to have an AABB; entry indices
 *  built on top of it (cell_entry_index) handle AABB coverage and entry-level queries.
 *
 * @tparam T Candidate type
 */
template <typename T>
concept CellIndexType = requires(T t,
                                 const T& ct,
                                 typename T::cell_coord cell,
                                 const geometry::aabb& box,
                                 const geometry::ray& ry,
                                 const geometry::frustum& fr,
                                 detail::cell_query_probe visitor) {
  typename T::value_type;
  typename T::cell_coord;

  { ct.cell_range(box, cell, cell) } -> std::same_as<bool>;

  { ct.get_cell(cell) } -> std::same_as<const typename T::value_type*>;
  { t.get_cell(cell) } -> std::same_as<typename T::value_type*>;
  { t.ensure_cell(cell) } -> std::same_as<typename T::value_type&>;
  { t.erase_cell(cell) } noexcept -> std::same_as<bool>;
  { ct.contains_cell(cell) } noexcept -> std::same_as<bool>;

  { ct.query(box, visitor) };
  { ct.query(ry, visitor) };
  { ct.query(fr, visitor) };

  { ct.empty() } noexcept -> std::same_as<bool>;
  { t.clear() } noexcept;
};

} // namespace spatial_index
} // namespace cyberverse
