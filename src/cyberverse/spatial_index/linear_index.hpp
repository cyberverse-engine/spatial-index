/**
 * @file linear_index.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Linear entry index
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

#include <glm/vec3.hpp>

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/geometry/intersect.hpp"
#include "cyberverse/spatial_index/type_traits.hpp"

namespace cyberverse {
namespace spatial_index {

/**
 * @brief Sequential entry index
 *
 *  Stores entries in a single vector and answers every query by scanning. It is primarily useful as a cell bucket
 *  and as a simple reference implementation of EntryIndexType.
 *
 * @tparam Handle Entry handle type
 * @tparam EntryValue Optional per-entry user value
 */
template <EntryHandleType Handle, typename EntryValue = std::monostate>
  requires std::default_initializable<EntryValue>
class linear_index {
public:
  using entry_handle_type = Handle;
  using entry_value_type = EntryValue;

  linear_index() = default;

  bool insert(const entry_handle_type& handle, const geometry::aabb& bounds, const entry_value_type& value = {});
  bool update(const entry_handle_type& handle, const geometry::aabb& bounds);
  bool set_value(const entry_handle_type& handle, const entry_value_type& value);
  bool erase(const entry_handle_type& handle) noexcept;
  bool contains(const entry_handle_type& handle) const noexcept;

  const geometry::aabb* bounds(const entry_handle_type& handle) const noexcept;
  const entry_value_type* value(const entry_handle_type& handle) const noexcept;

  template <typename Filter, typename Visitor>
    requires(std::predicate<Filter&, const entry_handle_type&, const entry_value_type&> &&
             std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>)
  void query(const geometry::aabb& range, Filter&& filter, Visitor&& visitor) const;

  template <typename Visitor>
    requires std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>
  void query(const geometry::aabb& range, Visitor&& visitor) const;

  template <typename Filter, typename Visitor>
    requires(std::predicate<Filter&, const entry_handle_type&, const entry_value_type&> &&
             std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>)
  void query(const geometry::bounding_sphere& sphere, Filter&& filter, Visitor&& visitor) const;

  template <typename Visitor>
    requires std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>
  void query(const geometry::bounding_sphere& sphere, Visitor&& visitor) const;

  template <typename Filter, typename Visitor>
    requires(std::predicate<Filter&, const entry_handle_type&, const entry_value_type&> &&
             std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>)
  void query(const geometry::ray& ry, Filter&& filter, Visitor&& visitor) const;

  template <typename Visitor>
    requires std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>
  void query(const geometry::ray& ry, Visitor&& visitor) const;

  template <typename Filter, typename Visitor>
    requires(std::predicate<Filter&, const entry_handle_type&, const entry_value_type&> &&
             std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>)
  void query(const geometry::frustum& fr, Filter&& filter, Visitor&& visitor) const;

  template <typename Visitor>
    requires std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>
  void query(const geometry::frustum& fr, Visitor&& visitor) const;

  template <typename Filter>
    requires std::predicate<Filter&, const entry_handle_type&, const entry_value_type&>
  std::optional<entry_handle_type> nearest(const glm::vec3& position,
                                           Filter&& filter,
                                           float max_distance = std::numeric_limits<float>::infinity()) const;

  std::optional<entry_handle_type> nearest(const glm::vec3& position,
                                           float max_distance = std::numeric_limits<float>::infinity()) const;

  template <typename Filter>
    requires std::predicate<Filter&, const entry_handle_type&, const entry_value_type&>
  std::vector<entry_handle_type> k_nearest(const glm::vec3& position,
                                           std::size_t k,
                                           Filter&& filter,
                                           float max_distance = std::numeric_limits<float>::infinity()) const;

  std::vector<entry_handle_type> k_nearest(const glm::vec3& position,
                                           std::size_t k,
                                           float max_distance = std::numeric_limits<float>::infinity()) const;

  template <typename Filter>
    requires std::predicate<Filter&, const entry_handle_type&, const entry_value_type&>
  std::optional<entry_ray_hit<entry_handle_type>>
  raycast(const geometry::ray& ry, Filter&& filter, float max_distance = std::numeric_limits<float>::infinity()) const;

  std::optional<entry_ray_hit<entry_handle_type>>
  raycast(const geometry::ray& ry, float max_distance = std::numeric_limits<float>::infinity()) const;

  std::size_t size() const noexcept { return entries_.size(); }
  bool empty() const noexcept { return entries_.empty(); }
  void clear() noexcept { entries_.clear(); }

private:
  struct entry {
    entry_handle_type handle;
    geometry::aabb bounds;
    entry_value_type value;
  };

  static bool valid_bounds(const geometry::aabb& bounds) noexcept;

  entry* find(const entry_handle_type& handle) noexcept;
  const entry* find(const entry_handle_type& handle) const noexcept;

  std::vector<entry> entries_;
};

} // namespace spatial_index
} // namespace cyberverse

#include "cyberverse/spatial_index/linear_index.inl"
