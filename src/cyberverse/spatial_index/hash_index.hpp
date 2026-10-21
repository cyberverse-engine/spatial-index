/**
 * @file hash_index.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Spatial hash entry index
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
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <glm/vec3.hpp>

#include "cyberverse/spatial_index/detail/cell_range.hpp"

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/spatial_index/entry_query_scratch.hpp"
#include "cyberverse/spatial_index/linear_index.hpp"
#include "cyberverse/spatial_index/type_traits.hpp"

namespace cyberverse {
namespace spatial_index {

/**
 * @brief Spatial hash over entry AABBs
 *
 *  Entries are inserted into every uniform cell overlapped by their AABB. Cell buckets are linear entry indexes. The
 *  index is a direct EntryIndexType and does not model CellIndexType.
 *
 *  Queries are const and use a local query scratch; they are safe to run concurrently as long as writes are
 *  externally synchronized.
 *
 * @tparam Handle Entry handle type
 * @tparam EntryValue Optional per-entry user value
 */
template <EntryHandleType Handle, typename EntryValue = std::monostate>
  requires std::default_initializable<EntryValue>
class hash_index {
public:
  using entry_handle_type = Handle;
  using entry_value_type = EntryValue;
  using bucket_type = linear_index<entry_handle_type, entry_value_type>;
  using query_scratch_type = entry_query_scratch<entry_handle_type>;

  explicit hash_index(glm::vec3 unit, glm::vec3 origin = glm::vec3(0.0f));

  bool insert(const entry_handle_type& handle, const geometry::aabb& bounds, const entry_value_type& value = {});
  bool update(const entry_handle_type& handle, const geometry::aabb& bounds);
  bool set_value(const entry_handle_type& handle, const entry_value_type& value);
  bool erase(const entry_handle_type& handle);

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

  std::size_t size() const noexcept { return records_.size(); }
  bool empty() const noexcept { return records_.empty(); }
  void clear() noexcept;

  const glm::vec3& unit() const noexcept { return unit_; }
  const glm::vec3& origin() const noexcept { return origin_; }

private:
  struct cell_key {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;
    friend bool operator==(const cell_key&, const cell_key&) noexcept = default;
  };

  struct cell_key_hash {
    std::size_t operator()(const cell_key& key) const noexcept;
  };

  struct record {
    geometry::aabb bounds;
    entry_value_type value;
    bool multi_cell = false;
  };

  static bool valid_bounds(const geometry::aabb& bounds) noexcept;
  bool to_cell_range(const geometry::aabb& bounds, cell_key& lo, cell_key& hi) const noexcept;

  void insert_into_cells(const entry_handle_type& handle, const geometry::aabb& bounds, const entry_value_type& value);
  void remove_from_cells(const entry_handle_type& handle, const geometry::aabb& bounds) noexcept;

  template <typename Filter, typename Visitor>
  bool visit(const entry_handle_type& handle, query_scratch_type& scratch, Filter& filter, Visitor& visitor) const;

  template <typename Filter, typename Visitor>
  void query_aabb_impl(const geometry::aabb& range, Filter& filter, Visitor& visitor) const;

  template <typename Filter, typename Visitor>
  void query_sphere_impl(const geometry::bounding_sphere& sphere, Filter& filter, Visitor& visitor) const;

  template <typename Filter, typename Visitor>
  void query_ray_impl(const geometry::ray& ry, Filter& filter, Visitor& visitor) const;

  template <typename Filter, typename Visitor>
  void query_frustum_impl(const geometry::frustum& fr, Filter& filter, Visitor& visitor) const;

  glm::vec3 unit_;
  glm::vec3 origin_;
  std::unordered_map<cell_key, bucket_type, cell_key_hash> cells_;
  std::unordered_map<entry_handle_type, record> records_;
};

} // namespace spatial_index
} // namespace cyberverse

#include "cyberverse/spatial_index/hash_index.inl"
