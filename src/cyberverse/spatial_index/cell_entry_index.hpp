/**
 * @file cell_entry_index.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Entry index built from a cell index and per-cell entry buckets
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
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/geometry/intersect.hpp"
#include "cyberverse/spatial_index/entry_query_scratch.hpp"
#include "cyberverse/spatial_index/type_traits.hpp"

namespace cyberverse {
namespace spatial_index {

/**
 * @brief Entry index over a cell index and a per-cell entry bucket
 *
 *  Entries are inserted into every cell overlapped by their AABB. The cell index stores one bucket per cell; the
 *  bucket stores entry handles, AABBs and optional values. A caller-owned query scratch is used to visit an entry at
 *  most once when it appears in multiple cells.
 *
 *  All queries are const and do not mutate the index. Writes require external synchronization.
 *
 * @tparam CellIndex Cell index type whose value_type satisfies EntryBucketType
 */
template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
class cell_entry_index {
public:
  using cell_index_type = CellIndex;
  using bucket_type = typename CellIndex::value_type;
  using entry_handle_type = typename bucket_type::entry_handle_type;
  using entry_value_type = typename bucket_type::entry_value_type;
  using cell_coord = typename CellIndex::cell_coord;
  using query_scratch_type = entry_query_scratch<entry_handle_type>;

  template <typename... Args>
    requires std::constructible_from<CellIndex, Args...>
  explicit cell_entry_index(Args&&... args);

  cell_entry_index(const cell_entry_index&) = delete;
  cell_entry_index(cell_entry_index&&) = delete;

  ~cell_entry_index() = default;

  cell_entry_index& operator=(const cell_entry_index&) = delete;
  cell_entry_index& operator=(cell_entry_index&&) = delete;

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
  void query(const geometry::aabb& range, query_scratch_type& scratch, Filter&& filter, Visitor&& visitor) const;

  template <typename Visitor>
    requires std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>
  void query(const geometry::aabb& range, query_scratch_type& scratch, Visitor&& visitor) const;

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
  void
  query(const geometry::bounding_sphere& sphere, query_scratch_type& scratch, Filter&& filter, Visitor&& visitor) const;

  template <typename Visitor>
    requires std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>
  void query(const geometry::bounding_sphere& sphere, query_scratch_type& scratch, Visitor&& visitor) const;

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
  void query(const geometry::ray& ry, query_scratch_type& scratch, Filter&& filter, Visitor&& visitor) const;

  template <typename Visitor>
    requires std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>
  void query(const geometry::ray& ry, query_scratch_type& scratch, Visitor&& visitor) const;

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
  void query(const geometry::frustum& fr, query_scratch_type& scratch, Filter&& filter, Visitor&& visitor) const;

  template <typename Visitor>
    requires std::predicate<Visitor&, const entry_handle_type&, const entry_value_type&, const geometry::aabb&>
  void query(const geometry::frustum& fr, query_scratch_type& scratch, Visitor&& visitor) const;

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
                                           query_scratch_type& scratch,
                                           Filter&& filter,
                                           float max_distance = std::numeric_limits<float>::infinity()) const;

  std::optional<entry_handle_type> nearest(const glm::vec3& position,
                                           query_scratch_type& scratch,
                                           float max_distance = std::numeric_limits<float>::infinity()) const;

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
                                           query_scratch_type& scratch,
                                           Filter&& filter,
                                           float max_distance = std::numeric_limits<float>::infinity()) const;

  std::vector<entry_handle_type> k_nearest(const glm::vec3& position,
                                           std::size_t k,
                                           query_scratch_type& scratch,
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
  raycast(const geometry::ray& ry,
          query_scratch_type& scratch,
          Filter&& filter,
          float max_distance = std::numeric_limits<float>::infinity()) const;

  std::optional<entry_ray_hit<entry_handle_type>>
  raycast(const geometry::ray& ry,
          query_scratch_type& scratch,
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

  CellIndex& cell_index() noexcept { return cell_index_; }
  const CellIndex& cell_index() const noexcept { return cell_index_; }

private:
  struct record {
    geometry::aabb bounds;
    entry_value_type value;
    bool multi_cell = false;
  };

  static bool valid_bounds(const geometry::aabb& bounds) noexcept;

  void insert_into_cells(const entry_handle_type& handle, const geometry::aabb& bounds, const entry_value_type& value);
  void remove_from_cells(const entry_handle_type& handle, const geometry::aabb& bounds) noexcept;

  template <typename Filter, typename Visitor>
  bool visit(const entry_handle_type& handle, query_scratch_type& scratch, Filter& filter, Visitor& visitor) const;

  template <typename Filter, typename Visitor>
  bool
  query_aabb_impl(const geometry::aabb& range, query_scratch_type& scratch, Filter& filter, Visitor& visitor) const;

  template <typename Filter, typename Visitor>
  bool query_sphere_impl(const geometry::bounding_sphere& sphere,
                         query_scratch_type& scratch,
                         Filter& filter,
                         Visitor& visitor) const;

  template <typename Filter, typename Visitor>
  bool query_ray_impl(const geometry::ray& ry, query_scratch_type& scratch, Filter& filter, Visitor& visitor) const;

  template <typename Filter, typename Visitor>
  bool
  query_frustum_impl(const geometry::frustum& fr, query_scratch_type& scratch, Filter& filter, Visitor& visitor) const;

  template <typename Filter>
  std::optional<entry_handle_type>
  nearest_impl(const glm::vec3& position, query_scratch_type& scratch, Filter& filter, float max_distance) const;

  template <typename Filter>
  std::vector<entry_handle_type> k_nearest_impl(
      const glm::vec3& position, std::size_t k, query_scratch_type& scratch, Filter& filter, float max_distance) const;

  template <typename Filter>
  std::optional<entry_ray_hit<entry_handle_type>>
  raycast_impl(const geometry::ray& ry, query_scratch_type& scratch, Filter& filter, float max_distance) const;

  CellIndex cell_index_;
  std::unordered_map<entry_handle_type, record> records_;
};

} // namespace spatial_index
} // namespace cyberverse

#include "cyberverse/spatial_index/cell_entry_index.inl"
