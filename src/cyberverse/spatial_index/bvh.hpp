/**
 * @file bvh.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Dynamic AABB BVH entry index
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

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/spatial_index/type_traits.hpp"

namespace cyberverse {
namespace spatial_index {

/**
 * @brief Dynamic binary AABB tree over entries
 *
 *  The BVH directly implements EntryIndexType and intentionally does not model CellIndexType. Leaf layout and other
 *  tree-quality optimizations are an implementation detail.
 *
 *  Queries are const and do not mutate the tree.
 *
 * @tparam Handle Entry handle type
 * @tparam EntryValue Optional per-entry user value
 */
template <EntryHandleType Handle, typename EntryValue = std::monostate>
  requires std::default_initializable<EntryValue>
class bvh {
public:
  using entry_handle_type = Handle;
  using entry_value_type = EntryValue;

  bvh() = default;
  bvh(const bvh&) = delete;
  bvh(bvh&&) = delete;
  ~bvh() = default;
  bvh& operator=(const bvh&) = delete;
  bvh& operator=(bvh&&) = delete;

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

  std::size_t size() const noexcept { return leaf_of_.size(); }
  bool empty() const noexcept { return leaf_of_.empty(); }
  void clear() noexcept;

private:
  static constexpr std::uint32_t invalid = std::numeric_limits<std::uint32_t>::max();

  struct entry {
    entry_handle_type handle;
    geometry::aabb bounds;
    entry_value_type value;
  };

  struct node {
    geometry::aabb bounds;
    std::uint32_t parent = invalid;
    std::uint32_t left = invalid;
    std::uint32_t right = invalid;
    std::optional<entry> data;
  };

  static bool valid_bounds(const geometry::aabb& bounds) noexcept;
  static float area(const geometry::aabb& bounds) noexcept;
  static geometry::aabb merge(const geometry::aabb& a, const geometry::aabb& b) noexcept;

  std::uint32_t allocate_node();
  void release_node(std::uint32_t index) noexcept;
  void refit(std::uint32_t index) noexcept;
  void refit_up(std::uint32_t index) noexcept;

  template <typename Predicate, typename Filter, typename Visitor>
  bool query_node(std::uint32_t index, Predicate& predicate, Filter& filter, Visitor& visitor) const;

  template <typename Filter>
  void nearest_node(std::uint32_t index,
                    const glm::vec3& position,
                    Filter& filter,
                    std::optional<entry_handle_type>& best,
                    float& best_distance_sq) const;

  template <typename Filter>
  void k_nearest_node(std::uint32_t index,
                      const glm::vec3& position,
                      Filter& filter,
                      float max_distance_sq,
                      std::vector<std::pair<float, entry_handle_type>>& candidates) const;

  template <typename Filter>
  void raycast_node(std::uint32_t index,
                    const geometry::ray& ry,
                    Filter& filter,
                    float max_distance,
                    std::optional<entry_ray_hit<entry_handle_type>>& best,
                    float& best_distance) const;

  std::vector<node> nodes_;
  std::vector<std::uint32_t> free_nodes_;
  std::unordered_map<entry_handle_type, std::uint32_t> leaf_of_;
  std::uint32_t root_ = invalid;
};

} // namespace spatial_index
} // namespace cyberverse

#include "cyberverse/spatial_index/bvh.inl"
