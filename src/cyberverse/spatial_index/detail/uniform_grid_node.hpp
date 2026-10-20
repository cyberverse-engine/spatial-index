/**
 * @file uniform_grid_node.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Uniform grid node both for sparse / dense grid
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cassert>
#include <cstddef>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/spatial_index/detail/keyed_array.hpp"

namespace cyberverse {
namespace spatial_index {
namespace detail {

template <typename T, bool Sparse>
class uniform_grid_node;

template <typename T, bool Sparse>
class uniform_grid_node_collection;

/**
 * @brief Uniform grid node
 *
 *  A node represents a sub-box of the grid. Leaf nodes are exactly one unit cell and may own a value; internal nodes
 * own up to `branch_factor.x * branch_factor.y * branch_factor.z` children. The child collection is allocated lazily,
 * so an empty leaf costs only the node itself (this matters for the sparse grid, where a single stored value must not
 * pull in a full child array).
 *
 *  The node is move-only: the child collection is held behind a `std::unique_ptr`, which also breaks the recursive
 * type dependency between a node and its collection.
 *
 * @tparam T Value type
 * @tparam Sparse Sparse or not
 */
template <typename T, bool Sparse>
class uniform_grid_node {
public:
  using value_type = T;
  using collection_type = uniform_grid_node_collection<T, Sparse>;

  /**
   * @brief Construct an empty (non-existing) node
   *
   */
  uniform_grid_node() noexcept = default;

  /**
   * @brief Construct a node
   *
   * @param box Box represented by the node
   * @param is_leaf Whether the box is a unit cell, i.e. the node can own a value
   * @param child_capacity Number of children slots of the node (`branch_factor` product)
   */
  uniform_grid_node(const geometry::aabb& box, bool is_leaf, size_t child_capacity);

  ~uniform_grid_node();

  uniform_grid_node(const uniform_grid_node&) = delete;
  uniform_grid_node(uniform_grid_node&&);

  uniform_grid_node& operator=(const uniform_grid_node&) = delete;
  uniform_grid_node& operator=(uniform_grid_node&&);

  /**
   * @brief Get the box of current node
   *
   * @return const geometry::aabb&
   */
  const geometry::aabb& box() const noexcept { return box_; }

  /**
   * @brief Whether the node is materialized
   *
   * @return true when the node has a valid box
   */
  bool exists() const noexcept { return box_.is_valid(); }

  /**
   * @brief Whether the node represents a single unit cell and can own a value
   *
   * @return bool
   */
  bool is_leaf() const noexcept { return leaf_; }

  /**
   * @brief Whether the node owns a value
   *
   * @return bool
   */
  bool has_value() const noexcept { return value_.has_value(); }

  /**
   * @brief Get the value
   *
   * @return const value_type*, nullptr when the node owns no value
   */
  const value_type* value() const noexcept { return value_ ? std::addressof(*value_) : nullptr; }

  /**
   * @brief Get the value
   *
   * @return value_type*, nullptr when the node owns no value
   */
  value_type* value() noexcept { return value_ ? std::addressof(*value_) : nullptr; }

  /**
   * @brief Ensure a value, constructing it with its default constructor when absent
   *
   * @return value_type&
   */
  value_type& ensure_value() {
    if (!value_) {
      value_.emplace();
    }
    return *value_;
  }

  /**
   * @brief Construct or overwrite the value
   *
   * @tparam Args Constructor / assignment argument types
   * @param args Arguments forwarded to the value constructor
   * @return value_type&
   */
  template <typename... Args>
  value_type& set_value(Args&&... args) {
    if (value_) {
      *value_ = value_type(std::forward<Args>(args)...);
    } else {
      value_.emplace(std::forward<Args>(args)...);
    }
    return *value_;
  }

  /**
   * @brief Erase the value
   *
   * @return bool, actually erased or not
   */
  bool erase_value() noexcept {
    if (!value_) {
      return false;
    }
    value_.reset();
    return true;
  }

  /**
   * @brief Whether the node owns neither a value nor a child
   *
   * @return bool
   */
  bool empty() const noexcept;

  /**
   * @brief Destroy the value and every child, then make the node non-existing
   *
   */
  void clear() noexcept;

  /**
   * @brief Reinitialize the node, releasing every previous value and child
   *
   * @param box Box represented by the node
   * @param is_leaf Whether the box is a unit cell
   * @param child_capacity Number of children slots
   */
  void reset(const geometry::aabb& box, bool is_leaf, size_t child_capacity) noexcept;

  /**
   * @brief Number of materialized children
   *
   * @return size_t
   */
  size_t child_count() const noexcept;

  /**
   * @brief Whether the node owns at least one child
   *
   * @return bool
   */
  bool has_children() const noexcept;

  /**
   * @brief Find a child by its flat child index
   *
   * @param index Flat child index
   * @return const uniform_grid_node*, nullptr when the child does not exist
   */
  const uniform_grid_node* child(size_t index) const noexcept;

  /**
   * @brief Find a child by its flat child index
   *
   * @param index Flat child index
   * @return uniform_grid_node*, nullptr when the child does not exist
   */
  uniform_grid_node* child(size_t index) noexcept;

  /**
   * @brief Ensure a child, creating it when absent
   *
   * @param index Flat child index
   * @param box Box represented by the child
   * @param is_leaf Whether the child is a unit cell
   * @return uniform_grid_node&
   */
  uniform_grid_node& ensure_child(size_t index, const geometry::aabb& box, bool is_leaf);

  /**
   * @brief Insert or replace a child
   *
   * @param index Flat child index
   * @param node Child to move in
   * @return uniform_grid_node&
   */
  uniform_grid_node& insert_child(size_t index, uniform_grid_node&& node);

  /**
   * @brief Erase a child
   *
   * @param index Flat child index
   * @return bool, actually erased or not
   */
  bool erase_child(size_t index) noexcept;

  /**
   * @brief Visit every materialized child
   *
   *  The visit order is implementation defined (child-index order for the dense collection, inline entries followed
   *  by hash-map entries for the sparse collection). The visit function returns false to stop the iteration.
   *
   * @tparam F Visit function, `bool(size_t, const uniform_grid_node&)`
   * @param f Visit function
   */
  template <typename F>
  void for_each_child(F&& f) const noexcept;

  /**
   * @brief Visit every materialized child
   *
   *  The visit order is implementation defined (child-index order for the dense collection, inline entries followed
   *  by hash-map entries for the sparse collection). The visit function returns false to stop the iteration.
   *
   * @tparam F Visit function, `bool(size_t, uniform_grid_node&)`
   * @param f Visit function
   */
  template <typename F>
  void for_each_child(F&& f) noexcept;

  /**
   * @brief Memory footprint of the node and its subtree
   *
   *  Includes the node object itself, its child collection and all descendants. The value is exact for the dense
   *  collection and an approximation for the sparse hash-map representation.
   *
   * @return size_t
   */
  size_t memory_usage_in_bytes() const noexcept;

  /**
   * @brief Memory footprint allocated on the heap by the node and its subtree
   *
   *  Same as memory_usage_in_bytes() but excludes `sizeof(*this)`; used by the owner of the node object.
   *
   * @return size_t
   */
  size_t owned_memory_usage_in_bytes() const noexcept;

private:
  geometry::aabb box_;
  std::unique_ptr<collection_type> collection_; // Lazily allocated; null for leaves
  std::optional<value_type> value_;
  size_t child_capacity_ = 0;
  bool leaf_ = true;
};

/**
 * @brief Uniform grid node collection
 *
 *  The default implementation is the dense implementation: every child slot is materialized as an empty node and
 *  only the slots that are actually used are marked as existing.
 *
 * @tparam T Value type
 * @tparam Sparse Sparse or not
 */
template <typename T, bool Sparse>
class uniform_grid_node_collection {
public:
  using node_type = uniform_grid_node<T, Sparse>;

  /**
   * @brief Construct a collection with a fixed number of child slots
   *
   * @param capacity Number of child slots
   */
  explicit uniform_grid_node_collection(size_t capacity);

  void clear() noexcept;

  size_t size() const noexcept { return size_; }

  bool empty() const noexcept { return size_ == 0; }

  node_type* find(size_t index) noexcept;

  const node_type* find(size_t index) const noexcept;

  node_type& ensure(size_t index, const geometry::aabb& box, bool is_leaf);

  node_type& insert(size_t index, node_type&& node);

  bool erase(size_t index) noexcept;

  template <typename F>
  void for_each(F&& f) const noexcept;

  template <typename F>
  void for_each(F&& f) noexcept;

  size_t memory_usage_in_bytes() const noexcept;

private:
  std::vector<node_type> container_;
  size_t size_ = 0;
};

/**
 * @brief Uniform grid node collection for sparse node
 *
 *  The first `keyed_array::capacity` children are stored inline; once the inline array is full, all entries are
 *  promoted into a hash map. This keeps the common case (a handful of children) allocation free.
 *
 * @tparam T Value type
 */
template <typename T>
class uniform_grid_node_collection<T, true> {
public:
  using node_type = uniform_grid_node<T, true>;

  /**
   * @brief Construct a collection with a fixed number of child slots
   *
   * @param capacity Number of child slots
   */
  explicit uniform_grid_node_collection(size_t capacity);

  void clear() noexcept;

  size_t size() const noexcept { return array_.size() + map_.size(); }

  bool empty() const noexcept { return size() == 0; }

  node_type* find(size_t index) noexcept;

  const node_type* find(size_t index) const noexcept;

  node_type& ensure(size_t index, const geometry::aabb& box, bool is_leaf);

  node_type& insert(size_t index, node_type&& node);

  bool erase(size_t index) noexcept;

  template <typename F>
  void for_each(F&& f) const noexcept;

  template <typename F>
  void for_each(F&& f) noexcept;

  size_t memory_usage_in_bytes() const noexcept;

private:
  void promote_to_map();

  using array_type = keyed_array<size_t, node_type>;
  using map_type = std::unordered_map<size_t, node_type>;

  array_type array_; // soo optimization, stores value in array at first, additional values will be stored in map_
  map_type map_;
  size_t capacity_ = 0;
};

} // namespace detail
} // namespace spatial_index
} // namespace cyberverse

#include "cyberverse/spatial_index/detail/uniform_grid_node.inl"
