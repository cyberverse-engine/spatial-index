/**
 * @file bvh.inl
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Dynamic AABB BVH implementation
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <variant>

#include "cyberverse/spatial_index/bvh.hpp"

namespace cyberverse {
namespace spatial_index {

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool bvh<Handle, EntryValue>::valid_bounds(const geometry::aabb& bounds) noexcept {
  return bounds.is_valid() && std::isfinite(bounds.min().x) && std::isfinite(bounds.min().y) &&
         std::isfinite(bounds.min().z) && std::isfinite(bounds.max().x) && std::isfinite(bounds.max().y) &&
         std::isfinite(bounds.max().z);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
float bvh<Handle, EntryValue>::area(const geometry::aabb& bounds) noexcept {
  if (!bounds.is_valid()) {
    return 0.0f;
  }
  const glm::vec3 extent = bounds.max() - bounds.min();
  return 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
geometry::aabb bvh<Handle, EntryValue>::merge(const geometry::aabb& a, const geometry::aabb& b) noexcept {
  return geometry::aabb(glm::min(a.min(), b.min()), glm::max(a.max(), b.max()));
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
std::uint32_t bvh<Handle, EntryValue>::allocate_node() {
  if (!free_nodes_.empty()) {
    const std::uint32_t index = free_nodes_.back();
    free_nodes_.pop_back();
    nodes_[index] = node{};
    return index;
  }
  nodes_.emplace_back();
  return static_cast<std::uint32_t>(nodes_.size() - 1);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
void bvh<Handle, EntryValue>::release_node(std::uint32_t index) noexcept {
  nodes_[index] = node{};
  free_nodes_.push_back(index);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
void bvh<Handle, EntryValue>::refit(std::uint32_t index) noexcept {
  node& n = nodes_[index];
  if (n.data) {
    n.bounds = n.data->bounds;
    return;
  }
  if (n.left == invalid || n.right == invalid) {
    n.bounds = geometry::aabb{};
    return;
  }
  n.bounds = merge(nodes_[n.left].bounds, nodes_[n.right].bounds);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
void bvh<Handle, EntryValue>::refit_up(std::uint32_t index) noexcept {
  while (index != invalid) {
    refit(index);
    index = nodes_[index].parent;
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool bvh<Handle, EntryValue>::insert(const entry_handle_type& handle,
                                     const geometry::aabb& bounds,
                                     const entry_value_type& value) {
  if (!valid_bounds(bounds)) {
    throw std::invalid_argument("bvh entry bounds must be valid and finite");
  }
  if (leaf_of_.contains(handle)) {
    return false;
  }

  const std::uint32_t leaf = allocate_node();
  nodes_[leaf].data = entry{ handle, bounds, value };
  nodes_[leaf].bounds = bounds;

  if (root_ == invalid) {
    root_ = leaf;
    leaf_of_.emplace(handle, leaf);
    return true;
  }

  std::uint32_t sibling = root_;
  while (!nodes_[sibling].data) {
    const std::uint32_t left = nodes_[sibling].left;
    const std::uint32_t right = nodes_[sibling].right;
    const float left_cost = area(merge(nodes_[left].bounds, bounds));
    const float right_cost = area(merge(nodes_[right].bounds, bounds));
    sibling = left_cost <= right_cost ? left : right;
  }

  const std::uint32_t old_parent = nodes_[sibling].parent;
  const std::uint32_t parent = allocate_node();
  nodes_[parent].left = sibling;
  nodes_[parent].right = leaf;
  nodes_[parent].parent = old_parent;
  nodes_[parent].bounds = merge(nodes_[sibling].bounds, bounds);
  nodes_[sibling].parent = parent;
  nodes_[leaf].parent = parent;

  if (old_parent == invalid) {
    root_ = parent;
  } else if (nodes_[old_parent].left == sibling) {
    nodes_[old_parent].left = parent;
  } else {
    nodes_[old_parent].right = parent;
  }

  refit_up(parent);
  leaf_of_.emplace(handle, leaf);
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool bvh<Handle, EntryValue>::update(const entry_handle_type& handle, const geometry::aabb& bounds) {
  if (!valid_bounds(bounds)) {
    throw std::invalid_argument("bvh entry bounds must be valid and finite");
  }
  const auto it = leaf_of_.find(handle);
  if (it == leaf_of_.end()) {
    return false;
  }
  nodes_[it->second].data->bounds = bounds;
  refit_up(it->second);
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool bvh<Handle, EntryValue>::set_value(const entry_handle_type& handle, const entry_value_type& value) {
  const auto it = leaf_of_.find(handle);
  if (it == leaf_of_.end()) {
    return false;
  }
  nodes_[it->second].data->value = value;
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool bvh<Handle, EntryValue>::erase(const entry_handle_type& handle) {
  const auto it = leaf_of_.find(handle);
  if (it == leaf_of_.end()) {
    return false;
  }

  const std::uint32_t leaf = it->second;
  const std::uint32_t parent = nodes_[leaf].parent;
  if (parent == invalid) {
    root_ = invalid;
    release_node(leaf);
    leaf_of_.erase(it);
    return true;
  }

  const std::uint32_t grandparent = nodes_[parent].parent;
  const std::uint32_t sibling = nodes_[parent].left == leaf ? nodes_[parent].right : nodes_[parent].left;
  nodes_[sibling].parent = grandparent;

  if (grandparent == invalid) {
    root_ = sibling;
  } else if (nodes_[grandparent].left == parent) {
    nodes_[grandparent].left = sibling;
  } else {
    nodes_[grandparent].right = sibling;
  }

  release_node(leaf);
  release_node(parent);
  leaf_of_.erase(it);
  refit_up(grandparent);
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool bvh<Handle, EntryValue>::contains(const entry_handle_type& handle) const noexcept {
  return leaf_of_.contains(handle);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
const geometry::aabb* bvh<Handle, EntryValue>::bounds(const entry_handle_type& handle) const noexcept {
  const auto it = leaf_of_.find(handle);
  return it == leaf_of_.end() ? nullptr : &nodes_[it->second].data->bounds;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
const typename bvh<Handle, EntryValue>::entry_value_type*
bvh<Handle, EntryValue>::value(const entry_handle_type& handle) const noexcept {
  const auto it = leaf_of_.find(handle);
  return it == leaf_of_.end() ? nullptr : &nodes_[it->second].data->value;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Predicate, typename Filter, typename Visitor>
bool bvh<Handle, EntryValue>::query_node(std::uint32_t index,
                                         Predicate& predicate,
                                         Filter& filter,
                                         Visitor& visitor) const {
  if (index == invalid || !predicate(nodes_[index].bounds)) {
    return true;
  }
  const node& n = nodes_[index];
  if (n.data) {
    if (!filter(n.data->handle, n.data->value)) {
      return true;
    }
    return visitor(n.data->handle, n.data->value, n.data->bounds);
  }
  if (!query_node(n.left, predicate, filter, visitor)) {
    return false;
  }
  return query_node(n.right, predicate, filter, visitor);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&, const Handle&, const EntryValue&> &&
           std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>)
void bvh<Handle, EntryValue>::query(const geometry::aabb& range, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  auto predicate = [&](const geometry::aabb& bounds) {
    return geometry::aabb_overlaps(bounds, range);
  };
  query_node(root_, predicate, filter_ref, visitor_ref);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void bvh<Handle, EntryValue>::query(const geometry::aabb& range, Visitor&& visitor) const {
  query(
      range,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      std::forward<Visitor>(visitor));
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&, const Handle&, const EntryValue&> &&
           std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>)
void bvh<Handle, EntryValue>::query(const geometry::bounding_sphere& sphere, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  auto predicate = [&](const geometry::aabb& bounds) {
    return geometry::sphere_overlaps_aabb(sphere, bounds);
  };
  query_node(root_, predicate, filter_ref, visitor_ref);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void bvh<Handle, EntryValue>::query(const geometry::bounding_sphere& sphere, Visitor&& visitor) const {
  query(
      sphere,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      std::forward<Visitor>(visitor));
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&, const Handle&, const EntryValue&> &&
           std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>)
void bvh<Handle, EntryValue>::query(const geometry::ray& ry, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  auto predicate = [&](const geometry::aabb& bounds) {
    float distance = 0.0f;
    return geometry::ray_aabb(ry, bounds, distance);
  };
  query_node(root_, predicate, filter_ref, visitor_ref);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void bvh<Handle, EntryValue>::query(const geometry::ray& ry, Visitor&& visitor) const {
  query(
      ry,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      std::forward<Visitor>(visitor));
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&, const Handle&, const EntryValue&> &&
           std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>)
void bvh<Handle, EntryValue>::query(const geometry::frustum& fr, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  auto predicate = [&](const geometry::aabb& bounds) {
    return !geometry::is_outside(fr, bounds);
  };
  query_node(root_, predicate, filter_ref, visitor_ref);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void bvh<Handle, EntryValue>::query(const geometry::frustum& fr, Visitor&& visitor) const {
  query(
      fr,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      std::forward<Visitor>(visitor));
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter>
void bvh<Handle, EntryValue>::nearest_node(std::uint32_t index,
                                           const glm::vec3& position,
                                           Filter& filter,
                                           std::optional<entry_handle_type>& best,
                                           float& best_distance_sq) const {
  if (index == invalid) {
    return;
  }
  if (geometry::point_aabb_distance_sq(position, nodes_[index].bounds) > best_distance_sq) {
    return;
  }
  const node& n = nodes_[index];
  if (n.data) {
    if (!filter(n.data->handle, n.data->value)) {
      return;
    }
    const float distance_sq = geometry::point_aabb_distance_sq(position, n.data->bounds);
    if (!best || distance_sq < best_distance_sq || (distance_sq == best_distance_sq && n.data->handle < *best)) {
      best_distance_sq = distance_sq;
      best = n.data->handle;
    }
    return;
  }
  nearest_node(n.left, position, filter, best, best_distance_sq);
  nearest_node(n.right, position, filter, best, best_distance_sq);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter>
  requires std::predicate<Filter&, const Handle&, const EntryValue&>
std::optional<typename bvh<Handle, EntryValue>::entry_handle_type>
bvh<Handle, EntryValue>::nearest(const glm::vec3& position, Filter&& filter, float max_distance) const {
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      std::isnan(max_distance) || max_distance < 0.0f) {
    return std::nullopt;
  }
  std::optional<entry_handle_type> best;
  float best_distance_sq =
      std::isfinite(max_distance) ? max_distance * max_distance : std::numeric_limits<float>::infinity();
  auto& filter_ref = filter;
  nearest_node(root_, position, filter_ref, best, best_distance_sq);
  return best;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
std::optional<typename bvh<Handle, EntryValue>::entry_handle_type>
bvh<Handle, EntryValue>::nearest(const glm::vec3& position, float max_distance) const {
  return nearest(
      position,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      max_distance);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter>
void bvh<Handle, EntryValue>::k_nearest_node(std::uint32_t index,
                                             const glm::vec3& position,
                                             Filter& filter,
                                             float max_distance_sq,
                                             std::vector<std::pair<float, entry_handle_type>>& candidates) const {
  if (index == invalid || geometry::point_aabb_distance_sq(position, nodes_[index].bounds) > max_distance_sq) {
    return;
  }
  const node& n = nodes_[index];
  if (n.data) {
    if (!filter(n.data->handle, n.data->value)) {
      return;
    }
    const float distance_sq = geometry::point_aabb_distance_sq(position, n.data->bounds);
    if (distance_sq <= max_distance_sq) {
      candidates.emplace_back(distance_sq, n.data->handle);
    }
    return;
  }
  k_nearest_node(n.left, position, filter, max_distance_sq, candidates);
  k_nearest_node(n.right, position, filter, max_distance_sq, candidates);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter>
  requires std::predicate<Filter&, const Handle&, const EntryValue&>
std::vector<typename bvh<Handle, EntryValue>::entry_handle_type> bvh<Handle, EntryValue>::k_nearest(
    const glm::vec3& position, std::size_t k, Filter&& filter, float max_distance) const {
  std::vector<entry_handle_type> result;
  if (k == 0 || !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      std::isnan(max_distance) || max_distance < 0.0f) {
    return result;
  }
  const float max_distance_sq =
      std::isfinite(max_distance) ? max_distance * max_distance : std::numeric_limits<float>::infinity();
  std::vector<std::pair<float, entry_handle_type>> candidates;
  auto& filter_ref = filter;
  k_nearest_node(root_, position, filter_ref, max_distance_sq, candidates);
  std::sort(candidates.begin(), candidates.end());
  const std::size_t count = std::min(k, candidates.size());
  result.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    result.push_back(candidates[i].second);
  }
  return result;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
std::vector<typename bvh<Handle, EntryValue>::entry_handle_type>
bvh<Handle, EntryValue>::k_nearest(const glm::vec3& position, std::size_t k, float max_distance) const {
  return k_nearest(
      position,
      k,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      max_distance);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter>
void bvh<Handle, EntryValue>::raycast_node(std::uint32_t index,
                                           const geometry::ray& ry,
                                           Filter& filter,
                                           float max_distance,
                                           std::optional<entry_ray_hit<entry_handle_type>>& best,
                                           float& best_distance) const {
  if (index == invalid) {
    return;
  }
  float node_distance = 0.0f;
  if (!geometry::ray_aabb(ry, nodes_[index].bounds, node_distance) || node_distance > best_distance ||
      node_distance > max_distance) {
    return;
  }
  const node& n = nodes_[index];
  if (n.data) {
    if (!filter(n.data->handle, n.data->value)) {
      return;
    }
    float distance = 0.0f;
    if (!geometry::ray_aabb(ry, n.data->bounds, distance) || distance > max_distance || distance > best_distance) {
      return;
    }
    if (distance < best_distance || (distance == best_distance && (!best || n.data->handle < best->handle))) {
      best_distance = distance;
      best = entry_ray_hit<entry_handle_type>{ n.data->handle, distance };
    }
    return;
  }
  raycast_node(n.left, ry, filter, max_distance, best, best_distance);
  raycast_node(n.right, ry, filter, max_distance, best, best_distance);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter>
  requires std::predicate<Filter&, const Handle&, const EntryValue&>
std::optional<entry_ray_hit<typename bvh<Handle, EntryValue>::entry_handle_type>>
bvh<Handle, EntryValue>::raycast(const geometry::ray& ry, Filter&& filter, float max_distance) const {
  if (!ry.is_valid() || std::isnan(max_distance) || max_distance < 0.0f) {
    return std::nullopt;
  }
  std::optional<entry_ray_hit<entry_handle_type>> best;
  float best_distance = std::numeric_limits<float>::infinity();
  auto& filter_ref = filter;
  raycast_node(root_, ry, filter_ref, max_distance, best, best_distance);
  return best;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
std::optional<entry_ray_hit<typename bvh<Handle, EntryValue>::entry_handle_type>>
bvh<Handle, EntryValue>::raycast(const geometry::ray& ry, float max_distance) const {
  return raycast(
      ry,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      max_distance);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
void bvh<Handle, EntryValue>::clear() noexcept {
  nodes_.clear();
  free_nodes_.clear();
  leaf_of_.clear();
  root_ = invalid;
}

} // namespace spatial_index
} // namespace cyberverse
