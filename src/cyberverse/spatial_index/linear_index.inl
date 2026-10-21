/**
 * @file linear_index.inl
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Linear entry index implementation
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
#include <utility>

#include "cyberverse/spatial_index/linear_index.hpp"

namespace cyberverse {
namespace spatial_index {

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool linear_index<Handle, EntryValue>::valid_bounds(const geometry::aabb& bounds) noexcept {
  return bounds.is_valid() && std::isfinite(bounds.min().x) && std::isfinite(bounds.min().y) &&
         std::isfinite(bounds.min().z) && std::isfinite(bounds.max().x) && std::isfinite(bounds.max().y) &&
         std::isfinite(bounds.max().z);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
typename linear_index<Handle, EntryValue>::entry*
linear_index<Handle, EntryValue>::find(const entry_handle_type& handle) noexcept {
  const auto it = std::find_if(entries_.begin(), entries_.end(), [&](const entry& e) {
    return e.handle == handle;
  });
  return it == entries_.end() ? nullptr : &*it;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
const typename linear_index<Handle, EntryValue>::entry*
linear_index<Handle, EntryValue>::find(const entry_handle_type& handle) const noexcept {
  const auto it = std::find_if(entries_.begin(), entries_.end(), [&](const entry& e) {
    return e.handle == handle;
  });
  return it == entries_.end() ? nullptr : &*it;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool linear_index<Handle, EntryValue>::insert(const entry_handle_type& handle,
                                              const geometry::aabb& bounds,
                                              const entry_value_type& value) {
  if (!valid_bounds(bounds)) {
    throw std::invalid_argument("linear index entry bounds must be valid and finite");
  }
  if (find(handle)) {
    return false;
  }
  entries_.push_back(entry{ handle, bounds, value });
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool linear_index<Handle, EntryValue>::update(const entry_handle_type& handle, const geometry::aabb& bounds) {
  if (!valid_bounds(bounds)) {
    throw std::invalid_argument("linear index entry bounds must be valid and finite");
  }
  entry* e = find(handle);
  if (!e) {
    return false;
  }
  e->bounds = bounds;
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool linear_index<Handle, EntryValue>::set_value(const entry_handle_type& handle, const entry_value_type& value) {
  entry* e = find(handle);
  if (!e) {
    return false;
  }
  e->value = value;
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool linear_index<Handle, EntryValue>::erase(const entry_handle_type& handle) noexcept {
  const auto it = std::find_if(entries_.begin(), entries_.end(), [&](const entry& e) {
    return e.handle == handle;
  });
  if (it == entries_.end()) {
    return false;
  }
  *it = std::move(entries_.back());
  entries_.pop_back();
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool linear_index<Handle, EntryValue>::contains(const entry_handle_type& handle) const noexcept {
  return find(handle) != nullptr;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
const geometry::aabb* linear_index<Handle, EntryValue>::bounds(const entry_handle_type& handle) const noexcept {
  const entry* e = find(handle);
  return e ? &e->bounds : nullptr;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
const typename linear_index<Handle, EntryValue>::entry_value_type*
linear_index<Handle, EntryValue>::value(const entry_handle_type& handle) const noexcept {
  const entry* e = find(handle);
  return e ? &e->value : nullptr;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&, const Handle&, const EntryValue&> &&
           std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>)
void linear_index<Handle, EntryValue>::query(const geometry::aabb& range, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  for (const entry& e : entries_) {
    if (!geometry::aabb_overlaps(e.bounds, range)) {
      continue;
    }
    if (!filter_ref(e.handle, e.value)) {
      continue;
    }
    if (!visitor_ref(e.handle, e.value, e.bounds)) {
      break;
    }
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void linear_index<Handle, EntryValue>::query(const geometry::aabb& range, Visitor&& visitor) const {
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
void linear_index<Handle, EntryValue>::query(const geometry::bounding_sphere& sphere,
                                             Filter&& filter,
                                             Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  for (const entry& e : entries_) {
    if (!geometry::sphere_overlaps_aabb(sphere, e.bounds)) {
      continue;
    }
    if (!filter_ref(e.handle, e.value)) {
      continue;
    }
    if (!visitor_ref(e.handle, e.value, e.bounds)) {
      break;
    }
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void linear_index<Handle, EntryValue>::query(const geometry::bounding_sphere& sphere, Visitor&& visitor) const {
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
void linear_index<Handle, EntryValue>::query(const geometry::ray& ry, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  for (const entry& e : entries_) {
    float distance = 0.0f;
    if (!geometry::ray_aabb(ry, e.bounds, distance)) {
      continue;
    }
    if (!filter_ref(e.handle, e.value)) {
      continue;
    }
    if (!visitor_ref(e.handle, e.value, e.bounds)) {
      break;
    }
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void linear_index<Handle, EntryValue>::query(const geometry::ray& ry, Visitor&& visitor) const {
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
void linear_index<Handle, EntryValue>::query(const geometry::frustum& fr, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  for (const entry& e : entries_) {
    if (geometry::is_outside(fr, e.bounds)) {
      continue;
    }
    if (!filter_ref(e.handle, e.value)) {
      continue;
    }
    if (!visitor_ref(e.handle, e.value, e.bounds)) {
      break;
    }
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void linear_index<Handle, EntryValue>::query(const geometry::frustum& fr, Visitor&& visitor) const {
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
  requires std::predicate<Filter&, const Handle&, const EntryValue&>
std::optional<typename linear_index<Handle, EntryValue>::entry_handle_type>
linear_index<Handle, EntryValue>::nearest(const glm::vec3& position, Filter&& filter, float max_distance) const {
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      std::isnan(max_distance) || max_distance < 0.0f) {
    return std::nullopt;
  }
  const float max_distance_sq = max_distance * max_distance;
  std::optional<entry_handle_type> best;
  float best_distance_sq = std::numeric_limits<float>::infinity();
  for (const entry& e : entries_) {
    const float distance_sq = geometry::point_aabb_distance_sq(position, e.bounds);
    if (distance_sq > max_distance_sq) {
      continue;
    }
    if (!filter(e.handle, e.value)) {
      continue;
    }
    if (distance_sq < best_distance_sq || (distance_sq == best_distance_sq && (!best || e.handle < *best))) {
      best_distance_sq = distance_sq;
      best = e.handle;
    }
  }
  return best;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
std::optional<typename linear_index<Handle, EntryValue>::entry_handle_type>
linear_index<Handle, EntryValue>::nearest(const glm::vec3& position, float max_distance) const {
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
  requires std::predicate<Filter&, const Handle&, const EntryValue&>
std::vector<typename linear_index<Handle, EntryValue>::entry_handle_type> linear_index<Handle, EntryValue>::k_nearest(
    const glm::vec3& position, std::size_t k, Filter&& filter, float max_distance) const {
  std::vector<entry_handle_type> result;
  if (k == 0 || !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      std::isnan(max_distance) || max_distance < 0.0f) {
    return result;
  }
  const float max_distance_sq = max_distance * max_distance;
  std::vector<std::pair<float, entry_handle_type>> candidates;
  for (const entry& e : entries_) {
    const float distance_sq = geometry::point_aabb_distance_sq(position, e.bounds);
    if (distance_sq > max_distance_sq || !filter(e.handle, e.value)) {
      continue;
    }
    candidates.emplace_back(distance_sq, e.handle);
  }
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
std::vector<typename linear_index<Handle, EntryValue>::entry_handle_type>
linear_index<Handle, EntryValue>::k_nearest(const glm::vec3& position, std::size_t k, float max_distance) const {
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
  requires std::predicate<Filter&, const Handle&, const EntryValue&>
std::optional<entry_ray_hit<typename linear_index<Handle, EntryValue>::entry_handle_type>>
linear_index<Handle, EntryValue>::raycast(const geometry::ray& ry, Filter&& filter, float max_distance) const {
  if (!ry.is_valid() || std::isnan(max_distance) || max_distance < 0.0f) {
    return std::nullopt;
  }
  std::optional<entry_ray_hit<entry_handle_type>> best;
  float best_distance = std::numeric_limits<float>::infinity();
  for (const entry& e : entries_) {
    float distance = 0.0f;
    if (!geometry::ray_aabb(ry, e.bounds, distance) || distance > max_distance) {
      continue;
    }
    if (!filter(e.handle, e.value)) {
      continue;
    }
    if (distance < best_distance || (distance == best_distance && (!best || e.handle < best->handle))) {
      best_distance = distance;
      best = entry_ray_hit<entry_handle_type>{ e.handle, distance };
    }
  }
  return best;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
std::optional<entry_ray_hit<typename linear_index<Handle, EntryValue>::entry_handle_type>>
linear_index<Handle, EntryValue>::raycast(const geometry::ray& ry, float max_distance) const {
  return raycast(
      ry,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      max_distance);
}

} // namespace spatial_index
} // namespace cyberverse
