/**
 * @file hash_index.inl
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Spatial hash implementation
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

#include "cyberverse/spatial_index/hash_index.hpp"

namespace cyberverse {
namespace spatial_index {

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
std::size_t hash_index<Handle, EntryValue>::cell_key_hash::operator()(const cell_key& key) const noexcept {
  std::uint64_t h = 1469598103934665603ull;
  const auto mix = [&](std::uint64_t v) {
    h ^= v;
    h *= 1099511628211ull;
  };
  mix(static_cast<std::uint64_t>(key.x));
  mix(static_cast<std::uint64_t>(key.y));
  mix(static_cast<std::uint64_t>(key.z));
  return static_cast<std::size_t>(h);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
hash_index<Handle, EntryValue>::hash_index(glm::vec3 unit, glm::vec3 origin) : unit_(unit), origin_(origin) {
  if (!std::isfinite(unit_.x) || !std::isfinite(unit_.y) || !std::isfinite(unit_.z) || unit_.x <= 0.0f ||
      unit_.y <= 0.0f || unit_.z <= 0.0f) {
    throw std::invalid_argument("hash index unit must be finite and greater than zero");
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool hash_index<Handle, EntryValue>::valid_bounds(const geometry::aabb& bounds) noexcept {
  return bounds.is_valid() && std::isfinite(bounds.min().x) && std::isfinite(bounds.min().y) &&
         std::isfinite(bounds.min().z) && std::isfinite(bounds.max().x) && std::isfinite(bounds.max().y) &&
         std::isfinite(bounds.max().z);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool hash_index<Handle, EntryValue>::to_cell_range(const geometry::aabb& bounds,
                                                   cell_key& lo,
                                                   cell_key& hi) const noexcept {
  detail::cell_vec cell_lo{};
  detail::cell_vec cell_hi{};
  if (!detail::cell_range(origin_, unit_, bounds, cell_lo, cell_hi)) {
    return false;
  }
  lo = cell_key{ cell_lo.x, cell_lo.y, cell_lo.z };
  hi = cell_key{ cell_hi.x, cell_hi.y, cell_hi.z };
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
void hash_index<Handle, EntryValue>::insert_into_cells(const entry_handle_type& handle,
                                                       const geometry::aabb& bounds,
                                                       const entry_value_type& value) {
  cell_key lo{};
  cell_key hi{};
  if (!to_cell_range(bounds, lo, hi)) {
    throw std::invalid_argument("hash index bounds do not overlap a cell");
  }
  for (std::int64_t x = lo.x; x <= hi.x; ++x) {
    for (std::int64_t y = lo.y; y <= hi.y; ++y) {
      for (std::int64_t z = lo.z; z <= hi.z; ++z) {
        if (!cells_[cell_key{ x, y, z }].insert(handle, bounds, value)) {
          throw std::logic_error("hash cell already contains the entry handle");
        }
      }
    }
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
void hash_index<Handle, EntryValue>::remove_from_cells(const entry_handle_type& handle,
                                                       const geometry::aabb& bounds) noexcept {
  cell_key lo{};
  cell_key hi{};
  if (!to_cell_range(bounds, lo, hi)) {
    return;
  }
  for (std::int64_t x = lo.x; x <= hi.x; ++x) {
    for (std::int64_t y = lo.y; y <= hi.y; ++y) {
      for (std::int64_t z = lo.z; z <= hi.z; ++z) {
        const auto it = cells_.find(cell_key{ x, y, z });
        if (it == cells_.end()) {
          continue;
        }
        it->second.erase(handle);
        if (it->second.empty()) {
          cells_.erase(it);
        }
      }
    }
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool hash_index<Handle, EntryValue>::insert(const entry_handle_type& handle,
                                            const geometry::aabb& bounds,
                                            const entry_value_type& value) {
  if (!valid_bounds(bounds)) {
    throw std::invalid_argument("hash index bounds must be valid and finite");
  }
  if (records_.contains(handle)) {
    return false;
  }
  cell_key lo{};
  cell_key hi{};
  if (!to_cell_range(bounds, lo, hi)) {
    throw std::invalid_argument("hash index bounds do not overlap a cell");
  }
  const bool multi_cell = lo.x != hi.x || lo.y != hi.y || lo.z != hi.z;
  try {
    insert_into_cells(handle, bounds, value);
    records_.emplace(handle, record{ bounds, value, multi_cell });
  } catch (...) {
    remove_from_cells(handle, bounds);
    throw;
  }
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool hash_index<Handle, EntryValue>::update(const entry_handle_type& handle, const geometry::aabb& bounds) {
  if (!valid_bounds(bounds)) {
    throw std::invalid_argument("hash index bounds must be valid and finite");
  }
  const auto it = records_.find(handle);
  if (it == records_.end()) {
    return false;
  }
  const geometry::aabb old_bounds = it->second.bounds;
  const bool old_multi_cell = it->second.multi_cell;
  const entry_value_type value = it->second.value;
  remove_from_cells(handle, old_bounds);
  it->second.bounds = bounds;
  it->second.multi_cell = false;
  try {
    cell_key lo{};
    cell_key hi{};
    if (!to_cell_range(bounds, lo, hi)) {
      throw std::invalid_argument("hash index bounds do not overlap a cell");
    }
    it->second.multi_cell = lo.x != hi.x || lo.y != hi.y || lo.z != hi.z;
    insert_into_cells(handle, bounds, value);
  } catch (...) {
    remove_from_cells(handle, bounds);
    it->second.bounds = old_bounds;
    it->second.multi_cell = old_multi_cell;
    insert_into_cells(handle, old_bounds, value);
    throw;
  }
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool hash_index<Handle, EntryValue>::set_value(const entry_handle_type& handle, const entry_value_type& value) {
  const auto it = records_.find(handle);
  if (it == records_.end()) {
    return false;
  }
  const entry_value_type old_value = it->second.value;
  it->second.value = value;
  cell_key lo{};
  cell_key hi{};
  if (!to_cell_range(it->second.bounds, lo, hi)) {
    it->second.value = old_value;
    return false;
  }
  for (std::int64_t x = lo.x; x <= hi.x; ++x) {
    for (std::int64_t y = lo.y; y <= hi.y; ++y) {
      for (std::int64_t z = lo.z; z <= hi.z; ++z) {
        const auto cell_it = cells_.find(cell_key{ x, y, z });
        if (cell_it == cells_.end() || !cell_it->second.set_value(handle, value)) {
          it->second.value = old_value;
          return false;
        }
      }
    }
  }
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool hash_index<Handle, EntryValue>::erase(const entry_handle_type& handle) {
  const auto it = records_.find(handle);
  if (it == records_.end()) {
    return false;
  }
  remove_from_cells(handle, it->second.bounds);
  records_.erase(it);
  return true;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
bool hash_index<Handle, EntryValue>::contains(const entry_handle_type& handle) const noexcept {
  return records_.contains(handle);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
const geometry::aabb* hash_index<Handle, EntryValue>::bounds(const entry_handle_type& handle) const noexcept {
  const auto it = records_.find(handle);
  return it == records_.end() ? nullptr : &it->second.bounds;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
const typename hash_index<Handle, EntryValue>::entry_value_type*
hash_index<Handle, EntryValue>::value(const entry_handle_type& handle) const noexcept {
  const auto it = records_.find(handle);
  return it == records_.end() ? nullptr : &it->second.value;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
bool hash_index<Handle, EntryValue>::visit(const entry_handle_type& handle,
                                           query_scratch_type& scratch,
                                           Filter& filter,
                                           Visitor& visitor) const {
  const auto it = records_.find(handle);
  if (it == records_.end()) {
    return true;
  }
  if (it->second.multi_cell && !scratch.mark(handle)) {
    return true;
  }
  if (!filter(handle, it->second.value)) {
    return true;
  }
  return visitor(handle, it->second.value, it->second.bounds);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
void hash_index<Handle, EntryValue>::query_aabb_impl(const geometry::aabb& range,
                                                     Filter& filter,
                                                     Visitor& visitor) const {
  query_scratch_type scratch;
  cell_key lo{};
  cell_key hi{};
  if (!to_cell_range(range, lo, hi)) {
    return;
  }
  const auto always_true = [](const entry_handle_type&, const entry_value_type&) {
    return true;
  };
  bool keep_going = true;
  for (std::int64_t x = lo.x; x <= hi.x && keep_going; ++x) {
    for (std::int64_t y = lo.y; y <= hi.y && keep_going; ++y) {
      for (std::int64_t z = lo.z; z <= hi.z && keep_going; ++z) {
        const auto it = cells_.find(cell_key{ x, y, z });
        if (it == cells_.end()) {
          continue;
        }
        it->second.query(
            range, always_true, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb&) {
              keep_going = visit(handle, scratch, filter, visitor);
              return keep_going;
            });
      }
    }
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
void hash_index<Handle, EntryValue>::query_sphere_impl(const geometry::bounding_sphere& sphere,
                                                       Filter& filter,
                                                       Visitor& visitor) const {
  if (!sphere.is_valid() || !std::isfinite(sphere.radius()) || !std::isfinite(sphere.center().x) ||
      !std::isfinite(sphere.center().y) || !std::isfinite(sphere.center().z)) {
    return;
  }
  query_scratch_type scratch;
  const glm::vec3 extent(sphere.radius());
  cell_key lo{};
  cell_key hi{};
  if (!to_cell_range(geometry::aabb(sphere.center() - extent, sphere.center() + extent), lo, hi)) {
    return;
  }
  const auto always_true = [](const entry_handle_type&, const entry_value_type&) {
    return true;
  };
  bool keep_going = true;
  for (std::int64_t x = lo.x; x <= hi.x && keep_going; ++x) {
    for (std::int64_t y = lo.y; y <= hi.y && keep_going; ++y) {
      for (std::int64_t z = lo.z; z <= hi.z && keep_going; ++z) {
        const auto it = cells_.find(cell_key{ x, y, z });
        if (it == cells_.end()) {
          continue;
        }
        it->second.query(
            sphere, always_true, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb&) {
              keep_going = visit(handle, scratch, filter, visitor);
              return keep_going;
            });
      }
    }
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
void hash_index<Handle, EntryValue>::query_ray_impl(const geometry::ray& ry, Filter& filter, Visitor& visitor) const {
  query_scratch_type scratch;
  const auto always_true = [](const entry_handle_type&, const entry_value_type&) {
    return true;
  };
  bool keep_going = true;
  for (const auto& [key, bucket] : cells_) {
    (void)key;
    if (!keep_going) {
      break;
    }
    bucket.query(ry, always_true, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb&) {
      keep_going = visit(handle, scratch, filter, visitor);
      return keep_going;
    });
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
void hash_index<Handle, EntryValue>::query_frustum_impl(const geometry::frustum& fr,
                                                        Filter& filter,
                                                        Visitor& visitor) const {
  query_scratch_type scratch;
  const auto always_true = [](const entry_handle_type&, const entry_value_type&) {
    return true;
  };
  bool keep_going = true;
  for (const auto& [key, bucket] : cells_) {
    (void)key;
    if (!keep_going) {
      break;
    }
    bucket.query(fr, always_true, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb&) {
      keep_going = visit(handle, scratch, filter, visitor);
      return keep_going;
    });
  }
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&, const Handle&, const EntryValue&> &&
           std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>)
void hash_index<Handle, EntryValue>::query(const geometry::aabb& range, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  query_aabb_impl(range, filter_ref, visitor_ref);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void hash_index<Handle, EntryValue>::query(const geometry::aabb& range, Visitor&& visitor) const {
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
void hash_index<Handle, EntryValue>::query(const geometry::bounding_sphere& sphere,
                                           Filter&& filter,
                                           Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  query_sphere_impl(sphere, filter_ref, visitor_ref);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void hash_index<Handle, EntryValue>::query(const geometry::bounding_sphere& sphere, Visitor&& visitor) const {
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
void hash_index<Handle, EntryValue>::query(const geometry::ray& ry, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  query_ray_impl(ry, filter_ref, visitor_ref);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void hash_index<Handle, EntryValue>::query(const geometry::ray& ry, Visitor&& visitor) const {
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
void hash_index<Handle, EntryValue>::query(const geometry::frustum& fr, Filter&& filter, Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  query_frustum_impl(fr, filter_ref, visitor_ref);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
template <typename Visitor>
  requires std::predicate<Visitor&, const Handle&, const EntryValue&, const geometry::aabb&>
void hash_index<Handle, EntryValue>::query(const geometry::frustum& fr, Visitor&& visitor) const {
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
std::optional<typename hash_index<Handle, EntryValue>::entry_handle_type>
hash_index<Handle, EntryValue>::nearest(const glm::vec3& position, Filter&& filter, float max_distance) const {
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      std::isnan(max_distance) || max_distance < 0.0f) {
    return std::nullopt;
  }
  std::optional<entry_handle_type> best;
  float best_distance_sq = std::numeric_limits<float>::infinity();
  const auto consider = [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb& bounds) {
    const float distance_sq = geometry::point_aabb_distance_sq(position, bounds);
    if (!best || distance_sq < best_distance_sq || (distance_sq == best_distance_sq && handle < *best)) {
      best_distance_sq = distance_sq;
      best = handle;
    }
    return true;
  };
  if (std::isfinite(max_distance)) {
    query(geometry::bounding_sphere(position, max_distance), filter, consider);
  } else {
    for (const auto& [handle, rec] : records_) {
      if (filter(handle, rec.value)) {
        const float distance_sq = geometry::point_aabb_distance_sq(position, rec.bounds);
        if (!best || distance_sq < best_distance_sq || (distance_sq == best_distance_sq && handle < *best)) {
          best_distance_sq = distance_sq;
          best = handle;
        }
      }
    }
  }
  return best;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
std::optional<typename hash_index<Handle, EntryValue>::entry_handle_type>
hash_index<Handle, EntryValue>::nearest(const glm::vec3& position, float max_distance) const {
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
std::vector<typename hash_index<Handle, EntryValue>::entry_handle_type> hash_index<Handle, EntryValue>::k_nearest(
    const glm::vec3& position, std::size_t k, Filter&& filter, float max_distance) const {
  std::vector<entry_handle_type> result;
  if (k == 0 || !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      std::isnan(max_distance) || max_distance < 0.0f) {
    return result;
  }
  std::vector<std::pair<float, entry_handle_type>> candidates;
  const float max_distance_sq = max_distance * max_distance;
  if (std::isfinite(max_distance)) {
    query(geometry::bounding_sphere(position, max_distance),
          filter,
          [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb& bounds) {
            const float distance_sq = geometry::point_aabb_distance_sq(position, bounds);
            if (distance_sq <= max_distance_sq) {
              candidates.emplace_back(distance_sq, handle);
            }
            return true;
          });
  } else {
    for (const auto& [handle, rec] : records_) {
      if (filter(handle, rec.value)) {
        candidates.emplace_back(geometry::point_aabb_distance_sq(position, rec.bounds), handle);
      }
    }
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
std::vector<typename hash_index<Handle, EntryValue>::entry_handle_type>
hash_index<Handle, EntryValue>::k_nearest(const glm::vec3& position, std::size_t k, float max_distance) const {
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
std::optional<entry_ray_hit<typename hash_index<Handle, EntryValue>::entry_handle_type>>
hash_index<Handle, EntryValue>::raycast(const geometry::ray& ry, Filter&& filter, float max_distance) const {
  if (!ry.is_valid() || std::isnan(max_distance) || max_distance < 0.0f) {
    return std::nullopt;
  }
  std::optional<entry_ray_hit<entry_handle_type>> best;
  float best_distance = std::numeric_limits<float>::infinity();
  query(ry, filter, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb& bounds) {
    float distance = 0.0f;
    if (!geometry::ray_aabb(ry, bounds, distance) || distance > max_distance) {
      return true;
    }
    if (distance < best_distance || (distance == best_distance && (!best || handle < best->handle))) {
      best_distance = distance;
      best = entry_ray_hit<entry_handle_type>{ handle, distance };
    }
    return true;
  });
  return best;
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
std::optional<entry_ray_hit<typename hash_index<Handle, EntryValue>::entry_handle_type>>
hash_index<Handle, EntryValue>::raycast(const geometry::ray& ry, float max_distance) const {
  return raycast(
      ry,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      max_distance);
}

template <EntryHandleType Handle, typename EntryValue>
  requires std::default_initializable<EntryValue>
void hash_index<Handle, EntryValue>::clear() noexcept {
  cells_.clear();
  records_.clear();
}

} // namespace spatial_index
} // namespace cyberverse
