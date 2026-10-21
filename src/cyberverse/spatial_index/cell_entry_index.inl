/**
 * @file cell_entry_index.inl
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Cell entry index implementation
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

#include "cyberverse/spatial_index/cell_entry_index.hpp"

namespace cyberverse {
namespace spatial_index {

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename... Args>
  requires std::constructible_from<CellIndex, Args...>
cell_entry_index<CellIndex>::cell_entry_index(Args&&... args) : cell_index_(std::forward<Args>(args)...) {}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
bool cell_entry_index<CellIndex>::valid_bounds(const geometry::aabb& bounds) noexcept {
  return bounds.is_valid() && std::isfinite(bounds.min().x) && std::isfinite(bounds.min().y) &&
         std::isfinite(bounds.min().z) && std::isfinite(bounds.max().x) && std::isfinite(bounds.max().y) &&
         std::isfinite(bounds.max().z);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
void cell_entry_index<CellIndex>::insert_into_cells(const entry_handle_type& handle,
                                                    const geometry::aabb& bounds,
                                                    const entry_value_type& value) {
  cell_coord lo{};
  cell_coord hi{};
  if (!cell_index_.cell_range(bounds, lo, hi)) {
    throw std::invalid_argument("cell entry index bounds do not overlap a cell");
  }
  for (std::int64_t x = lo.x; x <= hi.x; ++x) {
    for (std::int64_t y = lo.y; y <= hi.y; ++y) {
      for (std::int64_t z = lo.z; z <= hi.z; ++z) {
        auto& bucket = cell_index_.ensure_cell(cell_coord{ x, y, z });
        if (!bucket.insert(handle, bounds, value)) {
          throw std::logic_error("cell bucket already contains the entry handle");
        }
      }
    }
  }
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
void cell_entry_index<CellIndex>::remove_from_cells(const entry_handle_type& handle,
                                                    const geometry::aabb& bounds) noexcept {
  cell_coord lo{};
  cell_coord hi{};
  if (!cell_index_.cell_range(bounds, lo, hi)) {
    return;
  }
  for (std::int64_t x = lo.x; x <= hi.x; ++x) {
    for (std::int64_t y = lo.y; y <= hi.y; ++y) {
      for (std::int64_t z = lo.z; z <= hi.z; ++z) {
        const cell_coord cell{ x, y, z };
        bucket_type* bucket = cell_index_.get_cell(cell);
        if (!bucket) {
          continue;
        }
        bucket->erase(handle);
        if (bucket->empty()) {
          cell_index_.erase_cell(cell);
        }
      }
    }
  }
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
bool cell_entry_index<CellIndex>::insert(const entry_handle_type& handle,
                                         const geometry::aabb& bounds,
                                         const entry_value_type& value) {
  if (!valid_bounds(bounds)) {
    throw std::invalid_argument("cell entry index bounds must be valid and finite");
  }
  if (records_.contains(handle)) {
    return false;
  }

  cell_coord lo{};
  cell_coord hi{};
  if (!cell_index_.cell_range(bounds, lo, hi)) {
    throw std::invalid_argument("cell entry index bounds do not overlap a cell");
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

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
bool cell_entry_index<CellIndex>::update(const entry_handle_type& handle, const geometry::aabb& bounds) {
  if (!valid_bounds(bounds)) {
    throw std::invalid_argument("cell entry index bounds must be valid and finite");
  }
  auto it = records_.find(handle);
  if (it == records_.end()) {
    return false;
  }

  cell_coord lo{};
  cell_coord hi{};
  if (!cell_index_.cell_range(bounds, lo, hi)) {
    throw std::invalid_argument("cell entry index bounds do not overlap a cell");
  }

  const geometry::aabb old_bounds = it->second.bounds;
  const bool old_multi_cell = it->second.multi_cell;
  const entry_value_type value = it->second.value;

  remove_from_cells(handle, old_bounds);
  it->second.bounds = bounds;
  it->second.multi_cell = lo.x != hi.x || lo.y != hi.y || lo.z != hi.z;

  try {
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

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
bool cell_entry_index<CellIndex>::set_value(const entry_handle_type& handle, const entry_value_type& value) {
  auto it = records_.find(handle);
  if (it == records_.end()) {
    return false;
  }

  const entry_value_type old_value = it->second.value;
  cell_coord lo{};
  cell_coord hi{};
  if (!cell_index_.cell_range(it->second.bounds, lo, hi)) {
    return false;
  }

  std::vector<bucket_type*> buckets;
  buckets.reserve(static_cast<std::size_t>((hi.x - lo.x + 1) * (hi.y - lo.y + 1) * (hi.z - lo.z + 1)));
  for (std::int64_t x = lo.x; x <= hi.x; ++x) {
    for (std::int64_t y = lo.y; y <= hi.y; ++y) {
      for (std::int64_t z = lo.z; z <= hi.z; ++z) {
        bucket_type* bucket = cell_index_.get_cell(cell_coord{ x, y, z });
        if (!bucket) {
          return false;
        }
        buckets.push_back(bucket);
      }
    }
  }

  it->second.value = value;
  for (bucket_type* bucket : buckets) {
    if (!bucket->set_value(handle, value)) {
      for (bucket_type* previous : buckets) {
        previous->set_value(handle, old_value);
      }
      it->second.value = old_value;
      return false;
    }
  }
  return true;
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
bool cell_entry_index<CellIndex>::erase(const entry_handle_type& handle) {
  auto it = records_.find(handle);
  if (it == records_.end()) {
    return false;
  }
  const geometry::aabb bounds = it->second.bounds;
  remove_from_cells(handle, bounds);
  records_.erase(it);
  return true;
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
bool cell_entry_index<CellIndex>::contains(const entry_handle_type& handle) const noexcept {
  return records_.contains(handle);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
const geometry::aabb* cell_entry_index<CellIndex>::bounds(const entry_handle_type& handle) const noexcept {
  const auto it = records_.find(handle);
  return it == records_.end() ? nullptr : &it->second.bounds;
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
const typename cell_entry_index<CellIndex>::entry_value_type*
cell_entry_index<CellIndex>::value(const entry_handle_type& handle) const noexcept {
  const auto it = records_.find(handle);
  return it == records_.end() ? nullptr : &it->second.value;
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
bool cell_entry_index<CellIndex>::visit(const entry_handle_type& handle,
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

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
bool cell_entry_index<CellIndex>::query_aabb_impl(const geometry::aabb& range,
                                                  query_scratch_type& scratch,
                                                  Filter& filter,
                                                  Visitor& visitor) const {
  scratch.clear();
  const auto always_true = [](const entry_handle_type&, const entry_value_type&) {
    return true;
  };
  bool keep_going = true;
  cell_index_.query(range, [&](const bucket_type& bucket, const geometry::aabb&) {
    bucket.query(
        range, always_true, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb&) {
          keep_going = visit(handle, scratch, filter, visitor);
          return keep_going;
        });
    return keep_going;
  });
  return keep_going;
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
bool cell_entry_index<CellIndex>::query_sphere_impl(const geometry::bounding_sphere& sphere,
                                                    query_scratch_type& scratch,
                                                    Filter& filter,
                                                    Visitor& visitor) const {
  if (!sphere.is_valid() || !std::isfinite(sphere.radius()) || !std::isfinite(sphere.center().x) ||
      !std::isfinite(sphere.center().y) || !std::isfinite(sphere.center().z)) {
    return true;
  }

  scratch.clear();
  const glm::vec3 extent(sphere.radius());
  const geometry::aabb sphere_box(sphere.center() - extent, sphere.center() + extent);
  const auto always_true = [](const entry_handle_type&, const entry_value_type&) {
    return true;
  };
  bool keep_going = true;
  cell_index_.query(sphere_box, [&](const bucket_type& bucket, const geometry::aabb&) {
    bucket.query(
        sphere, always_true, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb&) {
          keep_going = visit(handle, scratch, filter, visitor);
          return keep_going;
        });
    return keep_going;
  });
  return keep_going;
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
bool cell_entry_index<CellIndex>::query_ray_impl(const geometry::ray& ry,
                                                 query_scratch_type& scratch,
                                                 Filter& filter,
                                                 Visitor& visitor) const {
  scratch.clear();
  const auto always_true = [](const entry_handle_type&, const entry_value_type&) {
    return true;
  };
  bool keep_going = true;
  cell_index_.query(ry, [&](const bucket_type& bucket, const geometry::aabb&) {
    bucket.query(ry, always_true, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb&) {
      keep_going = visit(handle, scratch, filter, visitor);
      return keep_going;
    });
    return keep_going;
  });
  return keep_going;
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
bool cell_entry_index<CellIndex>::query_frustum_impl(const geometry::frustum& fr,
                                                     query_scratch_type& scratch,
                                                     Filter& filter,
                                                     Visitor& visitor) const {
  scratch.clear();
  const auto always_true = [](const entry_handle_type&, const entry_value_type&) {
    return true;
  };
  bool keep_going = true;
  cell_index_.query(fr, [&](const bucket_type& bucket, const geometry::aabb&) {
    bucket.query(fr, always_true, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb&) {
      keep_going = visit(handle, scratch, filter, visitor);
      return keep_going;
    });
    return keep_going;
  });
  return keep_going;
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&> &&
           std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>)
void cell_entry_index<CellIndex>::query(const geometry::aabb& range,
                                        query_scratch_type& scratch,
                                        Filter&& filter,
                                        Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  query_aabb_impl(range, scratch, filter_ref, visitor_ref);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Visitor>
  requires std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>
void cell_entry_index<CellIndex>::query(const geometry::aabb& range,
                                        query_scratch_type& scratch,
                                        Visitor&& visitor) const {
  query(
      range,
      scratch,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&> &&
           std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>)
void cell_entry_index<CellIndex>::query(const geometry::aabb& range, Filter&& filter, Visitor&& visitor) const {
  query_scratch_type scratch;
  query(range, scratch, std::forward<Filter>(filter), std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Visitor>
  requires std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>
void cell_entry_index<CellIndex>::query(const geometry::aabb& range, Visitor&& visitor) const {
  query_scratch_type scratch;
  query(range, scratch, std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&> &&
           std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>)
void cell_entry_index<CellIndex>::query(const geometry::bounding_sphere& sphere,
                                        query_scratch_type& scratch,
                                        Filter&& filter,
                                        Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  query_sphere_impl(sphere, scratch, filter_ref, visitor_ref);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Visitor>
  requires std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>
void cell_entry_index<CellIndex>::query(const geometry::bounding_sphere& sphere,
                                        query_scratch_type& scratch,
                                        Visitor&& visitor) const {
  query(
      sphere,
      scratch,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&> &&
           std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>)
void cell_entry_index<CellIndex>::query(const geometry::bounding_sphere& sphere,
                                        Filter&& filter,
                                        Visitor&& visitor) const {
  query_scratch_type scratch;
  query(sphere, scratch, std::forward<Filter>(filter), std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Visitor>
  requires std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>
void cell_entry_index<CellIndex>::query(const geometry::bounding_sphere& sphere, Visitor&& visitor) const {
  query_scratch_type scratch;
  query(sphere, scratch, std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&> &&
           std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>)
void cell_entry_index<CellIndex>::query(const geometry::ray& ry,
                                        query_scratch_type& scratch,
                                        Filter&& filter,
                                        Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  query_ray_impl(ry, scratch, filter_ref, visitor_ref);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Visitor>
  requires std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>
void cell_entry_index<CellIndex>::query(const geometry::ray& ry, query_scratch_type& scratch, Visitor&& visitor) const {
  query(
      ry,
      scratch,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&> &&
           std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>)
void cell_entry_index<CellIndex>::query(const geometry::ray& ry, Filter&& filter, Visitor&& visitor) const {
  query_scratch_type scratch;
  query(ry, scratch, std::forward<Filter>(filter), std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Visitor>
  requires std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>
void cell_entry_index<CellIndex>::query(const geometry::ray& ry, Visitor&& visitor) const {
  query_scratch_type scratch;
  query(ry, scratch, std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&> &&
           std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>)
void cell_entry_index<CellIndex>::query(const geometry::frustum& fr,
                                        query_scratch_type& scratch,
                                        Filter&& filter,
                                        Visitor&& visitor) const {
  auto& filter_ref = filter;
  auto& visitor_ref = visitor;
  query_frustum_impl(fr, scratch, filter_ref, visitor_ref);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Visitor>
  requires std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>
void cell_entry_index<CellIndex>::query(const geometry::frustum& fr,
                                        query_scratch_type& scratch,
                                        Visitor&& visitor) const {
  query(
      fr,
      scratch,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter, typename Visitor>
  requires(std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&> &&
           std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>)
void cell_entry_index<CellIndex>::query(const geometry::frustum& fr, Filter&& filter, Visitor&& visitor) const {
  query_scratch_type scratch;
  query(fr, scratch, std::forward<Filter>(filter), std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Visitor>
  requires std::predicate<Visitor&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&,
                          const geometry::aabb&>
void cell_entry_index<CellIndex>::query(const geometry::frustum& fr, Visitor&& visitor) const {
  query_scratch_type scratch;
  query(fr, scratch, std::forward<Visitor>(visitor));
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter>
std::optional<typename cell_entry_index<CellIndex>::entry_handle_type> cell_entry_index<CellIndex>::nearest_impl(
    const glm::vec3& position, query_scratch_type& scratch, Filter& filter, float max_distance) const {
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
    query(geometry::bounding_sphere(position, max_distance), scratch, filter, consider);
  } else {
    for (const auto& [handle, rec] : records_) {
      if (!filter(handle, rec.value)) {
        continue;
      }
      const float distance_sq = geometry::point_aabb_distance_sq(position, rec.bounds);
      if (!best || distance_sq < best_distance_sq || (distance_sq == best_distance_sq && handle < *best)) {
        best_distance_sq = distance_sq;
        best = handle;
      }
    }
  }
  return best;
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter>
std::vector<typename cell_entry_index<CellIndex>::entry_handle_type> cell_entry_index<CellIndex>::k_nearest_impl(
    const glm::vec3& position, std::size_t k, query_scratch_type& scratch, Filter& filter, float max_distance) const {
  std::vector<entry_handle_type> result;
  if (k == 0 || !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      std::isnan(max_distance) || max_distance < 0.0f) {
    return result;
  }

  std::vector<std::pair<float, entry_handle_type>> candidates;
  const float max_distance_sq = max_distance * max_distance;
  if (std::isfinite(max_distance)) {
    query(geometry::bounding_sphere(position, max_distance),
          scratch,
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

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter>
std::optional<entry_ray_hit<typename cell_entry_index<CellIndex>::entry_handle_type>>
cell_entry_index<CellIndex>::raycast_impl(const geometry::ray& ry,
                                          query_scratch_type& scratch,
                                          Filter& filter,
                                          float max_distance) const {
  if (!ry.is_valid() || std::isnan(max_distance) || max_distance < 0.0f) {
    return std::nullopt;
  }

  std::optional<entry_ray_hit<entry_handle_type>> best;
  float best_distance = std::numeric_limits<float>::infinity();
  query(
      ry, scratch, filter, [&](const entry_handle_type& handle, const entry_value_type&, const geometry::aabb& bounds) {
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

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter>
  requires std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&>
std::optional<typename cell_entry_index<CellIndex>::entry_handle_type> cell_entry_index<CellIndex>::nearest(
    const glm::vec3& position, query_scratch_type& scratch, Filter&& filter, float max_distance) const {
  auto& filter_ref = filter;
  return nearest_impl(position, scratch, filter_ref, max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
std::optional<typename cell_entry_index<CellIndex>::entry_handle_type>
cell_entry_index<CellIndex>::nearest(const glm::vec3& position, query_scratch_type& scratch, float max_distance) const {
  return nearest(
      position,
      scratch,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter>
  requires std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&>
std::optional<typename cell_entry_index<CellIndex>::entry_handle_type>
cell_entry_index<CellIndex>::nearest(const glm::vec3& position, Filter&& filter, float max_distance) const {
  query_scratch_type scratch;
  return nearest(position, scratch, std::forward<Filter>(filter), max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
std::optional<typename cell_entry_index<CellIndex>::entry_handle_type>
cell_entry_index<CellIndex>::nearest(const glm::vec3& position, float max_distance) const {
  query_scratch_type scratch;
  return nearest(position, scratch, max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter>
  requires std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&>
std::vector<typename cell_entry_index<CellIndex>::entry_handle_type> cell_entry_index<CellIndex>::k_nearest(
    const glm::vec3& position, std::size_t k, query_scratch_type& scratch, Filter&& filter, float max_distance) const {
  auto& filter_ref = filter;
  return k_nearest_impl(position, k, scratch, filter_ref, max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
std::vector<typename cell_entry_index<CellIndex>::entry_handle_type> cell_entry_index<CellIndex>::k_nearest(
    const glm::vec3& position, std::size_t k, query_scratch_type& scratch, float max_distance) const {
  return k_nearest(
      position,
      k,
      scratch,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter>
  requires std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&>
std::vector<typename cell_entry_index<CellIndex>::entry_handle_type> cell_entry_index<CellIndex>::k_nearest(
    const glm::vec3& position, std::size_t k, Filter&& filter, float max_distance) const {
  query_scratch_type scratch;
  return k_nearest(position, k, scratch, std::forward<Filter>(filter), max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
std::vector<typename cell_entry_index<CellIndex>::entry_handle_type>
cell_entry_index<CellIndex>::k_nearest(const glm::vec3& position, std::size_t k, float max_distance) const {
  query_scratch_type scratch;
  return k_nearest(position, k, scratch, max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter>
  requires std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&>
std::optional<entry_ray_hit<typename cell_entry_index<CellIndex>::entry_handle_type>>
cell_entry_index<CellIndex>::raycast(const geometry::ray& ry,
                                     query_scratch_type& scratch,
                                     Filter&& filter,
                                     float max_distance) const {
  auto& filter_ref = filter;
  return raycast_impl(ry, scratch, filter_ref, max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
std::optional<entry_ray_hit<typename cell_entry_index<CellIndex>::entry_handle_type>>
cell_entry_index<CellIndex>::raycast(const geometry::ray& ry, query_scratch_type& scratch, float max_distance) const {
  return raycast(
      ry,
      scratch,
      [](const entry_handle_type&, const entry_value_type&) {
        return true;
      },
      max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
template <typename Filter>
  requires std::predicate<Filter&,
                          const typename cell_entry_index<CellIndex>::entry_handle_type&,
                          const typename cell_entry_index<CellIndex>::entry_value_type&>
std::optional<entry_ray_hit<typename cell_entry_index<CellIndex>::entry_handle_type>>
cell_entry_index<CellIndex>::raycast(const geometry::ray& ry, Filter&& filter, float max_distance) const {
  query_scratch_type scratch;
  return raycast(ry, scratch, std::forward<Filter>(filter), max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
std::optional<entry_ray_hit<typename cell_entry_index<CellIndex>::entry_handle_type>>
cell_entry_index<CellIndex>::raycast(const geometry::ray& ry, float max_distance) const {
  query_scratch_type scratch;
  return raycast(ry, scratch, max_distance);
}

template <CellIndexType CellIndex>
  requires(EntryBucketType<typename CellIndex::value_type> &&
           std::default_initializable<typename CellIndex::value_type>)
void cell_entry_index<CellIndex>::clear() noexcept {
  cell_index_.clear();
  records_.clear();
}

} // namespace spatial_index
} // namespace cyberverse
