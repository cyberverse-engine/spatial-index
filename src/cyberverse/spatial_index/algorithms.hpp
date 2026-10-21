/**
 * @file algorithms.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Generic entry-index algorithms
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/spatial_index/type_traits.hpp"

namespace cyberverse {
namespace spatial_index {

/**
 * @brief Collect handles of entries inside a frustum
 *
 * @tparam Index Entry index type
 * @tparam Filter Filter type
 * @param index Entry index
 * @param fr Frustum
 * @param filter Entry filter
 * @param out Output handles; cleared before the query
 * @return Number of collected handles
 */
template <EntryIndexType Index, typename Filter>
  requires std::predicate<Filter&, const typename Index::entry_handle_type&, const typename Index::entry_value_type&>
std::size_t collect_visible(const Index& index,
                            const geometry::frustum& fr,
                            Filter&& filter,
                            std::vector<typename Index::entry_handle_type>& out) {
  out.clear();
  index.query(fr,
              std::forward<Filter>(filter),
              [&](const typename Index::entry_handle_type& handle,
                  const typename Index::entry_value_type&,
                  const geometry::aabb&) {
                out.push_back(handle);
                return true;
              });
  return out.size();
}

/**
 * @brief Collect handles of entries inside a frustum without a filter
 *
 * @tparam Index Entry index type
 * @param index Entry index
 * @param fr Frustum
 * @param out Output handles; cleared before the query
 * @return Number of collected handles
 */
template <EntryIndexType Index>
std::size_t
collect_visible(const Index& index, const geometry::frustum& fr, std::vector<typename Index::entry_handle_type>& out) {
  out.clear();
  index.query(fr,
              [&](const typename Index::entry_handle_type& handle,
                  const typename Index::entry_value_type&,
                  const geometry::aabb&) {
                out.push_back(handle);
                return true;
              });
  return out.size();
}

} // namespace spatial_index
} // namespace cyberverse
