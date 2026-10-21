/**
 * @file entry_query_scratch.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Caller-owned query scratch
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cstddef>
#include <unordered_set>

#include "cyberverse/spatial_index/type_traits.hpp"

namespace cyberverse {
namespace spatial_index {

/**
 * @brief Reusable per-query handle set
 *
 *  The scratch is owned by the caller (normally one per thread) and is never stored in an index. A cell-based entry
 *  index uses it to visit each entry at most once when an entry is inserted into multiple cells.
 *
 * @tparam Handle Entry handle type
 */
template <EntryHandleType Handle>
class entry_query_scratch {
public:
  void clear() noexcept { seen_.clear(); }

  void reserve(std::size_t count) { seen_.reserve(count); }

  /**
   * @brief Mark a handle
   *
   * @param handle Handle to mark
   * @return true when the handle was not present before
   */
  bool mark(const Handle& handle) { return seen_.insert(handle).second; }

  std::size_t size() const noexcept { return seen_.size(); }

private:
  std::unordered_set<Handle> seen_;
};

} // namespace spatial_index
} // namespace cyberverse
