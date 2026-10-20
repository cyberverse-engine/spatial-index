cmake_minimum_required(VERSION 4.0.0)

# cyberverse::geometry::static_lib (and GLM transitively) is resolved by the root CMakeLists.txt.
add_library(cyberverse_spatial_index_static_lib STATIC)
add_library(cyberverse::spatial_index::static_lib ALIAS cyberverse_spatial_index_static_lib)

target_include_directories(cyberverse_spatial_index_static_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../src)

# Header files
set(
  HEADER_FILES
  cyberverse/spatial_index.hpp
  cyberverse/spatial_index/dense_uniform_grid.hpp
  cyberverse/spatial_index/dense_uniform_grid.inl
  cyberverse/spatial_index/sparse_uniform_grid.hpp
  cyberverse/spatial_index/sparse_uniform_grid.inl
  cyberverse/spatial_index/type_traits.hpp
  cyberverse/spatial_index/detail/keyed_array.hpp
  cyberverse/spatial_index/detail/uniform_grid_impl.hpp
  cyberverse/spatial_index/detail/uniform_grid_node.hpp
  cyberverse/spatial_index/detail/uniform_grid_node.inl
)

# Source files
set(
  SOURCE_FILES
  cyberverse/spatial_index/spatial_index.cpp
)

target_sources(cyberverse_spatial_index_static_lib PUBLIC ${HEADER_FILES})
target_sources(cyberverse_spatial_index_static_lib PRIVATE ${SOURCE_FILES})

target_link_libraries(
  cyberverse_spatial_index_static_lib
  PUBLIC
  cyberverse::geometry::static_lib
  glm::glm
)
