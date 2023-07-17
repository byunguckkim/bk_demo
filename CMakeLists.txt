cmake_minimum_required(VERSION 3.8)

project(simian_bk_demo C CXX)

# Enable verbose output from builds.
set(CMAKE_VERBOSE_MAKEFILE ON)
# Set default compilation flags to be used when compiling C++ files.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
message("MSVC Compiler, setting MSVC specific compile flags")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -WX")
endif()
set(CMAKE_CXX_STANDARD 17)

# By default, add_library will create shared libraries
if (NOT BUILD_SHARED_LIBS)
  set (BUILD_SHARED_LIBS True)
endif()


# If applicable, set Applied-specific CMake options here.
set(APPLIED_ENABLE_MAP_UTILS OFF CACHE BOOL "" FORCE)
set(APPLIED_ENABLE_SPECTRAL_UTILS ON CACHE BOOL "" FORCE)
set(APPLIED_ENABLE_FMU_UTILS OFF CACHE BOOL "" FORCE)
set(APPLIED_ENABLE_PUBLIC_MAP_SERVICE OFF CACHE BOOL "" FORCE)
set(USE_ROS OFF CACHE BOOL "" FORCE)

add_subdirectory(applied)

# Build customer_interface shared library
add_library(
  customer_interface SHARED
  interface/adp_bridge.cc
  interface/log_reader.cc
  $<TARGET_OBJECTS:applied::glue_cxx>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
target_link_libraries(customer_interface PUBLIC
  applied::interface_cxx
  stdc++fs
)
else()
target_link_libraries(customer_interface PUBLIC
  applied::interface_cxx
)
endif()


target_include_directories(customer_interface PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
)

set_target_properties(customer_interface PROPERTIES PREFIX "")
# End build customer_interface shared library