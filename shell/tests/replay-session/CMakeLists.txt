cmake_minimum_required(VERSION 3.16)
project(replay_session CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# bridge-receiver dependency
# ---------------------------------------------------------------------------

# When built standalone (cmake -S tests/replay-session -B build/replay-session),
# we locate bridge-receiver relative to this file.
# When built as part of the top-level project, spatial_bridge is already defined.

if(NOT TARGET spatial_bridge)
    set(BRIDGE_RECEIVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../bridge-receiver")
    add_subdirectory("${BRIDGE_RECEIVER_DIR}"
                     "${CMAKE_CURRENT_BINARY_DIR}/bridge-receiver"
                     EXCLUDE_FROM_ALL)
endif()

# ---------------------------------------------------------------------------
# replay-session executable
# ---------------------------------------------------------------------------

add_executable(replay-session main.cpp)

target_link_libraries(replay-session PRIVATE spatial_bridge)

target_include_directories(replay-session PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../bridge-receiver/include"
)

target_compile_options(replay-session PRIVATE -Wall -Wextra)
