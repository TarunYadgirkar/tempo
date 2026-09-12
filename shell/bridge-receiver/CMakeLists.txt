cmake_minimum_required(VERSION 3.16)
project(bridge_receiver C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SPATIAL_BRIDGE_ASAN "Build library and tests with AddressSanitizer" OFF)
if(SPATIAL_BRIDGE_ASAN)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(Threads REQUIRED)

# Find libjpeg-turbo. Prefer the turbojpeg pkg-config entry; fall back to the
# plain libjpeg pkg-config or find_package as shipped on Ubuntu 22.04/24.04.
find_package(PkgConfig QUIET)

set(JPEG_FOUND_VIA "")
set(JPEG_INCLUDE_DIRS "")
set(JPEG_LIBRARIES "")

if(PkgConfig_FOUND)
    pkg_check_modules(TURBOJPEG QUIET libturbojpeg)
    if(TURBOJPEG_FOUND)
        set(JPEG_FOUND_VIA "pkg-config libturbojpeg")
        set(JPEG_INCLUDE_DIRS ${TURBOJPEG_INCLUDE_DIRS})
        set(JPEG_LIBRARIES    ${TURBOJPEG_LIBRARIES})
    else()
        pkg_check_modules(LIBJPEG QUIET libjpeg)
        if(LIBJPEG_FOUND)
            set(JPEG_FOUND_VIA "pkg-config libjpeg")
            set(JPEG_INCLUDE_DIRS ${LIBJPEG_INCLUDE_DIRS})
            set(JPEG_LIBRARIES    ${LIBJPEG_LIBRARIES})
        endif()
    endif()
endif()

if(NOT JPEG_FOUND_VIA)
    find_package(JPEG REQUIRED)
    set(JPEG_FOUND_VIA "find_package(JPEG)")
    # JPEG_INCLUDE_DIRS and JPEG_LIBRARIES are set by find_package.
endif()

message(STATUS "JPEG found via: ${JPEG_FOUND_VIA}")
message(STATUS "JPEG include dirs: ${JPEG_INCLUDE_DIRS}")
message(STATUS "JPEG libraries: ${JPEG_LIBRARIES}")

# ---------------------------------------------------------------------------
# Library target: spatial_bridge (static)
# ---------------------------------------------------------------------------

add_library(spatial_bridge STATIC
    src/receiver.cpp
    src/world_model.cpp
    src/shm_pose.cpp
)

target_include_directories(spatial_bridge
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${JPEG_INCLUDE_DIRS}
)

# pkg-config reports a non-standard library dir on some platforms (e.g. Homebrew
# jpeg-turbo on macOS); add it so -lturbojpeg resolves. Empty/no-op on Linux.
target_link_directories(spatial_bridge PUBLIC ${TURBOJPEG_LIBRARY_DIRS} ${LIBJPEG_LIBRARY_DIRS})

target_link_libraries(spatial_bridge
    PUBLIC
        Threads::Threads
    PRIVATE
        ${JPEG_LIBRARIES}
        $<$<PLATFORM_ID:Linux>:rt>   # POSIX shm needs -lrt on Linux; in libSystem on macOS
        hand_reconstruct             # bone-length hand-joint depth reconstruction (runtime)
)

target_compile_options(spatial_bridge PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter   # libjpeg headers trigger this in our wrapper
)

# Position-independent code is required when spatial_bridge is linked into a
# shared object (e.g. Monado's SteamVR driver .so).
set_target_properties(spatial_bridge PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Library target: hand_reconstruct (static, pure C, no dependencies beyond libm)
# ---------------------------------------------------------------------------

add_library(hand_reconstruct STATIC
    src/hand_reconstruct.c
)

target_include_directories(hand_reconstruct
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(hand_reconstruct
    PUBLIC m
)

target_compile_options(hand_reconstruct PRIVATE -Wall -Wextra -Wno-unused-parameter)
set_target_properties(hand_reconstruct PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    C_STANDARD 11
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

enable_testing()
add_subdirectory(tests)
