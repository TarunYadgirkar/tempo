cmake_minimum_required(VERSION 3.16)
project(gesture_engine CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Library target: gesture_engine (static)
# ---------------------------------------------------------------------------

add_library(gesture_engine STATIC
    src/ge_features.cpp
    src/engine.cpp
    src/calibrate.cpp
    src/gesture_tap.cpp
)

target_include_directories(gesture_engine
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(gesture_engine PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

set_target_properties(gesture_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------

add_executable(replay_hands tools/replay_hands.cpp)
target_link_libraries(replay_hands PRIVATE gesture_engine)

add_executable(print_default_config tools/print_default_config.cpp)
target_link_libraries(print_default_config PRIVATE gesture_engine)
set_target_properties(print_default_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_executable(calibrate tools/calibrate.cpp)
target_link_libraries(calibrate PRIVATE gesture_engine)
set_target_properties(calibrate PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

# eval_recording: replays an iPhone-recorded stream.bin (wire-protocol
# format with 0x05 hand packets) through the engine and emits one event
# per detected gesture.  Used by the recording → detection workflow:
# pair with the labels.json sidecar to compute precision/recall against
# the human-labelled ground truth.
# It reconstructs each hand's joint depths with the SAME runtime code the
# device uses (bridge-receiver's hr_reconstruct) before feeding the engine, so
# the eval sees exactly the metric-corrected joints the compositor would —
# never the raw, noisy LiDAR-unprojected joints stored verbatim in stream.bin.
# See docs/hand-tracking.md.
add_executable(eval_recording
    tools/eval_recording.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../bridge-receiver/src/hand_reconstruct.c
)
target_include_directories(eval_recording PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../bridge-receiver/include
)
target_link_libraries(eval_recording PRIVATE gesture_engine m)
set_target_properties(eval_recording PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

enable_testing()
add_subdirectory(tests)
