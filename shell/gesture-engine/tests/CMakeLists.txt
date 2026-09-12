cmake_minimum_required(VERSION 3.16)

add_executable(test_gestures test_gestures.cpp)

target_link_libraries(test_gestures PRIVATE gesture_engine)

target_include_directories(test_gestures PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

target_compile_options(test_gestures PRIVATE -Wall -Wextra -UNDEBUG)

add_test(
    NAME test_gestures
    COMMAND test_gestures
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

set_tests_properties(test_gestures PROPERTIES TIMEOUT 30)

# ---------------------------------------------------------------------------
# Config-loader tests (ge_load_config TOML parser)
# ---------------------------------------------------------------------------

add_executable(test_gestures_config test_config.cpp)

target_link_libraries(test_gestures_config PRIVATE gesture_engine)

target_include_directories(test_gestures_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_compile_options(test_gestures_config PRIVATE -Wall -Wextra -UNDEBUG)

target_compile_definitions(test_gestures_config PRIVATE
    GE_SAMPLE_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../sample-config/gestures.toml"
)

add_test(
    NAME test_gestures_config
    COMMAND test_gestures_config
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

set_tests_properties(test_gestures_config PROPERTIES TIMEOUT 30)

# ---------------------------------------------------------------------------
# Calibration synthetic-jitter tests
# ---------------------------------------------------------------------------

add_executable(test_calibrate test_calibrate.cpp)

target_link_libraries(test_calibrate PRIVATE gesture_engine)

target_include_directories(test_calibrate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_compile_options(test_calibrate PRIVATE -Wall -Wextra -UNDEBUG)

add_test(
    NAME test_calibrate
    COMMAND test_calibrate
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

set_tests_properties(test_calibrate PROPERTIES TIMEOUT 30)

# ---------------------------------------------------------------------------
# Tap-detection tests (per-fingertip plane tap)
# ---------------------------------------------------------------------------

add_executable(test_tap test_tap.cpp)

target_link_libraries(test_tap PRIVATE gesture_engine)

target_include_directories(test_tap PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_compile_options(test_tap PRIVATE -Wall -Wextra -UNDEBUG)

add_test(
    NAME test_tap
    COMMAND test_tap
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

set_tests_properties(test_tap PROPERTIES TIMEOUT 30)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(
    NAME test_eval_input
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_eval_input.py
            --eval-recording $<TARGET_FILE:eval_recording>
)
set_tests_properties(test_eval_input PROPERTIES TIMEOUT 30)

# ---------------------------------------------------------------------------
# Dataset SMOKE tests — run scripts/eval_dataset.py against the clip mirror
# at ~/spatial-os-recordings.  Skipped (NOT_RUN status) when the mirror is
# empty or absent, so a fresh checkout never fails CI.
#
# These are plumbing checks, NOT accuracy gates: the mirror on this machine
# holds only synthetic clips from gen_synthetic_session.py (synthetic
# joints + synthetic keyframes).  They prove that eval_recording runs
# end-to-end on every clip and that the harness parses and judges its
# output.  Do not read a pass here as a detection score and do not tune a
# threshold to make one pass.  A real score needs labelled iPhone
# recordings (see gesture-engine/CLAUDE.md, "Dataset eval").
#
# To run locally:
#   ctest --test-dir build/gesture-engine -R eval_dataset --verbose
# ---------------------------------------------------------------------------

set(SPATOS_RECORDINGS_DIR "$ENV{HOME}/spatial-os-recordings"
    CACHE PATH "Mirror dir for iPhone-recorded gesture clips")

add_test(
    NAME eval_dataset_smoke
    COMMAND ${CMAKE_COMMAND}
            -DSCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/eval_dataset.py
            -DEVAL_BIN=$<TARGET_FILE:eval_recording>
            -DMIRROR=${SPATOS_RECORDINGS_DIR}
            -DSPLIT=all
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_eval_dataset.cmake
)

set_tests_properties(eval_dataset_smoke PROPERTIES
    TIMEOUT 120
    SKIP_RETURN_CODE 2
)

# Per-split entries.  With real labelled recordings, the train split is
# the one to tune against and the test split is held out (measure only —
# never drive a tuning decision from it).  With the synthetic mirror they
# are the same smoke check restricted to a bucket.  Split algorithm and
# frozen manifest live in gesture-engine/tests/datasets/dataset_split.json.
add_test(
    NAME eval_dataset_train_smoke
    COMMAND ${CMAKE_COMMAND}
            -DSCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/eval_dataset.py
            -DEVAL_BIN=$<TARGET_FILE:eval_recording>
            -DMIRROR=${SPATOS_RECORDINGS_DIR}
            -DSPLIT=train
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_eval_dataset.cmake
)

set_tests_properties(eval_dataset_train_smoke PROPERTIES
    TIMEOUT 120
    SKIP_RETURN_CODE 2
)

add_test(
    NAME eval_dataset_test_smoke
    COMMAND ${CMAKE_COMMAND}
            -DSCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/eval_dataset.py
            -DEVAL_BIN=$<TARGET_FILE:eval_recording>
            -DMIRROR=${SPATOS_RECORDINGS_DIR}
            -DSPLIT=test
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_eval_dataset.cmake
)

set_tests_properties(eval_dataset_test_smoke PROPERTIES
    TIMEOUT 120
    SKIP_RETURN_CODE 2
)
