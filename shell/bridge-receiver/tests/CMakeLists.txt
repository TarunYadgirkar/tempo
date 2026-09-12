cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# test_packets
# ---------------------------------------------------------------------------

add_executable(test_packets
    test_packets.cpp
)

target_link_libraries(test_packets PRIVATE spatial_bridge)

target_include_directories(test_packets PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# -UNDEBUG ensures assert() is always active regardless of build type.
target_compile_options(test_packets PRIVATE -Wall -Wextra -UNDEBUG)

add_test(
    NAME test_packets
    COMMAND test_packets
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# ---------------------------------------------------------------------------
# test_world_model
# ---------------------------------------------------------------------------

add_executable(test_world_model
    test_world_model.cpp
)

target_link_libraries(test_world_model PRIVATE spatial_bridge)

target_include_directories(test_world_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# -UNDEBUG ensures assert() is always active regardless of build type.
target_compile_options(test_world_model PRIVATE -Wall -Wextra -UNDEBUG)

add_test(
    NAME test_world_model
    COMMAND test_world_model
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# Give the stress test enough time to run (default CTest timeout is 1500s, but
# let's be explicit: our stress test runs for 3 seconds, so 30s is generous).
set_tests_properties(test_packets PROPERTIES TIMEOUT 30)
set_tests_properties(test_world_model PROPERTIES TIMEOUT 30)
