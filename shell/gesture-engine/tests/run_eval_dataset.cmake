# run_eval_dataset.cmake — Driver invoked by the eval_dataset ctest entry.
#
# CMake's add_test doesn't natively let us skip-when-mirror-missing, so
# we wrap the Python harness in a small CMake script that returns exit
# code 2 ("SKIP" per SKIP_RETURN_CODE) when the recordings directory is
# empty or absent.  The Python harness itself also returns 2 in that
# case — this wrapper just makes the absence visible at CMake level so
# we don't even try to invoke Python on a host that doesn't have it.

if(NOT EXISTS "${MIRROR}")
    message(STATUS "eval_dataset: mirror dir ${MIRROR} not present — SKIP")
    cmake_language(EXIT 2)
endif()

file(GLOB clips RELATIVE "${MIRROR}" "${MIRROR}/*")
list(FILTER clips EXCLUDE REGEX "^\\.")
if(clips STREQUAL "")
    message(STATUS "eval_dataset: mirror dir ${MIRROR} is empty — SKIP")
    cmake_language(EXIT 2)
endif()

find_program(PY3 python3 REQUIRED)

if(NOT DEFINED SPLIT OR SPLIT STREQUAL "")
    set(SPLIT "all")
endif()

execute_process(
    COMMAND ${PY3} ${SCRIPT}
            --eval-recording ${EVAL_BIN}
            --mirror ${MIRROR}
            --split ${SPLIT}
    RESULT_VARIABLE rc
)

if(rc EQUAL 2)
    cmake_language(EXIT 2)        # mirror became empty mid-run; SKIP
elseif(rc EQUAL 0)
    cmake_language(EXIT 0)
else()
    cmake_language(EXIT ${rc})    # any failure → ctest fail
endif()
