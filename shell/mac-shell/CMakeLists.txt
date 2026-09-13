cmake_minimum_required(VERSION 3.16)
project(mac_shell C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(SPATULA_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
set(WXRD_SRC ${SPATULA_ROOT}/vendor/wxrd/src)

# ---------------------------------------------------------------------------
# Dependency libraries: bridge-receiver + gesture-engine.
# Link the artifacts from build/<name>/ when another agent/session already
# built them; otherwise build them ourselves into build/mac-shell-deps/<name>.
# ---------------------------------------------------------------------------

set(MAC_SHELL_DEPS_DIR ${SPATULA_ROOT}/build/mac-shell-deps)

function(spatula_dep_dir name src_dir libs out_var)
    set(primary ${SPATULA_ROOT}/build/${name})
    set(found TRUE)
    foreach(lib ${libs})
        if(NOT EXISTS ${primary}/${lib})
            set(found FALSE)
        endif()
    endforeach()
    if(found)
        message(STATUS "mac-shell: using prebuilt ${name} from ${primary}")
        set(${out_var} ${primary} PARENT_SCOPE)
        return()
    endif()

    set(fallback ${MAC_SHELL_DEPS_DIR}/${name})
    set(found TRUE)
    foreach(lib ${libs})
        if(NOT EXISTS ${fallback}/${lib})
            set(found FALSE)
        endif()
    endforeach()
    if(NOT found)
        message(STATUS "mac-shell: building dependency ${name} into ${fallback}")
        execute_process(
            COMMAND ${CMAKE_COMMAND} -G Ninja -S ${src_dir} -B ${fallback}
                    -DCMAKE_BUILD_TYPE=RelWithDebInfo
            RESULT_VARIABLE rc)
        if(NOT rc EQUAL 0)
            message(FATAL_ERROR "mac-shell: configuring ${name} failed")
        endif()
        execute_process(
            COMMAND ${CMAKE_COMMAND} --build ${fallback}
            RESULT_VARIABLE rc)
        if(NOT rc EQUAL 0)
            message(FATAL_ERROR "mac-shell: building ${name} failed")
        endif()
    endif()
    set(${out_var} ${fallback} PARENT_SCOPE)
endfunction()

spatula_dep_dir(bridge-receiver ${SPATULA_ROOT}/bridge-receiver
    "libspatial_bridge.a;libhand_reconstruct.a" BRIDGE_BUILD_DIR)
spatula_dep_dir(gesture-engine ${SPATULA_ROOT}/gesture-engine
    "libgesture_engine.a" GESTURE_BUILD_DIR)

find_package(Threads REQUIRED)

# JPEG (transitively required by libspatial_bridge.a) — same lookup order as
# bridge-receiver/CMakeLists.txt.
find_package(PkgConfig QUIET)
set(JPEG_LIBRARIES "")
set(JPEG_LIBRARY_DIRS "")
if(PkgConfig_FOUND)
    pkg_check_modules(TURBOJPEG QUIET libturbojpeg)
    if(TURBOJPEG_FOUND)
        set(JPEG_LIBRARIES ${TURBOJPEG_LIBRARIES})
        set(JPEG_LIBRARY_DIRS ${TURBOJPEG_LIBRARY_DIRS})
    else()
        pkg_check_modules(LIBJPEG QUIET libjpeg)
        if(LIBJPEG_FOUND)
            set(JPEG_LIBRARIES ${LIBJPEG_LIBRARIES})
            set(JPEG_LIBRARY_DIRS ${LIBJPEG_LIBRARY_DIRS})
        endif()
    endif()
endif()
if(NOT JPEG_LIBRARIES)
    find_package(JPEG REQUIRED)
endif()

add_library(spatial_bridge_import STATIC IMPORTED)
set_target_properties(spatial_bridge_import PROPERTIES
    IMPORTED_LOCATION ${BRIDGE_BUILD_DIR}/libspatial_bridge.a
    INTERFACE_INCLUDE_DIRECTORIES ${SPATULA_ROOT}/bridge-receiver/include)

add_library(hand_reconstruct_import STATIC IMPORTED)
set_target_properties(hand_reconstruct_import PROPERTIES
    IMPORTED_LOCATION ${BRIDGE_BUILD_DIR}/libhand_reconstruct.a)

add_library(gesture_engine_import STATIC IMPORTED)
set_target_properties(gesture_engine_import PROPERTIES
    IMPORTED_LOCATION ${GESTURE_BUILD_DIR}/libgesture_engine.a
    INTERFACE_INCLUDE_DIRECTORIES ${SPATULA_ROOT}/gesture-engine/include)

# ---------------------------------------------------------------------------
# ctl_wire: the wxrd control-plane wire layer compiled verbatim.
# ---------------------------------------------------------------------------

add_library(ctl_wire STATIC
    ${WXRD_SRC}/control_protocol.c
    ${WXRD_SRC}/control_conn.c
)
target_include_directories(ctl_wire PUBLIC ${WXRD_SRC})
target_compile_options(ctl_wire PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# scene_core: pure C++ world state (no Metal, no AppKit).
# ---------------------------------------------------------------------------

set(KBD_DECODER_SRC ${SPATULA_ROOT}/spatial-shell/keyboard/decoder)
set(RADIAL_MENU_SRC ${SPATULA_ROOT}/spatial-shell/radial-menu)

add_library(scene_core STATIC
    src/core/anchor_math.cpp
    src/core/depth_cast.cpp
    src/core/hand_inject.cpp
    src/core/json_lite.cpp
    src/core/layout_store.cpp
    src/core/scene.cpp
    src/ui/font_atlas.cpp
    src/ui/note_card.cpp
    src/ui/keyboard_overlay.cpp
    src/ui/launcher_menu.cpp
    src/ui/panel_surface.cpp
    # Pure-C reuse: plunge typing decoder, radial-menu model, fist-roll tracker.
    ${KBD_DECODER_SRC}/decoder.c
    ${KBD_DECODER_SRC}/decoder_config.c
    ${KBD_DECODER_SRC}/keyboard_geom.c
    ${KBD_DECODER_SRC}/keyboard_layers.c
    ${KBD_DECODER_SRC}/lm.c
    ${RADIAL_MENU_SRC}/radial_menu.c
    ${RADIAL_MENU_SRC}/menubar_items.c
    ${WXRD_SRC}/fist_rotation.c
)
target_include_directories(scene_core PUBLIC src
    ${KBD_DECODER_SRC} ${RADIAL_MENU_SRC} ${WXRD_SRC})
target_link_directories(scene_core PUBLIC ${JPEG_LIBRARY_DIRS})
target_link_libraries(scene_core
    PUBLIC
        spatial_bridge_import
        gesture_engine_import
        hand_reconstruct_import
        Threads::Threads
        ${JPEG_LIBRARIES}
)
target_compile_options(scene_core PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# mac-shell binary (+ Stage 2 Metal renderer / ScreenCaptureKit on macOS)
# ---------------------------------------------------------------------------

option(MAC_SHELL_RENDERER "Build the Metal renderer + ScreenCaptureKit panels"
       ${APPLE})

add_executable(mac-shell
    src/platform/main.cpp
    src/platform/control_server.cpp
    src/platform/shot_path.cpp
)
target_link_libraries(mac-shell PRIVATE scene_core ctl_wire)
target_compile_options(mac-shell PRIVATE -Wall -Wextra)

if(APPLE AND MAC_SHELL_RENDERER)
    enable_language(OBJCXX)
    set(CMAKE_OBJCXX_STANDARD 17)
    set(MAC_SHELL_RENDERER_SRC src/platform/renderer.mm
        src/platform/renderer_panels.mm src/platform/renderer_keyboard.mm
        src/platform/renderer_hands.mm src/platform/renderer_dock.mm
        src/platform/renderer_overlays.mm src/platform/app_main.mm
        src/platform/capture.mm src/platform/png_write.mm
        src/platform/frame_export.mm)
    target_sources(mac-shell PRIVATE ${MAC_SHELL_RENDERER_SRC})
    set_source_files_properties(${MAC_SHELL_RENDERER_SRC} PROPERTIES
        COMPILE_OPTIONS "-fobjc-arc")
    # Short commit hash for the About panel, refreshed every build (a
    # configure-time snapshot goes stale across incremental builds). The
    # header is rewritten only when the hash changes so renderer.mm does not
    # recompile every time; it reaches renderer.mm via -include.
    set(MAC_SHELL_COMMIT_HDR ${CMAKE_BINARY_DIR}/generated/mac_shell_commit.h)
    set(MAC_SHELL_COMMIT_SCRIPT ${CMAKE_BINARY_DIR}/generated/mac_shell_commit.cmake)
    file(WRITE ${MAC_SHELL_COMMIT_SCRIPT}
        "execute_process(COMMAND git rev-parse --short HEAD\n"
        "    WORKING_DIRECTORY \"${SPATULA_ROOT}\"\n"
        "    OUTPUT_VARIABLE hash OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)\n"
        "if(NOT hash)\n  set(hash unknown)\nendif()\n"
        "set(content \"#define MAC_SHELL_COMMIT \\\"\${hash}\\\"\\n\")\n"
        "set(existing \"\")\n"
        "if(EXISTS \"${MAC_SHELL_COMMIT_HDR}\")\n"
        "  file(READ \"${MAC_SHELL_COMMIT_HDR}\" existing)\n"
        "endif()\n"
        "if(NOT existing STREQUAL content)\n"
        "  file(WRITE \"${MAC_SHELL_COMMIT_HDR}\" \"\${content}\")\n"
        "endif()\n")
    add_custom_target(mac_shell_commit
        COMMAND ${CMAKE_COMMAND} -P ${MAC_SHELL_COMMIT_SCRIPT}
        BYPRODUCTS ${MAC_SHELL_COMMIT_HDR}
        COMMENT "Refreshing mac_shell_commit.h")
    add_dependencies(mac-shell mac_shell_commit)
    set_source_files_properties(src/platform/renderer.mm PROPERTIES
        COMPILE_OPTIONS "-fobjc-arc;-include;${MAC_SHELL_COMMIT_HDR}"
        OBJECT_DEPENDS ${MAC_SHELL_COMMIT_HDR})
    target_compile_definitions(mac-shell PRIVATE MAC_SHELL_HAVE_RENDERER=1)
    target_link_libraries(mac-shell PRIVATE
        "-framework AppKit"
        "-framework Metal"
        "-framework MetalKit"
        "-framework QuartzCore"
        "-framework CoreVideo"
        "-framework CoreMedia"
        "-framework CoreGraphics"
        "-framework ImageIO"
        "-framework UniformTypeIdentifiers"
        "-framework ScreenCaptureKit"
        "-framework ApplicationServices"
        "-framework SystemConfiguration"
    )

    # Embed shaders.metal as a C string; the renderer compiles it at runtime
    # with newLibraryWithSource (the offline `metal` toolchain is an optional
    # Xcode download and absent on CI boxes).
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/shaders.metal MAC_SHELL_SHADER_SRC)
    file(WRITE ${CMAKE_BINARY_DIR}/generated/shaders_metal.h
        "// generated from mac-shell/shaders.metal — do not edit\n"
        "static const char *MAC_SHELL_SHADER_SOURCE = R\"MSL(\n"
        "${MAC_SHELL_SHADER_SRC}\n"
        ")MSL\";\n")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders.metal)
    target_include_directories(mac-shell PRIVATE ${CMAKE_BINARY_DIR}/generated)

    # -----------------------------------------------------------------------
    # Spatula app bundle: same sources as the CLI binary (which stays for
    # CI/headless), packaged as an app so TCC attributes Screen Recording /
    # Accessibility grants to a stable identity instead of the launching
    # terminal. Launch via `open` (scripts/run-mac.sh) so launchd — not the
    # shell — is the responsible process.
    #
    # FINAL IDENTITY — DO NOT CHANGE: bundle id `com.spatialos.spatula`,
    # app name "Spatula" (Info.plist). macOS TCC keys permission grants to
    # bundle id + signature; changing either resets every grant and leaves
    # stale rows in System Settings. The executable stays `mac-shell`
    # internally (CFBundleExecutable) — that part is free to keep.
    # scripts/install-mac.sh signs this bundle and rsyncs it to
    # /Applications/Spatula.app, which is the copy users grant and launch.
    # -----------------------------------------------------------------------
    set(MAC_SHELL_CODESIGN_ID "-" CACHE STRING
        "codesign identity for the Spatula app bundle ('-' = ad-hoc; ad-hoc \
grants must be re-approved after each rebuild — use an Apple Development \
identity for a stable grant; scripts/install-mac.sh auto-detects one)")

    set(MAC_SHELL_ICON ${CMAKE_CURRENT_SOURCE_DIR}/resources/Spatula.icns)
    set_source_files_properties(${MAC_SHELL_ICON} PROPERTIES
        MACOSX_PACKAGE_LOCATION Resources)

    add_executable(mac-shell-app MACOSX_BUNDLE
        src/platform/main.cpp
        src/platform/control_server.cpp
        src/platform/shot_path.cpp
        ${MAC_SHELL_RENDERER_SRC}
        ${MAC_SHELL_ICON}
    )
    set_target_properties(mac-shell-app PROPERTIES
        OUTPUT_NAME "mac-shell"
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Info.plist)
    target_compile_definitions(mac-shell-app PRIVATE MAC_SHELL_HAVE_RENDERER=1)
    add_dependencies(mac-shell-app mac_shell_commit)
    target_compile_options(mac-shell-app PRIVATE -Wall -Wextra)
    target_include_directories(mac-shell-app PRIVATE
        ${CMAKE_BINARY_DIR}/generated)
    get_target_property(MAC_SHELL_LINK_LIBS mac-shell LINK_LIBRARIES)
    target_link_libraries(mac-shell-app PRIVATE ${MAC_SHELL_LINK_LIBS})

    add_custom_command(TARGET mac-shell-app POST_BUILD
        COMMAND codesign --force --sign "${MAC_SHELL_CODESIGN_ID}"
                --entitlements ${CMAKE_CURRENT_SOURCE_DIR}/mac-shell.entitlements
                "$<TARGET_BUNDLE_DIR:mac-shell-app>"
        COMMENT "codesign mac-shell.app (${MAC_SHELL_CODESIGN_ID})"
        VERBATIM)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

enable_testing()

add_executable(test_anchor_math tests/test_anchor_math.cpp)
target_link_libraries(test_anchor_math PRIVATE scene_core)
add_test(NAME anchor_math COMMAND test_anchor_math)

add_executable(test_scene tests/test_scene.cpp)
target_link_libraries(test_scene PRIVATE scene_core)
add_test(NAME scene COMMAND test_scene)

add_executable(test_hud_state tests/test_hud_state.cpp)
target_link_libraries(test_hud_state PRIVATE scene_core)
add_test(NAME hud_state COMMAND test_hud_state)

add_executable(test_depth_math tests/test_depth_math.cpp)
target_link_libraries(test_depth_math PRIVATE scene_core)
add_test(NAME depth_math COMMAND test_depth_math)

add_executable(test_depth_cast tests/test_depth_cast.cpp)
target_link_libraries(test_depth_cast PRIVATE scene_core)
target_compile_options(test_depth_cast PRIVATE -Wall -Wextra)
add_test(NAME depth_cast COMMAND test_depth_cast)

add_executable(test_placement tests/test_placement.cpp)
target_link_libraries(test_placement PRIVATE scene_core)
add_test(NAME placement COMMAND test_placement)

add_executable(test_keyboard tests/test_keyboard.cpp)
target_link_libraries(test_keyboard PRIVATE scene_core)
add_test(NAME keyboard COMMAND test_keyboard)
set_tests_properties(keyboard PROPERTIES TIMEOUT 60)

add_executable(test_hand_inject tests/test_hand_inject.cpp)
target_link_libraries(test_hand_inject PRIVATE scene_core)
target_compile_options(test_hand_inject PRIVATE -Wall -Wextra)
add_test(NAME hand_inject COMMAND test_hand_inject)

add_executable(test_hands_e2e tests/test_hands_e2e.cpp)
target_include_directories(test_hands_e2e PRIVATE
    ${SPATULA_ROOT}/bridge-receiver/tests)
target_compile_options(test_hands_e2e PRIVATE -Wall -Wextra)
target_compile_definitions(test_hands_e2e PRIVATE
    MAC_SHELL_BIN="$<TARGET_FILE:mac-shell>")
add_dependencies(test_hands_e2e mac-shell)
add_test(NAME hands_e2e COMMAND test_hands_e2e)
set_tests_properties(hands_e2e PROPERTIES TIMEOUT 120)

add_executable(test_note_layout tests/test_note_layout.cpp)
target_link_libraries(test_note_layout PRIVATE scene_core)
target_compile_options(test_note_layout PRIVATE -Wall -Wextra)
add_test(NAME note_layout COMMAND test_note_layout)

add_executable(test_launcher tests/test_launcher.cpp)
target_link_libraries(test_launcher PRIVATE scene_core)
add_test(NAME launcher COMMAND test_launcher)
set_tests_properties(launcher PROPERTIES TIMEOUT 60)

add_executable(test_control_socket_busy tests/test_control_socket_busy.cpp
    src/platform/control_server.cpp src/platform/shot_path.cpp)
target_link_libraries(test_control_socket_busy PRIVATE scene_core ctl_wire)
target_compile_options(test_control_socket_busy PRIVATE -Wall -Wextra)
add_test(NAME control_socket_busy COMMAND test_control_socket_busy)
set_tests_properties(control_socket_busy PROPERTIES TIMEOUT 60)

add_executable(test_control_e2e tests/test_control_e2e.cpp)
target_link_libraries(test_control_e2e PRIVATE scene_core ctl_wire)
target_compile_definitions(test_control_e2e PRIVATE
    MAC_SHELL_BIN="$<TARGET_FILE:mac-shell>")
add_dependencies(test_control_e2e mac-shell)
add_test(NAME control_e2e COMMAND test_control_e2e)
set_tests_properties(control_e2e PROPERTIES TIMEOUT 60)

add_executable(test_hack_verbs_e2e tests/test_hack_verbs_e2e.cpp)
target_link_libraries(test_hack_verbs_e2e PRIVATE scene_core ctl_wire)
target_compile_options(test_hack_verbs_e2e PRIVATE -Wall -Wextra)
target_compile_definitions(test_hack_verbs_e2e PRIVATE
    MAC_SHELL_BIN="$<TARGET_FILE:mac-shell>")
add_dependencies(test_hack_verbs_e2e mac-shell)
add_test(NAME hack_verbs_e2e COMMAND test_hack_verbs_e2e)
set_tests_properties(hack_verbs_e2e PROPERTIES TIMEOUT 60)

add_executable(test_cast_e2e tests/test_cast_e2e.cpp)
target_compile_options(test_cast_e2e PRIVATE -Wall -Wextra)
target_compile_definitions(test_cast_e2e PRIVATE
    MAC_SHELL_BIN="$<TARGET_FILE:mac-shell>")
add_dependencies(test_cast_e2e mac-shell)
add_test(NAME cast_e2e COMMAND test_cast_e2e)
set_tests_properties(cast_e2e PROPERTIES TIMEOUT 120)

add_executable(test_port_retry_e2e tests/test_port_retry_e2e.cpp)
target_compile_definitions(test_port_retry_e2e PRIVATE
    MAC_SHELL_BIN="$<TARGET_FILE:mac-shell>")
target_compile_options(test_port_retry_e2e PRIVATE -Wall -Wextra)
add_dependencies(test_port_retry_e2e mac-shell)
add_test(NAME port_retry_e2e COMMAND test_port_retry_e2e)
set_tests_properties(port_retry_e2e PROPERTIES TIMEOUT 60)

add_executable(test_shot_path tests/test_shot_path.cpp
    src/platform/shot_path.cpp)
target_include_directories(test_shot_path PRIVATE src)
target_compile_options(test_shot_path PRIVATE -Wall -Wextra)
add_test(NAME shot_path COMMAND test_shot_path)

add_executable(test_ui_e2e tests/test_ui_e2e.cpp)
target_link_libraries(test_ui_e2e PRIVATE scene_core ctl_wire)
target_compile_definitions(test_ui_e2e PRIVATE
    MAC_SHELL_BIN="$<TARGET_FILE:mac-shell>")
add_dependencies(test_ui_e2e mac-shell)
add_test(NAME ui_e2e COMMAND test_ui_e2e)
set_tests_properties(ui_e2e PROPERTIES TIMEOUT 120)

if(APPLE AND MAC_SHELL_RENDERER)
    # Runtime shader + pipeline build on the real Metal device (headless).
    add_executable(test_shader_compile tests/test_shader_compile.mm)
    set_source_files_properties(tests/test_shader_compile.mm PROPERTIES
        COMPILE_OPTIONS "-fobjc-arc")
    target_include_directories(test_shader_compile PRIVATE
        ${CMAKE_BINARY_DIR}/generated)
    target_link_libraries(test_shader_compile PRIVATE "-framework Metal"
        "-framework Foundation")
    add_test(NAME shader_compile COMMAND test_shader_compile)

    # Capture permission degrade path (headless-safe: never prompts).
    add_executable(test_capture_degrade tests/test_capture_degrade.cpp)
    target_include_directories(test_capture_degrade PRIVATE src)
    target_compile_definitions(test_capture_degrade PRIVATE
        MAC_SHELL_BIN="$<TARGET_FILE:mac-shell>")
    add_dependencies(test_capture_degrade mac-shell)
    add_test(NAME capture_degrade COMMAND test_capture_degrade)
    set_tests_properties(capture_degrade PROPERTIES TIMEOUT 60)

    # `screenshot` verb against a real window. Skips itself (ctest SKIP) when
    # no display is attached — the only mac-shell test that needs one.
    add_executable(test_screenshot_e2e tests/test_screenshot_e2e.cpp)
    target_include_directories(test_screenshot_e2e PRIVATE src)
    target_compile_options(test_screenshot_e2e PRIVATE -Wall -Wextra)
    target_link_libraries(test_screenshot_e2e PRIVATE
        "-framework CoreGraphics" "-framework Foundation")
    target_compile_definitions(test_screenshot_e2e PRIVATE
        MAC_SHELL_BIN="$<TARGET_FILE:mac-shell>")
    add_dependencies(test_screenshot_e2e mac-shell)
    add_test(NAME screenshot_e2e COMMAND test_screenshot_e2e)
    set_tests_properties(screenshot_e2e PROPERTIES TIMEOUT 120
        SKIP_RETURN_CODE 77)
endif()
