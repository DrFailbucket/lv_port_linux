cmake_minimum_required(VERSION 3.10)
project(lvgl)

cmake_policy(SET CMP0079 NEW)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic")

set(LV_BUILD_SET_CONFIG_OPTS ON CACHE BOOL "create CMAKE variables from lv_conf_internal.h" FORCE)
set(LV_CONF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(LV_CONF_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
# --- Manuelle Backend-Auswahl ---
set(CONFIG_LV_USE_EVDEV ON CACHE BOOL "" FORCE)
set(CONFIG_LV_USE_LINUX_FBDEV ON CACHE BOOL "" FORCE)

# alles andere abschalten
set(CONFIG_LV_USE_SDL OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_USE_DRM OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_USE_X11 OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_USE_WAYLAND OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_USE_LIBINPUT OFF CACHE BOOL "" FORCE)

add_subdirectory(lvgl)

# --- EVDEV (Touch Input) ---
if (CONFIG_LV_USE_EVDEV)
    message("Including EVDEV support")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EVDEV REQUIRED libevdev)
    list(APPEND PKG_CONFIG_LIB ${EVDEV_LIBRARIES})
    list(APPEND PKG_CONFIG_INC ${EVDEV_INCLUDE_DIRS})
    list(APPEND LV_LINUX_BACKEND_SRC src/lib/indev_backends/evdev.c)
endif()

# --- FBDEV (Framebuffer Display) ---
if (CONFIG_LV_USE_LINUX_FBDEV)
    message("Including FBDEV support")
    list(APPEND LV_LINUX_BACKEND_SRC src/lib/display_backends/fbdev.c)
endif()

# --- Basis LVGL-Linux-Integration ---
file(GLOB LV_LINUX_SRC src/lib/*.c)
set(LV_LINUX_INC src/lib)

target_include_directories(lvgl PUBLIC ${PKG_CONFIG_INC})

add_library(lvgl_linux STATIC ${LV_LINUX_SRC} ${LV_LINUX_BACKEND_SRC})

set_target_properties(lvgl_linux PROPERTIES COMPILE_DEFINITIONS "${LVGL_COMPILER_DEFINES}")
target_include_directories(lvgl_linux PUBLIC
        ${LV_LINUX_INC}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/src/lib
        ${LVGL_CONF_INC_DIR}
)

target_link_libraries(lvgl PUBLIC ${PKG_CONFIG_LIB} m pthread)

# --- Dein eigenes UI-Projekt ---
add_executable(lvgl_app
        src/main.c
        src/ui/ui.c
        src/ui/ui_helpers.c
        src/ui/ui_theme_manager.c
        src/ui/ui_themes.c
        src/ui/ui_events.c
        src/ui/screens/ui_Screen3.c
        src/ui/components/ui_comp_hook.c
        ${LV_LINUX_SRC}
        ${LV_LINUX_BACKEND_SRC}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CJSON REQUIRED libcjson)

target_link_libraries(lvgl_app
        lvgl_linux
        lvgl
        m
        pthread
        curl
        ${CJSON_LIBRARIES}
)

target_include_directories(lvgl_app PRIVATE
        src/ui
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CJSON_INCLUDE_DIRS}
)

# --- Build Targets ---
add_custom_target(run COMMAND ${EXECUTABLE_OUTPUT_PATH}/lvgl_app DEPENDS lvgl_app)
