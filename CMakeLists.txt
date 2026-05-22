cmake_minimum_required(VERSION 3.16)
project(oled_driver VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Dependencies ──────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# ── Library ───────────────────────────────────────────────────
add_library(oled_driver STATIC
    src/oled_driver.cpp
)

target_include_directories(oled_driver
    PUBLIC  include
    PRIVATE ${GPIOD_INCLUDE_DIRS}
)

target_link_libraries(oled_driver
    PUBLIC  pthread
    PRIVATE ${GPIOD_LIBRARIES}
)

target_compile_options(oled_driver PRIVATE
    -Wall -Wextra -Wpedantic -O2
)

# ── Example ───────────────────────────────────────────────────
add_subdirectory(example)