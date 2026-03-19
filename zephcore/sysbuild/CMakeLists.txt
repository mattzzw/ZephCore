# Sysbuild resolves the board BEFORE the application CMakeLists.txt runs,
# so the BOARD_ROOT set there never fires. Set it here, before find_package(Sysbuild).
set(BOARD_ROOT "${APP_DIR}" CACHE STRING "" FORCE)

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sysbuild LANGUAGES)
