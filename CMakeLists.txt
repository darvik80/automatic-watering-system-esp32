# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(ColourBold  "${Esc}[1m")
    set(Red         "${Esc}[31m")
    set(Green       "${Esc}[32m")
    set(Yellow      "${Esc}[33m")
    set(Blue        "${Esc}[34m")
    set(Magenta     "${Esc}[35m")
    set(Cyan        "${Esc}[36m")
    set(White       "${Esc}[37m")
    set(BoldRed     "${Esc}[1;31m")
    set(BoldGreen   "${Esc}[1;32m")
    set(BoldYellow  "${Esc}[1;33m")
    set(BoldBlue    "${Esc}[1;34m")
    set(BoldMagenta "${Esc}[1;35m")
    set(BoldCyan    "${Esc}[1;36m")
    set(BoldWhite   "${Esc}[1;37m")
endif()

message(STATUS "$ENV{HOME}/esp/esp-idf")

message(STATUS "${Red}${ColourBold}Target ${IDF_TARGET}${ColourReset}")
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

if (${IDF_TARGET} MATCHES "esp32c3")
    # esp32-c3
    list(APPEND CMAKE_PROGRAM_PATH "$ENV{HOME}/.espressif/tools/riscv32-esp-elf/esp-2021r2-patch3-8.4.0/riscv32-esp-elf/bin")
    set(CMAKE_C_COMPILER "riscv32-esp-elf-gcc")
    set(CMAKE_CXX_COMPILER "riscv32-esp-elf-g++")
    set(CMAKE_CXX_FLAGS "-std=c++17 -std=gnu++17 -Wno-missing-field-initializers -Wno-frame-address -fexceptions -fno-rtti -ffunction-sections -fdata-sections -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=deprecated-declarations -Wno-unused-parameter -Wno-sign-compare -ggdb -Os -freorder-blocks -Wwrite-strings -fstack-protector -fstrict-volatile-bitfields -Wno-error=unused-but-set-variable -fno-jump-tables -fno-tree-switch-conversion -MMD")
    set(CMAKE_C_FLAGS "-Wno-frame-address -std=gnu99 -Wno-old-style-declaration -ffunction-sections -fdata-sections -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=deprecated-declarations -Wno-unused-parameter -Wno-sign-compare -ggdb -Os -freorder-blocks -Wwrite-strings -fstack-protector -fstrict-volatile-bitfields -Wno-error=unused-but-set-variable -fno-jump-tables -fno-tree-switch-conversion -MMD")
else()
    # esp32
    list(APPEND CMAKE_PROGRAM_PATH  "$ENV{HOME}/.espressif/tools/xtensa-esp32-elf/esp-2021r2-patch5-8.4.0/xtensa-esp32-elf/bin")
    set(CMAKE_C_COMPILER "xtensa-esp32-elf-gcc")
    set(CMAKE_CXX_COMPILER "xtensa-esp32-elf-g++")
    set(CMAKE_CXX_FLAGS "-std=c++17 -std=gnu++17 -mlongcalls -Wno-missing-field-initializers -Wno-frame-address -fexceptions -fno-rtti -ffunction-sections -fdata-sections -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=deprecated-declarations -Wno-unused-parameter -Wno-sign-compare -ggdb -Os -freorder-blocks -Wwrite-strings -fstack-protector -fstrict-volatile-bitfields -Wno-error=unused-but-set-variable -fno-jump-tables -fno-tree-switch-conversion -MMD")
    set(CMAKE_C_FLAGS "-mlongcalls -Wno-frame-address -std=gnu99 -Wno-old-style-declaration -ffunction-sections -fdata-sections -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=deprecated-declarations -Wno-unused-parameter -Wno-sign-compare -ggdb -Os -freorder-blocks -Wwrite-strings -fstack-protector -fstrict-volatile-bitfields -Wno-error=unused-but-set-variable -fno-jump-tables -fno-tree-switch-conversion -MMD")
endif()

include($ENV{HOME}/esp/esp-idf/tools/cmake/project.cmake)
project(automatic-watering-system)
include_directories("main")
