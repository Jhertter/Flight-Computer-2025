# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(libraries/icm_20948/CMakeLists.txt)
include(libraries/MadgwickAHRS/CMakeLists.txt)
include(libraries/sam_m8q/CMakeLists.txt)
include(libraries/sam_m8q_v2/CMakeLists.txt)
include(libraries/bmp_280/CMakeLists.txt)


project(SRAD-Computer-code C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(SRAD-Computer-code SRAD-Computer-code.cpp)

pico_set_program_name(SRAD-Computer-code "SRAD-Computer-code")
pico_set_program_version(SRAD-Computer-code "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(SRAD-Computer-code 0)
pico_enable_stdio_usb(SRAD-Computer-code 1)

# Add the standard library to the build
target_link_libraries(SRAD-Computer-code
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(SRAD-Computer-code PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# add_library(MadgwickAHRS libraries/pico-icm20948/MadgwickAHRS/MadgwickAHRS.c)
# add_library(pico-icm20948 libraries/pico-icm20948/src/pico-icm20948.c)

# Add any user requested libraries
target_link_libraries(SRAD-Computer-code 
        hardware_spi
        hardware_i2c
        pico-icm20948
        MadgwickAHRS
        sam_m8q
        sam_m8q_v2
        bmp280
        )

target_include_directories(SRAD-Computer-code PRIVATE
)

pico_add_extra_outputs(SRAD-Computer-code)