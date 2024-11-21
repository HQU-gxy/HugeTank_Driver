cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME HugeTank)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM CXX)


# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

add_library(st7735 INTERFACE)

# Add STM32Cube sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/Src/main.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    # Add user defined libraries
    stm32cubemx
    st7735
)

target_include_directories(st7735 INTERFACE
    Drivers/st7735
)

target_sources(st7735 INTERFACE
    Drivers/st7735/st7735.c
    Drivers/st7735/st7735_reg.c
)



find_program(OOCD openocd)
if(OOCD)
        add_custom_target(
                flash-oocd COMMAND
                DEPENDS flash-oocd-cmd)
        add_custom_command(
                USES_TERMINAL
                OUTPUT flash-oocd-cmd
                COMMAND openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg -c 'program ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf verify reset exit'
                DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf)
endif()

