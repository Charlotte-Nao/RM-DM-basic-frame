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
set(CMAKE_PROJECT_NAME DM)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

set(MCU_FLAGS
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard
)

add_compile_options(${MCU_FLAGS})
add_link_options(${MCU_FLAGS})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        bsp/LED/LED.c
        bsp/LED/LED.h
        task/test_task/test_task.c
        task/test_task/test_task.h
        task/all_task.h
        bsp/uart/uart.c
        bsp/uart/uart.h
        bsp/can/can.c
        bsp/can/can.h
        device/motor/motor.c
        device/motor/motor.h
        dsp/pid/pid.c
        dsp/pid/pid.h
        dsp/MahonyAHRS/MahonyAHRS.c
        device/BMI088/BMI088driver.c
        device/BMI088/BMI088Middleware.c
        task/sensor_task/sensor_task.c
        task/sensor_task/sensor_task.h
        application/global_data.c
        application/global_data.h
        task/printf_task/printf_task.c
        task/printf_task/printf_task.h
        device/YB_SD15M/YB_SD15M.c
        device/YB_SD15M/YB_SD15M.h
        task/servo_task/servo_task.c
        task/servo_task/servo_task.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m
    # Add user defined libraries
)

# Enable float formatting for printf / snprintf / vsnprintf
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -T${CMAKE_SOURCE_DIR}/STM32H723xG_flash.ld

        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_PROJECT_NAME}.map
        -Wl,--cref

        -Wl,-u,_printf_float
)
