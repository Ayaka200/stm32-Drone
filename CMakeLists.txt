cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Project name
set(CMAKE_PROJECT_NAME New_Drone)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: ${CMAKE_BUILD_TYPE}")

# Enable C and ASM
enable_language(C ASM)

# ===========================
# 编译优化选项
# ===========================

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(
            -mthumb
            -mcpu=cortex-m3
            -mfloat-abi=soft
            -fno-PIC

            -Og
            -g3
    )
else()
    add_compile_options(
            -mthumb
            -mcpu=cortex-m3
            -mfloat-abi=soft
            -fno-PIC

            -Os
            -ffunction-sections
            -fdata-sections
    )
endif()

# ===========================
# Executable
# ===========================

add_executable(${CMAKE_PROJECT_NAME}

        FreeRTOS/Src/croutine.c
        FreeRTOS/Src/event_groups.c
        FreeRTOS/Src/list.c
        FreeRTOS/Src/queue.c
        FreeRTOS/Src/stream_buffer.c
        FreeRTOS/Src/tasks.c
        FreeRTOS/Src/timers.c

        FreeRTOS/Portable/MemMang/heap_4.c
        FreeRTOS/Portable/GCC/ARM_CM3/port.c

        Core/Src/FreeRTOS_demo.c

        Application/InterFace/TP4336.c
        Application/InterFace/Motor.c
        Application/InterFace/LED.c
        Application/InterFace/nRF24L01P.c
        Application/InterFace/MPU6050.c

        Application/App_Receive_Data.c
        Application/App_Flight.c

        Common/Com_filter.c
        Common/Com_imu.c
        Common/Com_PID.c
        Common/Com_PID.h
        Application/InterFace/Battery.c
        Application/InterFace/Battery.h
)

# STM32CubeMX generated files
add_subdirectory(cmake/stm32cubemx)

# ===========================
# Include directories
# ===========================

include_directories(
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include

        ${CMAKE_SOURCE_DIR}/FreeRTOS/Inc
        ${CMAKE_SOURCE_DIR}/FreeRTOS/Portable/GCC/ARM_CM3

        ${CMAKE_SOURCE_DIR}/Application
        ${CMAKE_SOURCE_DIR}/Application/InterFace

        ${CMAKE_SOURCE_DIR}/Common
)

# ===========================
# Remove wrong dependency
# ===========================

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ===========================
# Link libraries
# ===========================

target_link_libraries(
        ${CMAKE_PROJECT_NAME}
        stm32cubemx
)

# ===========================
# Link options
# ===========================

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE

        # 生成 map 文件
        -Wl,-Map=${PROJECT_NAME}.map

        # 显示 Flash/RAM 使用率
        -Wl,--print-memory-usage

        # 删除未使用代码
        -Wl,--gc-sections

        # 支持 printf("%f")
        -u _printf_float
)