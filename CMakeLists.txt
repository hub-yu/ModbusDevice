# 设置 CMake 最低支持版本
cmake_minimum_required(VERSION 3.17)

# Cmake 交叉编译配置
set(CMAKE_SYSTEM_NAME Generic)

# 定义工程名称
project("demo")

# 指定编译工具
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")
set(CMAKE_AR "arm-none-eabi-ar")
set(CMAKE_OBJCOPY "arm-none-eabi-objcopy")
set(CMAKE_OBJDUMP "arm-none-eabi-objdump")
set(CMAKE_SIZE "arm-none-eabi-size")


if(${BOARD_TYPE} MATCHES "BOARD_SOCKET")
    add_definitions(-DUSE_STDPERIPH_DRIVER)

    option(BOOT "build for boot!" OFF)
    option(RELEASE "build for release!" OFF)
    option(FREERTOS "FreeRTOS support" ON)
    option(WIZNET_DRIVER "WizNET support" ON)

    set(DEVICE_TYPE  "STM32F103C8T6")
    # add_definitions(-DSTM32)
    add_definitions(-DSTM32F10X_MD)
endif()

if(${BOARD_TYPE} MATCHES "BOARD_DEVICE")
    add_definitions(-DUSE_STDPERIPH_DRIVER)
    option(BOOT "build for boot!" OFF)
    option(RELEASE "build for release!" OFF)
    option(FREERTOS "FreeRTOS support" ON)
    option(WIZNET_DRIVER "WizNET support" OFF)

    set(DEVICE_TYPE  "STM32F030F4P6")
    add_definitions(-DSTM32F030)

    # set(DEVICE_TYPE  "STM32F070F6P6")
    # add_definitions(-DSTM32F070x6)

endif()

if(${DEVICE_TYPE} MATCHES "STM32F030F4P6|STM32F070F6P6")
    # 编译相关选项
    set(MCU_FLAGS "-mcpu=cortex-m0 -mthumb -mfloat-abi=soft") 
    set(CMAKE_C_FLAGS_DEBUG "-g -ggdb -Og")
    set(CMAKE_C_FLAGS_RELEASE "-O3")
    # set(CMAKE_C_FLAGS "${MCU_FLAGS}  -ffunction-sections -fdata-sections -fno-builtin -fno-common -Wextra -Werror -Wno-unknown-pragmas -Wl,-u,_printf_float") #-w -Wall
    set(CMAKE_C_FLAGS "${MCU_FLAGS} -ffunction-sections -fdata-sections -fno-builtin -fno-common -Wall -Wno-unknown-pragmas")
    set(CMAKE_ASM_FLAGS "${MCU_FLAGS} -x assembler-with-cpp")
endif()

if(${DEVICE_TYPE} MATCHES "STM32F103C8T6")
    # 编译相关选项
    set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb -mfloat-abi=soft") 
    set(CMAKE_C_FLAGS_DEBUG "-g -ggdb -Og")
    set(CMAKE_C_FLAGS_RELEASE "-O3")
    # set(CMAKE_C_FLAGS "${MCU_FLAGS}  -ffunction-sections -fdata-sections -fno-builtin -fno-common -Wextra -Werror -Wno-unknown-pragmas -Wl,-u,_printf_float") #-w -Wall
    set(CMAKE_C_FLAGS "${MCU_FLAGS} -ffunction-sections -fdata-sections -fno-builtin -fno-common ") # -Wall -Wno-unknown-pragmas
    set(CMAKE_ASM_FLAGS "${MCU_FLAGS} -x assembler-with-cpp")
endif()

if(RELEASE)
    message("build for release!")
    set(CMAKE_BUILD_TYPE "Release")
    add_definitions(-DRELEASE)
else()
    message("build for debug!")
    set(CMAKE_BUILD_TYPE "Debug")
    add_definitions(-DDEBUG)
endif()


add_subdirectory(Platform) #平台库
add_subdirectory(Application) #应用


