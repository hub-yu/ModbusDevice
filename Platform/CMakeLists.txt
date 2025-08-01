
#添加标准库源文件
if(${DEVICE_TYPE} MATCHES "STM32F070F6P6|STM32F030F4P6")

    #添加头文件搜索路径
    include_directories(
        ./CMSIS/Include
        ./CMSIS/Device/ST/STM32F0xx/Include
        ./STM32F0xx_StdPeriph_Driver/inc
    )

    #添加源文件
    aux_source_directory(./CMSIS/Device/ST/STM32F0xx/Source SRC_LIST)
    aux_source_directory(./STM32F0xx_StdPeriph_Driver/src STD_LIST)

    if(${FREERTOS})

        include_directories(
            ./FreeRTOS-Kernel
            ./FreeRTOS-Kernel/include
            ./FreeRTOS-Kernel/portable/GCC/ARM_CM0
        )

        file(GLOB FREERTOS_LIST
            ./FreeRTOS-Kernel/portable/GCC/ARM_CM0/mpu_wrappers_v2_asm.c
            ./FreeRTOS-Kernel/portable/GCC/ARM_CM0/portasm.c
            ./FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
            ./FreeRTOS-Kernel/portable/MemMang/heap_4.c
            # ./FreeRTOS-Kernel/portable/MemMang/heap_1.c
            ./FreeRTOS-Kernel/croutine.c
            ./FreeRTOS-Kernel/event_groups.c
            ./FreeRTOS-Kernel/list.c
            ./FreeRTOS-Kernel/queue.c
            ./FreeRTOS-Kernel/stream_buffer.c
            ./FreeRTOS-Kernel/tasks.c
            ./FreeRTOS-Kernel/timers.c
        )
    endif()

endif()

if(${DEVICE_TYPE} MATCHES "STM32F103C8T6")

    #添加头文件搜索路径
    include_directories(
        ./CMSIS/Include
        ./CMSIS/Device/ST/STM32F10x/Include
        ./STM32F10x_StdPeriph_Driver/inc
    )

    #添加源文件
    aux_source_directory(./CMSIS/Device/ST/STM32F10x/Source SRC_LIST)
    aux_source_directory(./STM32F10x_StdPeriph_Driver/src STD_LIST)

    if(${FREERTOS})

        include_directories(
            ./FreeRTOS-Kernel
            ./FreeRTOS-Kernel/include
            # ./FreeRTOS-Kernel/portable/GCC/ARM_CM3_MPU
            ./FreeRTOS-Kernel/portable/GCC/ARM_CM3
        )

        file(GLOB FREERTOS_LIST
            # ./FreeRTOS-Kernel/portable/GCC/ARM_CM3_MPU/mpu_wrappers_v2_asm.c
            # ./FreeRTOS-Kernel/portable/GCC/ARM_CM3_MPU/port.c
            ./FreeRTOS-Kernel/portable/GCC/ARM_CM3/port.c
            ./FreeRTOS-Kernel/portable/MemMang/heap_4.c
            # ./FreeRTOS-Kernel/portable/MemMang/heap_1.c
            ./FreeRTOS-Kernel/croutine.c
            ./FreeRTOS-Kernel/event_groups.c
            ./FreeRTOS-Kernel/list.c
            ./FreeRTOS-Kernel/queue.c
            ./FreeRTOS-Kernel/stream_buffer.c
            ./FreeRTOS-Kernel/tasks.c
            ./FreeRTOS-Kernel/timers.c
        )
    endif()

endif()



#设置启动文件变量
if(${DEVICE_TYPE} MATCHES "STM32F070F6P6")
    set(START_UP_ASM ./CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f070x6.s)
endif()

if(${DEVICE_TYPE} MATCHES "STM32F030F4P6")
    set(START_UP_ASM ./CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f030x6.s)
endif()

if(${DEVICE_TYPE} MATCHES "STM32F103C8T6")
    set(START_UP_ASM ./CMSIS/Device/ST/STM32F10x/Source/Templates/gcc_ride7/startup_stm32f10x_md.s)
endif()

#设置支持 ASM
enable_language(ASM)

#设置启动文件 C 属性
set_property(SOURCE ${START_UP_ASM} PROPERTY LANGUAGE C)

#生成库目标 platform
add_library(platform STATIC
    ${START_UP_ASM}
    ${SRC_LIST}
    ${STD_LIST}
    ${FREERTOS_LIST}
)

#设置库输出的名称
set_target_properties(platform PROPERTIES OUTPUT_NAME ${DEVICE_TYPE})

#设置库文件的默认输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib)
