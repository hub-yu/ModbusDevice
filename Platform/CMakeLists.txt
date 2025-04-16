#添加头文件搜索路径
include_directories(
    ./CMSIS/Include
    ./CMSIS/Device/ST/STM32F0xx/Include
    ./STM32F0xx_StdPeriph_Driver/inc
    ./FreeRTOS-Kernel
    ./FreeRTOS-Kernel/include
    ./FreeRTOS-Kernel/portable/GCC/ARM_CM0
)

#添加源文件
aux_source_directory(./CMSIS/Device/ST/STM32F0xx/Source/Templates SRC_LIST)

#添加标准库源文件
# if(${DEVICE_TYPE} MATCHES "STM32F070F6P6")

    file(GLOB STD_LIST
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_adc.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_can.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_cec.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_comp.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_crc.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_crs.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_dac.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_dbgmcu.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_dma.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_exti.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_flash.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_gpio.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_i2c.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_iwdg.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_misc.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_pwr.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_rcc.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_rtc.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_spi.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_syscfg.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_tim.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_usart.c
        ./STM32F0xx_StdPeriph_Driver/src/stm32f0xx_wwdg.c

    )
# endif()

if(${FREERTOS})
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

#设置启动文件变量
if(${DEVICE_TYPE} MATCHES "STM32F070F6P6")
    set(START_UP_ASM ./CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f070x6.s)
endif()

if(${DEVICE_TYPE} MATCHES "STM32F030F4P6")
    set(START_UP_ASM ./CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f030x6.s)
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
