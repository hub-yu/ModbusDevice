
if(${DEVICE_TYPE} MATCHES "STM32F070F6P6|STM32F030F4P6")
    #添加头文件搜索路径
    include_directories(
        ${PROJECT_SOURCE_DIR}/Platform/CMSIS/Include
        ${PROJECT_SOURCE_DIR}/Platform/CMSIS/Device/ST/STM32F0xx/Include
        ${PROJECT_SOURCE_DIR}/Platform/STM32F0xx_StdPeriph_Driver/inc    
    )

    if(${FREERTOS})
        include_directories(
            ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel
            ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel/include
            ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel/portable/GCC/ARM_CM0
        )
    endif()
endif()

if(${DEVICE_TYPE} MATCHES "STM32F103C8T6")


    #添加头文件搜索路径
    include_directories(
        ${PROJECT_SOURCE_DIR}/Platform/CMSIS/Include
        ${PROJECT_SOURCE_DIR}/Platform/CMSIS/Device/ST/STM32F10x/Include
        ${PROJECT_SOURCE_DIR}/Platform/STM32F10x_StdPeriph_Driver/inc  
    )

    if(${FREERTOS})
        include_directories(
            ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel
            ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel/include
            ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel/portable/GCC/ARM_CM3
            # ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel/portable/GCC/ARM_CM3_MPU
        )
    endif()

endif()

if(${WIZNET_DRIVER})
    include_directories(
        ${PROJECT_SOURCE_DIR}/Platform/ioLibrary_Driver/Ethernet
        ${PROJECT_SOURCE_DIR}/Platform/ioLibrary_Driver/Ethernet/W5500
        ${PROJECT_SOURCE_DIR}/Platform/ioLibrary_Driver/Internet/DHCP
        ${PROJECT_SOURCE_DIR}/Platform/ioLibrary_Driver/Internet/DNS
    )
endif()


#设备
if(${BOARD_TYPE} MATCHES "BOARD_DEVICE") 
    add_definitions(-DBOARD_DEVICE)
    message("build for BOARD_DEVICE!")
    add_subdirectory(BoardDevice)
endif()

#网络

if(${BOARD_TYPE} MATCHES "BOARD_SOCKET") 
    add_definitions(-DBOARD_SOCKET)
    message("build for BOARD_SOCKET!")
    add_subdirectory(BoardSocket)
endif()

