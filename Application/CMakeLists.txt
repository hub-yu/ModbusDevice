#添加头文件搜索路径
include_directories(
    ${PROJECT_SOURCE_DIR}/Platform/CMSIS/Include
    ${PROJECT_SOURCE_DIR}/Platform/CMSIS/Device/ST/STM32F0xx/Include
    ${PROJECT_SOURCE_DIR}/Platform/STM32F0xx_StdPeriph_Driver/inc 
    ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel
    ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel/include
    ${PROJECT_SOURCE_DIR}/Platform/FreeRTOS-Kernel/portable/GCC/ARM_CM0
    ${PROJECT_SOURCE_DIR}/Application
)


#裸跑
if(${BOARD_TYPE} MATCHES "BOARD_DEMO") 
    add_definitions(-DBOARD_DEMO)
    message("build for BOARD_DEMO!")
    add_subdirectory(BoardDemo)
endif()

#RTOS
if(${BOARD_TYPE} MATCHES "BOARD_RTOS") 
    add_definitions(-DBOARD_RTOS)
    message("build for BOARD_RTOS!")
    add_subdirectory(BoardRTOS)
endif()

#设备
if(${BOARD_TYPE} MATCHES "BOARD_DEVICE") 
    add_definitions(-DBOARD_DEVICE)
    message("build for BOARD_DEVICE!")
    add_subdirectory(BoardDevice)
endif()
