set(VERSION_MAJOR "2")
set(VERSION_MINOR "4")
set(VERSION_PATCH "2")

#APP
# v2.4.2 调整任务内存
# v2.4.1 修复flash溢出
# v2.4.0 增加浏览器接口复位，恢复出厂设置，状态查询
# v2.3.0 增加MQTT客户端支持
# v2.2.0 增加ASCII, 及网络MODBUS协议，
#        调整部分保持寄存器地址，参见readme.md
#        增加串口助手MODBUS ASCII配置文件
# v2.1.0 修改TCP 超时重连机制时间30秒->10秒
#        调整各个通道类型寄存器
#        减少通道数量7 -> 6
#        修改通道默认类型 
# v2.0.0 增加接收发送缓冲区大小64字节->256字节 最长MODBUS 单帧数据内容不超过64字节
#        增加DHCP 默认关
#        增加可独立配置的7通道socket,可自由配置协议，支持udp, dns, tcp server, tcp client 即最大支持7路UDP 或7路dns, 或7路tcp server, 或7路tcp client
#        增加TCP 超时重连机制 默认30秒 
#        增加串口波特率配置 支持115200(默认) 57600 19200 9600
#        增加软件复位、恢复出厂设置
#        增加输出掉电存储功能开关  默认关
#        增加串口日志输出功能  默认关
# v1.2.0 移除对输出的掉电存储
# v1.1.0 增加网口的TCP UDP通讯
# v1.0.0 首次添加



set(BOOT_VERSION_MAJOR "1")
set(BOOT_VERSION_MINOR "0")
set(BOOT_VERSION_PATCH "0")

#BOOT
# v1.0.0 首次添加

add_definitions(-DVERSION_MAJOR=${VERSION_MAJOR})
add_definitions(-DVERSION_MINOR=${VERSION_MINOR})
add_definitions(-DVERSION_PATCH=${VERSION_PATCH})
add_definitions(-DBOOT_VERSION_MAJOR=${BOOT_VERSION_MAJOR})
add_definitions(-DBOOT_VERSION_MINOR=${BOOT_VERSION_MINOR})
add_definitions(-DBOOT_VERSION_PATCH=${BOOT_VERSION_PATCH})

#添加头文件搜索路径
include_directories(
    ./Include
     ${PROJECT_SOURCE_DIR}/Application/Modbus
     ${PROJECT_SOURCE_DIR}/Application/Log
)

aux_source_directory(./Source SRC_LIST)

aux_source_directory(${PROJECT_SOURCE_DIR}/Application/Modbus/ MODBUS_LIST)
aux_source_directory(${PROJECT_SOURCE_DIR}/Application/Log/ LOG_LIST)

#添加非标准的共享库搜索路径
link_directories(${PROJECT_SOURCE_DIR}/build/lib)
link_directories(${PROJECT_SOURCE_DIR}/Platform/CMSIS/Lib/GCC)

if(${BOOT})
    set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/scripts/${DEVICE_TYPE}_BOOT.ld)
else()
    set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/scripts/${DEVICE_TYPE}.ld)
endif()

#设置链接选项
# set(CMAKE_EXE_LINKER_FLAGS " -specs=rdimon.specs --specs=nano.specs -specs=nosys.specs -T${LINKER_SCRIPT} -Wl,-Map=${PROJECT_BINARY_DIR}/${DEVICE_TYPE}.map,--cref -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS " -specs=rdimon.specs --specs=nano.specs -specs=nosys.specs -T${LINKER_SCRIPT} -Wl,-Map=${PROJECT_BINARY_DIR}/${DEVICE_TYPE}.map,--cref -Wl,--gc-sections")


#生成目标文件
add_executable(app.elf main.c ${SRC_LIST} ${INTERNET_LIST} ${MODBUS_LIST} ${LOG_LIST})

# 确保可执行文件依赖于动态库
add_dependencies(app.elf platform)

if(${BOOT})
    #把目标文件与库文件进行链接
    target_link_libraries(app.elf
        ${DEVICE_TYPE}

    )
else()
    #把目标文件与库文件进行链接
    target_link_libraries(app.elf
        ${DEVICE_TYPE}
    )
endif()

#设置可执行文件输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

#设置 ELF 转换路径
set(ELF_FILE ${PROJECT_BINARY_DIR}/app.elf)
set(HEX_FILE ${PROJECT_BINARY_DIR}/app.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/app.bin)



if(${BOOT})
    #添加自定义命令实现 ELF 转换 hex 和 bin 文件
    add_custom_command(TARGET "${DEVICE_TYPE}.elf" POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Oihex ${ELF_FILE} ${HEX_FILE}
        COMMENT "Building ${PROJECT_NAME}.bin and ${PROJECT_NAME}.hex"
        COMMAND ${CMAKE_COMMAND} -E copy ${HEX_FILE} "${PROJECT_BINARY_DIR}/bin/${PROJECT_NAME}_Boot_V${BOOT_VERSION_MAJOR}.${BOOT_VERSION_MINOR}.${BOOT_VERSION_PATCH}.hex"
        COMMAND ${CMAKE_COMMAND} -E copy ${BIN_FILE} "${PROJECT_BINARY_DIR}/bin/${PROJECT_NAME}_Boot_V${BOOT_VERSION_MAJOR}.${BOOT_VERSION_MINOR}.${BOOT_VERSION_PATCH}.bin"
        COMMAND ${CMAKE_COMMAND} -E copy ${BIN_FILE} "${PROJECT_BINARY_DIR}/bin/${PROJECT_NAME}.bin"
        COMMAND ${CMAKE_COMMAND} -E copy ${ELF_FILE} "${PROJECT_BINARY_DIR}/bin/${PROJECT_NAME}.elf"
        COMMAND ${CMAKE_SIZE} --format=berkeley ${ELF_FILE} ${HEX_FILE}
        COMMENT "Invoking: Cross ARM GNU Print Size")
else()
    #添加自定义命令实现 ELF 转换 hex 和 bin 文件
    add_custom_command(TARGET "app.elf" POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Oihex ${ELF_FILE} ${HEX_FILE}
        COMMENT "Building ${PROJECT_NAME}_${DEVICE_TYPE}.bin and ${PROJECT_NAME}_${DEVICE_TYPE}.hex"
        COMMAND ${CMAKE_COMMAND} -E copy ${HEX_FILE} "${PROJECT_BINARY_DIR}/bin/${BOARD_TYPE}_${DEVICE_TYPE}_V${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}.hex"
        COMMAND ${CMAKE_COMMAND} -E copy ${BIN_FILE} "${PROJECT_BINARY_DIR}/bin/${BOARD_TYPE}_${DEVICE_TYPE}_V${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}.bin"
        # COMMAND ${CMAKE_COMMAND} -E copy ${BIN_FILE} "${PROJECT_BINARY_DIR}/bin/${BOARD_TYPE}_${DEVICE_TYPE}.bin"
        # COMMAND ${CMAKE_COMMAND} -E copy ${ELF_FILE} "${PROJECT_BINARY_DIR}/bin/${BOARD_TYPE}_${DEVICE_TYPE}.elf"
        COMMAND ${CMAKE_SIZE} --format=berkeley ${ELF_FILE} ${HEX_FILE}
        COMMENT "Invoking: Cross ARM GNU Print Size")
endif()
