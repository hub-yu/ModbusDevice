set(VERSION_MAJOR "1")
set(VERSION_MINOR "1")
set(VERSION_PATCH "0")

#APP
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
