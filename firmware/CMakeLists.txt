cmake_minimum_required(VERSION 3.22)

#---------------------------------------------------------------------------------------
# HWB-MC4G4 固件
#
# 构建方式：
#   cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
#   cmake --build build
#
# 依赖：arm-none-eabi-gcc 12.3+、STM32CubeG4 v1.6.0（HAL 驱动）
#---------------------------------------------------------------------------------------

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(TARGET hwb_mc4g4)

# ---- 源文件 -----------------------------------------------------------
set(APP_SOURCES
    Core/Src/main.c
    Core/Src/board.c
    Core/Src/param.c
    Core/Src/foc_math.c
    Core/Src/foc.c
    Core/Src/pid.c
    Core/Src/encoder.c
    Core/Src/current_sense.c
    Core/Src/motor.c
    Core/Src/pump.c
    Core/Src/cam_io.c
    Core/Src/can_proto.c
    Core/Src/can_node.c
    Core/Src/stm32g4xx_it.c
)

# STM32CubeG4 HAL（通过 CubeMX 生成的 Drivers 目录）
file(GLOB HAL_SOURCES
    Drivers/STM32G4xx_HAL_Driver/Src/*.c
)

set(CUBE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS)

add_executable(${TARGET}
    ${APP_SOURCES}
    ${HAL_SOURCES}
    Startup/startup_stm32g474vetx.s
)

target_include_directories(${TARGET} PRIVATE
    Core/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${CUBE_DIR}/Device/ST/STM32G4xx/Include
    ${CUBE_DIR}/Include
)

target_compile_definitions(${TARGET} PRIVATE
    USE_HAL_DRIVER
    STM32G474xx
    ARM_MATH_CM4
)

# ---- 编译选项 ---------------------------------------------------------
# -O2 而不是 -Ofast：浮点严格符合 IEEE754 对控制环是必须的，
# -ffast-math 会允许重排 (a*b)+c 之类的表达式，导致与仿真模型不一致。
target_compile_options(${TARGET} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -O2
    -g3
    -Wall
    -Wextra
    -Wdouble-promotion        # 抓隐式 double 提升（FPU 只支持单精度，提升会拖慢 10 倍）
    -ffunction-sections
    -fdata-sections
    -fno-common
)

target_link_options(${TARGET} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32G474VETx_FLASH.ld
    -Wl,--gc-sections
    -Wl,-Map=${TARGET}.map,--cref
    -specs=nano.specs
    -specs=nosys.specs
    -u_printf_float
)

# ---- 生成 hex/bin 与体积报告 ------------------------------------------
add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${TARGET}> ${TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>
    COMMENT "Generating hex/bin and size report"
)

# ---- 主机侧单元测试（把纯算法模块编成 PC 可执行文件） -------------------
# foc.c / pid.c / foc_math.c 不依赖 HAL（foc_math 的 CORDIC 有 libm 回退），
# 因此可以直接在 Ubuntu 上跑数值回归，与固件共用同一份源码。
option(BUILD_HOST_TESTS "Build PC-side algorithm tests" OFF)
if(BUILD_HOST_TESTS)
    add_executable(foc_host_test
        test/test_foc_regression.c
        Core/Src/foc.c
        Core/Src/pid.c
        Core/Src/foc_math.c
        Core/Src/can_proto.c
    )
    target_compile_definitions(foc_host_test PRIVATE HOST_BUILD=1)
    target_include_directories(foc_host_test PRIVATE Core/Inc test/stub)
    target_link_libraries(foc_host_test m)
endif()
