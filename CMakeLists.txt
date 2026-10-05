cmake_minimum_required(VERSION 3.22)
project(bike C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
option(BIKE_UART7_ECHO "Echo UART7 bytes for a physical-link diagnostic" OFF)

set(APP_SOURCES
    Src/balance_monitor.c Src/can.c Src/fonts.c Src/gpio.c Src/i2c.c Src/imu.c
    Src/imu_data_decode.c Src/key.c Src/main.c Src/odrive.c Src/oled.c
    Src/packet.c Src/servo.c Src/ssd1306.c Src/stm32f4xx_hal_msp.c
    Src/stm32f4xx_it.c Src/system_stm32f4xx.c Src/task.c Src/tim.c
    Src/upper.c Src/uart7_bridge.c Src/usart.c Src/zero_persistence.c
)

set(HAL_SOURCES
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c
)

add_executable(bike ${APP_SOURCES} ${HAL_SOURCES} startup_stm32f427xx_gcc.s)
set_target_properties(bike PROPERTIES SUFFIX ".elf")

target_include_directories(bike PRIVATE
    Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

target_compile_definitions(bike PRIVATE
    USE_HAL_DRIVER STM32F427xx USE_FULL_LL_DRIVER
    __packed=__attribute__\(\(packed\)\)
    $<$<BOOL:${BIKE_UART7_ECHO}>:BIKE_UART7_ECHO=1>
)

set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(bike PRIVATE
    ${CPU_FLAGS}
    "-D__align(x)=__attribute__((aligned(x)))"
    -ffunction-sections -fdata-sections -fno-common
    -Wall -Wextra -Wno-unused-parameter
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-O2;-g1>
)
target_link_options(bike PRIVATE
    ${CPU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32F427IIHx_FLASH.ld
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/bike.map
    -Wl,-u,_printf_float
)
target_link_libraries(bike PRIVATE m)

add_custom_command(TARGET bike POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:bike> ${CMAKE_CURRENT_BINARY_DIR}/bike.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:bike> ${CMAKE_CURRENT_BINARY_DIR}/bike.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:bike>
    COMMENT "Generating bike.hex and bike.bin"
)
