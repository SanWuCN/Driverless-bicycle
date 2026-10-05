set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(_arm_toolchain_hints
    "$ENV{HOME}/Library/ArmGNUToolchain/15.3.rel1/bin"
    "/Applications/ArmGNUToolchain/15.3.rel1/arm-none-eabi/bin"
)

find_program(ARM_NONE_EABI_GCC arm-none-eabi-gcc HINTS ${_arm_toolchain_hints})
if(NOT ARM_NONE_EABI_GCC)
    message(FATAL_ERROR
        "arm-none-eabi-gcc was not found. Install Arm GNU Toolchain or add its bin directory to PATH.")
endif()

get_filename_component(ARM_NONE_EABI_BIN_DIR "${ARM_NONE_EABI_GCC}" DIRECTORY)
set(CMAKE_C_COMPILER "${ARM_NONE_EABI_GCC}")
set(CMAKE_ASM_COMPILER "${ARM_NONE_EABI_GCC}")
set(CMAKE_OBJCOPY "${ARM_NONE_EABI_BIN_DIR}/arm-none-eabi-objcopy" CACHE FILEPATH "")
set(CMAKE_SIZE "${ARM_NONE_EABI_BIN_DIR}/arm-none-eabi-size" CACHE FILEPATH "")
set(CMAKE_EXECUTABLE_SUFFIX ".elf")
