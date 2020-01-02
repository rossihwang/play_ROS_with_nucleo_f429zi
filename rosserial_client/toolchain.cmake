set(TOOLCHAIN_PREFIX /home/rossihwang/zephyr_rtos/gcc-arm-none-eabi-8-2018-q4-major-linux/gcc-arm-none-eabi-8-2018-q4-major)

# Cmake cross compile settings
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Cross compile toolchain settings
message(STATUS "TOOLCHAIN_PREFIX: " ${TOOLCHAIN_PREFIX})
set(CROSS_COMPILER arm-none-eabi-)
message(STATUS "CROSS_COMPILER: " ${CROSS_COMPILER})

set(TOOLCHAIN_BIN_DIR ${TOOLCHAIN_PREFIX}/bin)

# Compilers settings
set(CMAKE_C_COMPILER ${TOOLCHAIN_BIN_DIR}/${CROSS_COMPILER}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN_DIR}/${CROSS_COMPILER}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN_DIR}/${CROSS_COMPILER}as)

# Other tools settings
set(CMAKE_OBJCOPY ${TOOLCHAIN_BIN_DIR}/${CROSS_COMPILER}objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_BIN_DIR}/${CROSS_COMPILER}objdump)

# Libc settings
# set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PREFIX})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)