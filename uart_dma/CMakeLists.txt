cmake_minimum_required(VERSION 3.12)
project(uart3_dma C CXX ASM) # set PROJECT_NAME

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_C_STANDARD 99)

# Add new target
set(MCU_ELF ${PROJECT_NAME}.elf)

add_executable(${MCU_ELF})

include(libcube.cmake)
include(device.cmake)
include(toolchain.cmake)

# Startup file
set(START_FILE ${CMAKE_CURRENT_SOURCE_DIR}/startup/startup_stm32f429xx.s)

# User code
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src USER_SOURCES)
target_sources(${MCU_ELF} PRIVATE ${USER_SOURCES})

target_include_directories(${MCU_ELF} PRIVATE ./inc)

# -specs=nosys.specs: syscall library
target_link_libraries(${MCU_ELF} PUBLIC ${START_FILE} -Wl,--gc-sections -specs=nosys.specs)

# Compilers flags settings
# "-ffunction-sections -fdata-sections, -Wl,--gc-sections usage"
# https://gcc.gnu.org/onlinedocs/gnat_ugn/Compilation-options.html
target_compile_options(${MCU_ELF}
        PUBLIC $<$<COMPILE_LANGUAGE:C>:-ffunction-sections -fdata-sections -Wall> # -fno-builtin -Wall -O0
        PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-ffunction-sections -fdata-sections -Wall -fno-exceptions -fno-rtti> # disable c++ exception and runtime type info
        )

# Convert elf file into intel hex/binary format(after elf file is build)
set(HEX_FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(ASM_FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.s)

add_custom_command(TARGET ${MCU_ELF} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex ${MCU_ELF} ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary ${MCU_ELF} ${BIN_FILE} # st-flash uses binary for default format
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}")
add_custom_command(TARGET ${MCU_ELF} POST_BUILD
        COMMAND ${CMAKE_OBJDUMP} -sfd ${MCU_ELF} > ${ASM_FILE}
        COMMENT "Disasembling ${ASM_FILE}")