cmake_minimum_required(VERSION 3.10)
project(test C CXX ASM) # set PROJECT_NAME

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

# Add new target
set(MCU_ELF ${PROJECT_NAME}.elf)

# Driver
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src DRIVER_SOURCES)
list(FILTER DRIVER_SOURCES
     EXCLUDE REGEX ".template.c")
set(DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
                        Drivers/STM32F4xx_HAL_Driver/Inc/
                        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
                        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Core/Include)

# Startup file
set(START_FILE startup_stm32f429xx.s)
set_source_files_properties(${START_FILE} PROPERTY LANGUAGE ASM)

# Thirdparty
aux_source_directory(../Thirdparty/nanopb/src NANOPB_SOURCES)
set(NANOPB_INCLUDE_DIRS ../Thirdparty/nanopb/include)

# Message
aux_source_directory(../shared/msg/ MSG_SOURCES)
set(MSG_INCLUDE_DIRS ../shared ../shared/msg/)

# FreeRTOS
aux_source_directory(Middlewares/Third_Party/FreeRTOS/Source RTOS_SOURCES)
list(APPEND RTOS_SOURCES Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
                         Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
                         Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c)
set(RTOS_INCLUDE_DIRS Middlewares/Third_Party/FreeRTOS/Source/include
                      Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
                      Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2)


# User code
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Src  USER_SOURCES)
list(REMOVE_ITEM USER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.c)
set(USER_INCLUDE_DIRS .)

add_executable(${MCU_ELF} ${DRIVER_SOURCES} 
                          ${USER_SOURCES} 
                          ${THIRDPARTY_SOURCES} 
                          ${RTOS_SOURCES} 
                          ${START_FILE} 
                          ${NANOPB_SOURCES} 
                          ${MSG_SOURCES})
# include(libcube.cmake)
include(device.cmake)
include(toolchain.cmake)

target_include_directories(${MCU_ELF} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Inc
                                              ${NANOPB_INCLUDE_DIRS}
                                              ${MSG_INCLUDE_DIRS}
                                              ${DRIVER_INCLUDE_DIRS}
                                              ${RTOS_INCLUDE_DIRS}
                                              ${USER_INCLUDE_DIRS})

# -specs=nosys.specs: syscall library
target_link_libraries(${MCU_ELF} PUBLIC -Wl,--gc-sections -specs=nosys.specs -lc -lm )  # -specs=nano.specs -lnosys

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