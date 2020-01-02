
set(CUBE_PREFIX /home/rossihwang/STM32Cube/Repository/STM32Cube_FW_F4_V1.24.1)
set(SOC_FAMILY STM32F4xx)

aux_source_directory(${CUBE_PREFIX}/Drivers/${SOC_FAMILY}_HAL_Driver/Src HAL_SOURCES)
### Exclude files
list(FILTER HAL_SOURCES
        EXCLUDE REGEX ".template.c"
        )

target_sources(${MCU_ELF} PUBLIC ${HAL_SOURCES})

target_include_directories(${MCU_ELF}
        PUBLIC ${CUBE_PREFIX}/Drivers/${SOC_FAMILY}_HAL_Driver/Inc
        PUBLIC ${CUBE_PREFIX}/Drivers/${SOC_FAMILY}_HAL_Driver/Inc/Legacy
        PUBLIC ${CUBE_PREFIX}/Drivers/CMSIS/Include
        PUBLIC ${CUBE_PREFIX}/Drivers/CMSIS/Device/ST/${SOC_FAMILY}/Include
        )