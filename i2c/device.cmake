
target_compile_definitions(${MCU_ELF}
        PRIVATE STM32F429xx
        PRIVATE USE_HAL_DRIVER
        PRIVATE USE_DEBUGGER
        )

target_compile_options(${MCU_ELF}
        PRIVATE -mthumb
        PRIVATE -mcpu=cortex-m4  # ‘cortex-m7’, ‘cortex-m4’, ‘cortex-m3’, ‘cortex-m1’, ‘cortex-m0’, ‘cortex-m0plus’
        # PRIVATE -mfloat-abi=hard   # for stm32f4 only
        PRIVATE -mfloat-abi=hard
        PRIVATE -mfpu=fpv4-sp-d16  # for stm32f4 only
        )

target_link_libraries(${MCU_ELF}
        PRIVATE -T${CUBE_PREFIX}/Projects/STM32F429ZI-Nucleo/Templates_LL/SW4STM32/NUCLEO-F429ZI/STM32F429ZITx_FLASH.ld
        PRIVATE -mthumb                        # Ensure link to the Thumb version libc
        PRIVATE -mcpu=cortex-m4
        PRIVATE -mfloat-abi=hard   # for stm32f4 only
        PRIVATE -mfpu=fpv4-sp-d16  # for stm32f4 only
        )
