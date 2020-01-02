#include "main.h"
#include "uart_dma.hpp"


hal::UartDma uart_dma(&huart3);

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
  /* This sample code shows how to use GPIO HAL API to toggle LED1 and LED2 IOs
    in an infinite loop. */

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();

  uart_dma.init();

  while (1) {
    uart_dma.loopback_test();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // HAL_UART_AbortTransmit_IT(huart);
  uart_dma.flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  uart_dma.reset_rbuf();
}