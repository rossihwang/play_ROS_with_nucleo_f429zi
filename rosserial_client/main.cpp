// Copyright <2020> [Copyright rossihwang@gmail.com

#include "./generated/Inc/main.h"
#include "./hal/uart_dma.hpp"

#include <ros.h>
#include <std_msgs/String.h>


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;


hal::UartDma uart_dma(&huart3);

ros::NodeHandle nh;
std_msgs::String str_msg1;
std_msgs::String str_msg2;

ros::Publisher chatter1("chatter1", &str_msg1);
ros::Publisher chatter2("chatter2", &str_msg2);
char hello[] = "Hello world";
char good[] = "Good morning";

void setup();
void loop();

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
  
  setup();

  while (1) {
    loop();
  }
}

void setup() {
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();

  nh.initNode();
  nh.advertise(chatter1);
  nh.advertise(chatter2);
}

void loop() {
  str_msg1.data = hello;
  str_msg2.data = good;
  chatter1.publish(&str_msg1);
  chatter2.publish(&str_msg2);
  nh.spinOnce();

  HAL_Delay(1000);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  nh.getHardware()->reset_rbuf();
}