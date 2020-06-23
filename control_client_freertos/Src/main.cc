/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <hal/uart_dma.hpp>
#include <hal/pwm.hpp>
#include <hal/encoder.hpp>
#include <msg/msg_def.h>
#include <pigeon/pigeon.h>
#include <module/mpu6050.hpp>
#include <module/motor.hpp>

#include <mx_export.h>
#include <sstream>

using pigeon::Pigeon;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

Pigeon pg;
hal::UartDmaInterface uart(&huart3);
module::Mpu6050 imu(&hi2c1, 0x68);  // 0x68: AD0->GND, 0x69: AD0->VCC
hal::PwmInterface pwm(&htim8);
module::DiffDrive driver(&htim8, &htim3, &htim4);

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal
  
};
/* Definitions for debugLedTask */
osThreadId_t imuReadTaskHandle;
const osThreadAttr_t imuReadTask_attributes = {
  .name = "debugLedTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow
};

osThreadId_t uartRxTaskHandle;
const osThreadAttr_t uartRxTask_attributes = {
  .name = "uartRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh
};

osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal
};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void *argument);
void ImuReadTaskHandle(void *argument);
void UartRxTaskHandle(void *argument);
void ControlTaskHandle(void *argument);

void callback_twist(const uint8_t *data, uint16_t length);
void callback_pwm_ctrl(const uint8_t *data, uint16_t length);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();

  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  uart.init();
  pwm.Init(500);
  pwm.SetChannelDuty(1, 0.0);
  pwm.SetChannelDuty(2, 0.0);
  pwm.SetChannelDuty(3, 0.0);
  pwm.SetChannelDuty(4, 0.0);

  pg.create_subscriber(MessageId::TWIST, callback_twist);
  pg.create_subscriber(MessageId::PWM_CTRL, callback_pwm_ctrl);
  using std::placeholders::_1;
  using std::placeholders::_2;
  pg.register_read(std::bind(static_cast<size_t (hal::UartDmaInterface::*)(uint8_t*, size_t)>(&hal::UartDmaInterface::read), &uart, _1, _2));
  pg.register_write(std::bind(static_cast<void (hal::UartDmaInterface::*)(const uint8_t*, size_t)>(&hal::UartDmaInterface::write), &uart, _1, _2));
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  // imuReadTaskHandle = osThreadNew(ImuReadTaskHandle, NULL, &imuReadTask_attributes);

  uartRxTaskHandle = osThreadNew(UartRxTaskHandle, NULL, &uartRxTask_attributes);

  controlTaskHandle = osThreadNew(ControlTaskHandle, NULL, &controlTask_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void callback_twist(const uint8_t *data, uint16_t length) {
  Twist message = Twist_init_zero;
  bool status;
  pb_istream_t stream = pb_istream_from_buffer(data, length);
  status = pb_decode(&stream, Twist_fields, &message);
  if (status) {
    module::linear lv;
    module::angular av;

    if (message.has_angular) {
      av.x = message.angular.x;
      av.y = message.angular.y;
      av.z = message.angular.z;
    }
    if (message.has_linear) {
      lv.x = message.linear.x;
      lv.y = message.linear.y;
      lv.z = message.linear.z;
    }
    taskENTER_CRITICAL();
    driver.Set(lv, av);
    taskEXIT_CRITICAL();
  }
  return;
}

void callback_pwm_ctrl(const uint8_t *data, uint16_t length) {
  PwmCtrl message = PwmCtrl_init_zero;
  bool status;
  pb_istream_t stream = pb_istream_from_buffer(data, length);
  status = pb_decode(&stream, PwmCtrl_fields, &message);
  if (status) {
    pwm.SetChannelDuty(message.channel, message.duty);
  }
  
  return;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  uart.flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  uart.reset_rbuf();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
 RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void StartDefaultTask(void *argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // For debug
    taskENTER_CRITICAL();
    pg.log<Log_Level_DEBUG>("led toggle");
    taskEXIT_CRITICAL();
    osDelay(500);
  }
}

void ImuReadTaskHandle(void *argument) {

  imu.ComputeCalibrateParams();
  imu.Reset();

  if (!imu.CheckCommunication()) {
    return;
  }

  imu.Init();
  Imu message = Imu_init_zero;
  float gx, gy, gz;
  float ax, ay, az;

  for(;;)
  { 
    std::tie(gx, gy, gz) = imu.ReadGyroData(true);
    std::tie(ax, ay, az) = imu.ReadAccelData(true);

    message.angular_velocity.x = gx;
    message.angular_velocity.y = gy;
    message.angular_velocity.z = gz;
    message.linear_acceleration.x = ax;
    message.linear_acceleration.y = ay;
    message.linear_acceleration.z = az;
    message.has_angular_velocity = true;
    message.has_linear_acceleration = true;

    taskENTER_CRITICAL();
    pg.publish<Imu>(MessageId::IMU, message);
    taskEXIT_CRITICAL();

    osDelay(1000);  // 1s
  }
}

void UartRxTaskHandle(void *argument) {

  for (;;) {
    pg.poll();
    osDelay(10);
  }
}

void ControlTaskHandle(void *argument) {
  driver.Init();
  uint16_t target, set, current;
  static char sb[50];

  for (;;) {
    std::tie(target, set, current) = driver.Update();
    snprintf(sb, 50, "target: %d, set: %d, current: %d", target, set, current);
    taskENTER_CRITICAL();
    pg.log<Log_Level_DEBUG>(std::string(sb));
    taskEXIT_CRITICAL();

    osDelay(100);  // 100ms
  }
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // For debug
    osDelay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
