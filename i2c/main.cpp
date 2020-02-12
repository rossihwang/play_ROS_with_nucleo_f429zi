#include "generated/Inc/main.h"
#include "./sensor/mpu6050.hpp"
#include "./hal/uart_dma.hpp"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

extern I2C_HandleTypeDef hi2c1;

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
std_msgs::String chatter_msg;
ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher chatter("hello", &chatter_msg);

sensor::Mpu6050 imu(&hi2c1, 0x68); // 0x68: AD0->GND, 0x69: AD0->VCC
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
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();

  std::array<float, 6> self_test_ratio;
  // Self-test
  imu.SelfTest(self_test_ratio);
  if (1.0 < self_test_ratio[0] || 1.0 < self_test_ratio[1] || 1.0 < self_test_ratio[2] ||
      1.0 < self_test_ratio[3] || 1.0 < self_test_ratio[4] || 1.0 < self_test_ratio[5]) {
    return 1;
  }
  // Calibration
  imu.ComputeCalibrateParams();

  imu.Reset();
  if (!imu.CheckCommunication()) {
    return 1;
  }
  imu.Init();
  
  float gx, gy, gz;
  float ax, ay, az;
  float temp_dc;

  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(chatter);
  ros::Time now;

  while (1) {
    if (imu.IsDataReady()) {
      std::tie(gx, gy, gz) = imu.ReadGyroData(true);
      std::tie(ax, ay, az) = imu.ReadAccelData(true);
      temp_dc = imu.ReadTempData();
      
      imu_msg.angular_velocity.x = gx;
      imu_msg.angular_velocity.y = gy;
      imu_msg.angular_velocity.z = gz;
      imu_msg.linear_acceleration.x = ax;
      imu_msg.linear_acceleration.y = ay;
      imu_msg.linear_acceleration.z = az;
      now = nh.now();
      imu_msg.header.frame_id = "imu_frame";  // need for rviz
      imu_msg.header.stamp.sec = now.sec;  // need for rviz
      imu_msg.header.stamp.nsec = now.nsec;  // need for rviz
      imu_pub.publish(&imu_msg);

      chatter_msg.data = "hello";
      chatter.publish(&chatter_msg);
    }
    nh.spinOnce();
    HAL_Delay(200);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  nh.getHardware()->reset_rbuf();
}
