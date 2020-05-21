# play_ROS_with_nucleo_f429zi



> May 9 2020: Totally rewrite this project for ROS2



## Features

- MCU can communicate with host via message based on nanopb
- MCU can send logs to host, which is also based on nanopb
- MCU read data from IMU(MPU6050)
- TODO
  - Motor close loop controller



## MCU Pins

| Pin  | Function          | Description  |
| ---- | ----------------- | ------------ |
| PD8  | USART3_RX         |              |
| PD9  | USART3_TX         |              |
| PB14 | on-board LED(red) |              |
| PB8  | I2C1_SCL          | IMU          |
| PB9  | I2C1_SDA          |              |
| PA6  | TIM3_CH1          | Hall Sensor1 |
| PA7  | TIM3_CH2          | Hall Sensor1 |
| PD12 | TIM4_CH1          | Hall Sensor2 |
| PD13 | TIM4_CH2          | Hall Sensor2 |
| PC6  | TIM8_CH1          | Motor1       |
| PC7  | TIM8_CH2          | Motor1       |
| PC8  | TIM8_CH3          | Motor2       |
| PC9  | TIM8_CH4          | Motor2       |

