// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"


namespace module {

class DiffDrive {

 private:
  
 public:
  DiffDrive(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_encoder)

};

}  // namespace module