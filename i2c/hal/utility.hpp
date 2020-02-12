// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include "stm32f4xx_hal_i2c.h"

namespace hal {

namespace utility {

  void DelayMs(int n_ms) {
    HAL_Delay(n_ms);
  }


}  // namespace utility

}  // namespace hal 