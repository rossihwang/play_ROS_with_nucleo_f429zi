// Copyright <2020> [Copyright rossihwang@gmail.com]
#pragma once 

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <tuple>

namespace hal {

class Encoder {
 private:
  TIM_HandleTypeDef *htim_;
  uint16_t last_count_;

 public:
  enum class Direction {
    kForward,
    kBackward
  };
  Encoder(TIM_HandleTypeDef *htim)
    : htim_(htim),
      last_count_(0) {
    assert_param(htim_->Instance == TIM3 || htim_->Instance== TIM4);
  }

  void Start() {
    if (HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL) != HAL_OK) {
      Error_Handler();
    }
  }
  void Stop() {
    if (HAL_TIM_Encoder_Stop(htim_, TIM_CHANNEL_ALL) != HAL_OK) {
      Error_Handler();
    }
  }
  std::tuple<Direction, uint16_t> Read() {
    // uint16_t temp = __HAL_TIM_GET_COUNTER(htim_);
    Direction dir;
    uint16_t count;

    count = __HAL_TIM_GET_COUNTER(htim_);
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim_)) {
      dir = Direction::kBackward;
    } else {
      dir = Direction::kForward;
    }
    last_count_ = count;
    return std::make_tuple(dir, count);
    
  }
};

}  // namespace hal