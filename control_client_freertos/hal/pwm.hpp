// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#include <array>

namespace hal {

class PwmInterface {
 private:
  TIM_HandleTypeDef *htim_;
  uint32_t period_;
  
 public:
  PwmInterface(TIM_HandleTypeDef *htim)
    : htim_(htim),
      period_(0) {
    // static_assert(htim_->Instance == TIM1 || htim_->Instance == TIM8);
    
  }
  void Init(float frequency) {  // hz

    constexpr float kPrescaler = 100;
    constexpr float kABP2TimerClock = 180000000;
    uint32_t period = static_cast<uint32_t>(kABP2TimerClock / kPrescaler / frequency);
    htim_->Init.Prescaler = kPrescaler - 1;
    htim_->Init.Period = period - 1;
    period_ = period;

    if (HAL_TIM_PWM_Init(htim_) != HAL_OK) {
      Error_Handler();
    }
  }
  void SetChannelDuty(const int channel, float duty) {
    static constexpr std::array<uint32_t, 4> kTimChannel = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

    uint8_t c = channel - 1;
    TIM_OC_InitTypeDef sConfigOC = {0};
    uint32_t pulse = 0;

    pulse = static_cast<uint32_t>(duty * period_);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(htim_, &sConfigOC, kTimChannel[c]);
    HAL_TIM_PWM_Start(htim_, kTimChannel[c]);
  }

};


}  // namespace hal