// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <module/pid.hpp>
#include <hal/pwm.hpp>
#include <hal/encoder.hpp>

namespace module {

constexpr float kCountFullPwmMps = 1300;
constexpr float kControlPeriod = 0.1;  // s

class MotorController {

 private:
  hal::PwmInterface pwm_;
  hal::Encoder right_encoder_;
  hal::Encoder left_encoder_;
  module::PIDController right_controller_;
  module::PIDController left_controller_;
  int32_t right_target_;
  int32_t left_target_;

 public:
  MotorController(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_encoder1, TIM_HandleTypeDef *htim_encoder2)
    : pwm_(htim_pwm),
      right_encoder_(htim_encoder1),
      left_encoder_(htim_encoder2),
      right_controller_(2, 5, 0, 0.01, kControlPeriod, kCountFullPwmMps),
      left_controller_(2, 5, 0, 0.01, kControlPeriod, kCountFullPwmMps),
      right_target_(0),
      left_target_(0) {
    
  }
  void init() {
    right_encoder_.Start();
    left_encoder_.Start();
    pwm_.Init(500);
    pwm_.SetChannelDuty(1, 0);
    pwm_.SetChannelDuty(2, 0);
    pwm_.SetChannelDuty(3, 0);
    pwm_.SetChannelDuty(4, 0);
  }
  void stop() {
    right_encoder_.Stop();
    left_encoder_.Stop();
    pwm_.SetChannelDuty(1, 0);
    pwm_.SetChannelDuty(2, 0);
    pwm_.SetChannelDuty(3, 0);
    pwm_.SetChannelDuty(4, 0);
  }
  std::tuple<int32_t, int32_t, int32_t, int32_t> update() {
    hal::Encoder::Direction right_dir, left_dir;
    int32_t right_counter, left_counter;
    float right_pwm_out, left_pwm_out;

    std::tie(right_dir, right_counter) = right_encoder_.ReadDiff();
    std::tie(left_dir, left_counter) = left_encoder_.ReadDiff();

    if (right_dir == hal::Encoder::Direction::kAntiClockwise) {
      right_counter = -right_counter;
    }
    if (left_dir == hal::Encoder::Direction::kClockwise) {
      left_counter = -left_counter;
    }

    if (right_target_ != 0) {
      right_pwm_out = right_controller_.Update(right_target_, right_counter);
    } else {
      right_pwm_out = 0;
    }
    if (left_target_ != 0) {
      left_pwm_out = left_controller_.Update(left_target_, left_counter);
    } else {
      left_pwm_out = 0;
    }

    if (right_pwm_out < 0) {
      pwm_.SetChannelDuty(1, 0);
      pwm_.SetChannelDuty(2, -right_pwm_out / kCountFullPwmMps);  // backward
    } else {
      pwm_.SetChannelDuty(1, right_pwm_out / kCountFullPwmMps);  // forward
      pwm_.SetChannelDuty(2, 0);
    }
    if (left_pwm_out < 0) {
      pwm_.SetChannelDuty(3, -left_pwm_out / kCountFullPwmMps);  // backward
      pwm_.SetChannelDuty(4, 0);
    } else {
      pwm_.SetChannelDuty(3, 0);
      pwm_.SetChannelDuty(4, left_pwm_out / kCountFullPwmMps);  // forward
    }
    return std::make_tuple(right_pwm_out, right_counter, left_pwm_out, left_counter);
  }
  void set(int32_t right_target, int32_t left_target) {
    right_target_ = right_target;
    left_target_ = left_target;
  }
};

}  // namespace module