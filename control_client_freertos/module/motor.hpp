// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include <module/pid.hpp>
#include <hal/pwm.hpp>
#include <hal/encoder.hpp>

namespace module {

constexpr float kMaxVelocity = 0.2;
constexpr float kCountsMps = 7644;  // 
constexpr float kCountFullPwmMps = 1300;  // 13000
constexpr float kControlPeriod = 0.1;  // s

struct linear {
  float x;
  float y;
  float z;
  linear (float x = 0, float y = 0, float z = 0)
    : x(x),
      y(y),
      z(z) {
  }
};

struct angular {
  float x;
  float y;
  float z;
  angular (float x = 0, float y = 0, float z = 0)
    : x(x),
      y(y),
      z(z) {
  }
};

class DiffDrive {

 private:
  hal::PwmInterface pwm_;
  hal::Encoder encoder1_;  // right
  hal::Encoder encoder2_;  // left
  module::PIDController left_controller_;
  module::PIDController right_controller_;
  int32_t left_target_;
  int32_t right_target_;
  
 public:
  DiffDrive(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_encoder1, TIM_HandleTypeDef *htim_encoder2)
    : pwm_(htim_pwm),
      encoder1_(htim_encoder1),
      encoder2_(htim_encoder2),
      left_controller_(2, 5, 0, 0.01, kControlPeriod, kCountFullPwmMps),
      right_controller_(2, 5, 0, 0.01, kControlPeriod, kCountFullPwmMps),
      left_target_(0),
      right_target_(0) {
    
  }
  void Init() {
    encoder1_.Start();
    encoder2_.Start();
    pwm_.Init(500);
    pwm_.SetChannelDuty(1, 0);
    pwm_.SetChannelDuty(2, 0);
    pwm_.SetChannelDuty(3, 0);
    pwm_.SetChannelDuty(4, 0);
  }
  void Stop() {
    encoder1_.Stop();
    encoder2_.Stop();
    pwm_.SetChannelDuty(1, 0);
    pwm_.SetChannelDuty(2, 0);
    pwm_.SetChannelDuty(3, 0);
    pwm_.SetChannelDuty(4, 0);
  }
  std::tuple<int32_t, int32_t, int32_t> Update() {
    hal::Encoder::Direction rdirect, ldirect;
    int32_t rcount, lcount;
    float left_out, right_out;


    // if (left_target_ == 0 && right_target_ == 0) {
    //   Stop();
    // }

    std::tie(ldirect, lcount) = encoder2_.ReadDiff();
    std::tie(rdirect, rcount) = encoder1_.ReadDiff();

    if (ldirect == hal::Encoder::Direction::kClockwise) {
      lcount = -lcount;
    }
    if (rdirect == hal::Encoder::Direction::kAntiClockwise) {
      rcount = -rcount;
    }

    if (left_target_ != 0) {
      left_out = left_controller_.Update(left_target_, lcount);
    } else {
      left_out = 0;
    }
    
    if (right_target_ != 0) {
      right_out = right_controller_.Update(right_target_, rcount);
    } else {
      right_out = 0;
    }
    

    // pwm channel 1&2 used for right wheel, while channel 3&4 used for left wheel
    // if (is_forward_) {
    //   pwm_.SetChannelDuty(1, right_out / kCountFullPwmMps);
    //   pwm_.SetChannelDuty(2, 0);
    //   pwm_.SetChannelDuty(3, 0);
    //   pwm_.SetChannelDuty(4, left_out / kCountFullPwmMps);
    // } else {
    //   pwm_.SetChannelDuty(1, 0);
    //   pwm_.SetChannelDuty(2, right_out / kCountFullPwmMps);
    //   pwm_.SetChannelDuty(3, left_out / kCountFullPwmMps);
    //   pwm_.SetChannelDuty(4, 0);
    // }

    if (right_out < 0) {
      pwm_.SetChannelDuty(1, 0);
      pwm_.SetChannelDuty(2, -right_out / kCountFullPwmMps);
    } else {
      pwm_.SetChannelDuty(1, right_out / kCountFullPwmMps);
      pwm_.SetChannelDuty(2, 0);
    }

    if (left_out < 0) {
      pwm_.SetChannelDuty(3, -left_out / kCountFullPwmMps);
      pwm_.SetChannelDuty(4, 0);
    } else {
      pwm_.SetChannelDuty(3, 0);
      pwm_.SetChannelDuty(4, left_out / kCountFullPwmMps);
    }
    
    return std::make_tuple(right_target_, static_cast<int32_t>(right_out), rcount);
  }
  void Set(const linear &v, const angular &w) {
    left_target_ = (v.x * kCountsMps) * kControlPeriod;
    right_target_ = (v.x * kCountsMps) * kControlPeriod;
  }
};

}  // namespace module