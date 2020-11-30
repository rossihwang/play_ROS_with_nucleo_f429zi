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

class DcMotor {

 private:
  hal::PwmInterface pwm_;
  hal::Encoder encoder_;
  module::PIDController controller_;
  int32_t target_;
  int pwm_ch1_;
  int pwm_ch2_;
  hal::Encoder::Direction ref_direction_;
  int32_t last_counter_;
  bool inverted_;

 public:
  DcMotor(TIM_HandleTypeDef *pwm_timer, TIM_HandleTypeDef *encoder_timer,  int pwm_ch1, int pwm_ch2, bool inverted = false) 
    : pwm_(pwm_timer),
      encoder_(encoder_timer),
      controller_(2, 5, 0, 0.01, kControlPeriod, kCountFullPwmMps),
      target_(0),
      pwm_ch1_(pwm_ch1),
      pwm_ch2_(pwm_ch2),
      last_counter_(0),
      inverted_(inverted) {
  }
  void init() {
    encoder_.Start();
    pwm_.Init(500);
    pwm_.SetChannelDuty(pwm_ch1_, 0);
    pwm_.SetChannelDuty(pwm_ch2_, 0);
  }

  void stop () {
    encoder_.Stop();
    pwm_.SetChannelDuty(pwm_ch1_, 0);
    pwm_.SetChannelDuty(pwm_ch2_, 0);
  }
  std::tuple<int32_t, int32_t> update() {
    hal::Encoder::Direction dir;
    int32_t counter;
    float pwm_out;

    std::tie(dir, counter) = encoder_.ReadDiff();
    // counter is unsigned after readout
    if (dir == hal::Encoder::Direction::kAntiClockwise) {
      counter = -counter;
    }

    // FIXME: kind of overflow problem in the ReadDiff???
    // currently just ignore the invalid counter value and return the last value
    if (1.5 * kCountFullPwmMps < std::abs(counter)) {
      counter = last_counter_;
    }
    if (target_ != 0) {
      pwm_out = controller_.Update(target_, counter);
    } else {
      pwm_out = 0;
    }

    if (pwm_out < 0) {
      pwm_.SetChannelDuty(pwm_ch1_, 0);
      pwm_.SetChannelDuty(pwm_ch2_, -pwm_out / kCountFullPwmMps);  // backward
    } else if (0 < pwm_out) {
      pwm_.SetChannelDuty(pwm_ch1_, pwm_out / kCountFullPwmMps);  // forward
      pwm_.SetChannelDuty(pwm_ch2_, 0);
    } else {
      pwm_.SetChannelDuty(pwm_ch1_, 0);
      pwm_.SetChannelDuty(pwm_ch2_, 0);
    }
    
    last_counter_ = counter;
    return std::make_tuple(pwm_out, counter); 
  }

  void set(int32_t target) {
    if (inverted_) {
      target_ = -target;
    } else {
      target_ = target;
    }
  }
};
}  // namespace module