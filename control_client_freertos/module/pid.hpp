// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

namespace module {

class PIDController {
 private:
  float kp_;
  float ki_;
  float kd_;

  float tau_;

  float lim_min_;
  float lim_max_;

  float T_;

  float integrator_;
  float prev_error_;
  float differentiator_;
  float prev_measurement_;

 public:
  PIDController(float kp, float ki, float kd, float tau, float T, float lim_max) 
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      tau_(tau),
      lim_min_(-lim_max),
      lim_max_(lim_max),
      T_(T),
      integrator_(0),
      prev_error_(0),
      differentiator_(0),
      prev_measurement_(0) {

  }
  float Update(float setpoint, float measurement) {
    // Error signal
    float error = setpoint - measurement;

    // Proportional
    float proportional = kp_ * error;

    // Integral
    integrator_ = integrator_ + 0.5 * ki_ * T_ * (error + prev_error_);

    // Anti-wind-up via dynamic integrator clamping
    float lim_min_init, lim_max_init;
    // Compute integrator limits
    if (proportional < lim_max_) {
      lim_max_init = lim_max_ - proportional;
    } else {
      lim_max_init = 0;
    }
    if (lim_min_ < proportional) {
      lim_min_init = lim_min_ - proportional; 
    } else {
      lim_min_init = 0;
    }

    // Clamp integrator
    if (lim_max_init < integrator_) {
      integrator_ = lim_max_init;
    } else if (integrator_ < lim_min_init) {
      integrator_ = lim_min_init;
    }

    differentiator_ = -(2 * kd_ * (measurement - prev_measurement_)
                      + (2 * tau_ - T_) * differentiator_)
                      / (2 * tau_ + T_);
    float out;
    out = proportional + integrator_ + differentiator_;
    if (lim_max_ < out) {
      out = lim_max_;
    } else if (out < lim_min_) {
      out = lim_min_;
    }

    prev_error_ = error;
    prev_measurement_ = measurement;

    return out;
  }
};

}  // namespace module