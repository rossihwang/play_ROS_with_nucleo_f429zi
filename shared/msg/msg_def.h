// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include <unordered_map>
#include <msg/imu.pb.h>
#include <msg/twist.pb.h>
#include <msg/logging.pb.h>
#include <msg/pwm_ctrl.pb.h>
#include <msg/counter2.pb.h>

enum class MessageId {
  CNTR2 = 0,
  IMU,
  TWIST,
  PWM_CTRL,
  LOG
};

std::unordered_map<MessageId, const pb_msgdesc_t*> fields_map({
  {MessageId::CNTR2, Counter2_fields},
  {MessageId::IMU, Imu_fields},
  {MessageId::TWIST, Twist_fields},
  {MessageId::PWM_CTRL, PwmCtrl_fields},
  {MessageId::LOG, Log_fields},
});