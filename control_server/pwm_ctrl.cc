// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <string>
#include <iostream>
#include <cstdio>
#include <thread>
#include <unistd.h>
#include <serial/serial.h>
#include <pigeon/pigeon.h>
#include <msg/msg_def.h>

using std::string;
using std::exception;
using pigeon::Pigeon;

Pigeon pg;

void print_usage() {
  std::cerr << "Usage: pwm_ctrl_test <dev> <baud_rate> <channel(int:1-2)> <duty(float:0-1)>";
}

int main(int argc, char **argv) {
  if (argc < 5) {
    print_usage();
    return 1;
  }

  serial::Serial my_serial(argv[1], atoi(argv[2]), serial::Timeout::simpleTimeout(3000));
  
  using std::placeholders::_1;
  using std::placeholders::_2;
  pg.register_read(std::bind(static_cast<size_t (serial::Serial::*)(uint8_t*, size_t)>(&serial::Serial::read), &my_serial, _1, _2));
  pg.register_write(std::bind(static_cast<size_t (serial::Serial::*)(const uint8_t*, size_t)>(&serial::Serial::write), &my_serial, _1, _2));
  
  std::cout << "Is the serial port open?";
  if (my_serial.isOpen()) {
    std::cout << " Yes." << std::endl;
  } else {
    std::cout << " No." << std::endl;
  }

  PwmCtrl message = PwmCtrl_init_zero;
  message.channel = atoi(argv[3]);
  message.duty = atof(argv[4]);
  pg.publish(MessageId::PWM_CTRL, message);


  return 0;

}