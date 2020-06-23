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
  std::cerr << "Usage: twist_ctrl <dev> <baud_rate> <x> <theta>" << std::endl;
}

void callback_log(const uint8_t *data, uint16_t length) {
  Log message = Log_init_zero;
  static std::array<std::string, 4> level_string = {"DEBUG", "INFO", "WARN", "ERROR"};

  bool status;
  pb_istream_t stream = pb_istream_from_buffer(data, length);
  status = pb_decode(&stream, Log_fields, &message);
  if (status) {
    std::string level = level_string[message.level];
    std::string log_message(message.log_message);
    std::cout << level << ": " << log_message << std::endl;
  } else {
    std::cout << "Log Decode failed" << std::endl;
  }
}

void thread1() {
  while (1) {
    pg.poll();
    usleep(10000);  // 10ms
  }
}

int main(int argc, char **argv) {
  if (argc < 5) {
    print_usage();
    return 1;
  }

  serial::Serial my_serial(argv[1], atoi(argv[2]), serial::Timeout::simpleTimeout(3000));
  
  pg.create_subscriber(MessageId::LOG, callback_log);
  std::cout << "subscriber to log" << std::endl;
  using std::placeholders::_1;
  using std::placeholders::_2;
  pg.register_read(std::bind(static_cast<size_t (serial::Serial::*)(uint8_t*, size_t)>(&serial::Serial::read), &my_serial, _1, _2));
  pg.register_write(std::bind(static_cast<size_t (serial::Serial::*)(const uint8_t*, size_t)>(&serial::Serial::write), &my_serial, _1, _2));
  
  std::cout << "Is the serial port open?";
  if (my_serial.isOpen()) {
    std::cout << " Yes." << std::endl;
  } else {
    std::cout << " No." << std::endl;
    return 1;
  }

  Twist message = Twist_init_zero;
  message.has_angular = true;
  message.angular.x = 0;
  message.angular.y = 0;
  message.angular.z = atof(argv[4]);
  message.has_linear = true;
  message.linear.x = atof(argv[3]);
  message.linear.y =  0;
  message.linear.z = 0;
  pg.publish<Twist>(MessageId::TWIST, message);
  printf("publish: Twist - angluar: (%f, %f, %f), linear: (%f, %f, %f)\n", message.angular.x, message.angular.y, message.angular.z,
                                                                           message.linear.x, message.linear.y, message.linear.z);
  my_serial.flush();

  std::thread t1(thread1);

  while (1) {
    t1.join();
  }

  return 0;

}