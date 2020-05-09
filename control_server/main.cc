#include <string>
#include <iostream>
#include <cstdio>
#include <thread>
#include <unistd.h>
#include <serial/serial.h>
#include <pigeon/pigeon.h>
#include <msg/msg_def.h>
#include <msg/imu.pb.h>


using std::string;
using std::exception;

Pigeon pigeon;

void enumerate_ports() {
  for (auto d : serial::list_ports()) {
    printf("(%s, %s, %s)\n", d.port.c_str(), d.description.c_str(), d.hardware_id.c_str());
  }
}

void print_usage() {
  std::cerr << "Usage: msg_server <dev> <baud_rate>";
}

void callback_imu(const uint8_t *data, uint16_t length) {
  Imu message = Imu_init_zero;
  bool status;
  pb_istream_t stream = pb_istream_from_buffer(data, length);
  status = pb_decode(&stream, Imu_fields, &message);
  if (status) {
    if (message.has_angular_velocity) {
      printf("av x: %f, y: %f, z: %f\n", message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z);
    }
    if (message.has_linear_acceleration) {
      printf("la x: %f, y: %f, z: %f\n", message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z);
    }
    
  } else {
    std::cout << "Decode failed" << std::endl;
  }
}

void callback_twist(const uint8_t *data, uint16_t length) {
  std::cout << "Receive Twist" << std::endl;
}

void thread1() {
  while (1) {
    pigeon.poll();
    usleep(10000); // 10ms
  }
}
void thread2() {
  while (1) {
    Twist message = Twist_init_zero;
    message.has_angular = true;
    message.angular.x = 1;
    message.angular.y = 2;
    message.angular.z = 3;
    message.has_linear = true;
    message.linear.x = 4;
    message.linear.y = 5;
    message.linear.z = 6;
    pigeon.publish<Twist>(MessageId::TWIST, message);
    std::cout << "publish: Twist" << std::endl;
    sleep(3);
  }
}

int main(int argc, char **argv) {
  if (argc < 3) {
    print_usage();
    return 1;
  }

  serial::Serial my_serial(argv[1], atoi(argv[2]), serial::Timeout::simpleTimeout(3000));
  

  pigeon.create_subscriber(MessageId::IMU, callback_imu);
  pigeon.create_subscriber(MessageId::TWIST, callback_twist);
  using std::placeholders::_1;
  using std::placeholders::_2;
  pigeon.register_read(std::bind(static_cast<size_t (serial::Serial::*)(uint8_t*, size_t)>(&serial::Serial::read), &my_serial, _1, _2));
  pigeon.register_write(std::bind(static_cast<size_t (serial::Serial::*)(const uint8_t*, size_t)>(&serial::Serial::write), &my_serial, _1, _2));
  
  std::cout << "Is the serial port open?";
  if (my_serial.isOpen()) {
    std::cout << " Yes." << std::endl;
  } else {
    std::cout << " No." << std::endl;
  }

  //uint8_t buffer[Imu_size];
  my_serial.flushInput();
  std::thread t1(thread1);
  std::thread t2(thread2);

  while (1) {
    t1.join();
    t2.join();
  }
  return 0;

}