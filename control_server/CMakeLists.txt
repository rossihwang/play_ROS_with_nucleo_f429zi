cmake_minimum_required(VERSION 3.10)
project(msg_server)
set(CMAKE_BUILD_TYPE Debug)

add_library(SERIAL STATIC)
target_sources(SERIAL PRIVATE ../Thirdparty/serial/src/impl/list_ports/list_ports_linux.cc
                              ../Thirdparty/serial/src/impl/unix.cc
                              ../Thirdparty/serial/src/serial.cc)
target_include_directories(SERIAL PUBLIC ../Thirdparty/serial/include)

add_library(NANOPB STATIC)
target_sources(NANOPB PRIVATE ../Thirdparty/nanopb/src/pb_common.c
                              ../Thirdparty/nanopb/src/pb_decode.c
                              ../Thirdparty/nanopb/src/pb_encode.c)
target_include_directories(NANOPB PUBLIC ../Thirdparty/nanopb/include)

set(MSG_SOURCE ../shared/msg/imu.pb.c 
               ../shared/msg/twist.pb.c 
               ../shared/msg/Vector3.pb.c
               ../shared/msg/logging.pb.c
               ../shared/msg/pwm_ctrl.pb.c
               ../shared/msg/wheels_counter.pb.c)
set(MSG_INCLUDE_DIRS ../shared/)

add_executable(msg_server main.cc ${MSG_SOURCE})
target_include_directories(msg_server PRIVATE . ${MSG_INCLUDE_DIRS})
target_link_libraries(msg_server SERIAL NANOPB pthread)

add_executable(pwm_ctrl pwm_ctrl.cc ${MSG_SOURCE})
target_include_directories(pwm_ctrl PRIVATE . ${MSG_INCLUDE_DIRS})
target_link_libraries(pwm_ctrl SERIAL NANOPB pthread)

add_executable(twist_ctrl twist_ctrl.cc ${MSG_SOURCE})
target_include_directories(twist_ctrl PRIVATE . ${MSG_INCLUDE_DIRS})
target_link_libraries(twist_ctrl SERIAL NANOPB pthread)