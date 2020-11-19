// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <msg/msg_def.h>
#include <pb_encode.h>
#include <pb_decode.h>


namespace pigeon {

constexpr uint8_t kHeader1 = 0x42;
constexpr uint8_t kHeader2 = 0x24;

enum class ParseState {
  HEAD,
  LEN,
  ID,
};

template <typename T>
constexpr T max(T a, T b) {
  return a < b ? b : a;
}

const std::array<uint16_t, 4> message_size = {{Counter2_size, Imu_size, Twist_size, Log_size}};

constexpr uint16_t kMaxBufferSize = max(max(Counter2_size, Imu_size), max(Twist_size, Log_size));  // FIXME: How to implement a compile-time for loop?

class Pigeon {
private:
  std::function<size_t(uint8_t*, size_t)> read_func_;
  std::function<void(const uint8_t*, size_t)> write_func_;
  std::unordered_map<MessageId, std::function<void(const uint8_t*, uint16_t)>> callback_map_;
  uint8_t read_buffer_[kMaxBufferSize];
  uint8_t write_buffer_[kMaxBufferSize];
  
public:  
 Pigeon() {}
 void register_read(std::function<size_t(uint8_t*, size_t)> func) {
   read_func_ = func;
 }
 void register_write(std::function<void(const uint8_t*, size_t)> func) {
   write_func_ = func;
 }

 template <typename T>
 bool publish(MessageId id, const T &message) {
   pb_ostream_t ostream = pb_ostream_from_buffer(write_buffer_, sizeof(write_buffer_));
   bool status = pb_encode(&ostream, fields_map[id], &message);
   size_t length = ostream.bytes_written;
   
   if (status) {
     const uint8_t header[] = {kHeader1, kHeader2, static_cast<uint8_t>(length%255), static_cast<uint8_t>(length/255), static_cast<uint8_t>(id)};
     write_func_(header, 5);
     write_func_(write_buffer_, length);
   }
   return status;
 }
 bool create_subscriber(MessageId id, std::function<void(const uint8_t*, uint16_t)> callback) {
   callback_map_[id] = callback;
   return true;
 }
 void poll() {
   static ParseState state = ParseState::HEAD;
   static uint16_t length = 0;

   // for (;;) {
     switch (state) {
       case ParseState::HEAD: {
         auto size = read_func_(read_buffer_, 1);
         if (size <= 0 || read_buffer_[0] != kHeader1) {
           break;
         }
         size = read_func_(read_buffer_, 1);
         if (size <= 0 || read_buffer_[0] != kHeader2) {
           break;
         }
         state = ParseState::LEN;
         break;
       }
       case ParseState::LEN: {
         auto size = read_func_(read_buffer_, 2);
         if (size <= 0) {
           state = ParseState::HEAD;
           break;
         }
         length = static_cast<uint16_t>(read_buffer_[1]) << 8 | read_buffer_[0];
         state = ParseState::ID;
         break;
       }
       case ParseState::ID: {
         auto size = read_func_(read_buffer_, 1);
         if (size <= 0) {
           state = ParseState::HEAD;
           break;
         }
         auto func = callback_map_.at(static_cast<MessageId>(read_buffer_[0]));
         if (func != nullptr) {
           size = read_func_(read_buffer_, length);
           func(read_buffer_, length);  // callback function
         }
         state = ParseState::HEAD;
         break;
       }
       default:
         state = ParseState::HEAD;
         break;
     }
   // }
 }
 template<Log_Level L>
 void log(const std::string &s) {
   Log message = Log_init_zero;
   size_t size_clip = Log_size;
   message.level = L;
   if (s.size() < size_clip) {
     memcpy(message.log_message, s.c_str(), s.size());
   } else {
     memcpy(message.log_message, s.c_str(), size_clip);
   }
   
   pb_ostream_t ostream = pb_ostream_from_buffer(write_buffer_, sizeof(write_buffer_));
   bool status = pb_encode(&ostream, Log_fields, &message);
   size_t length = ostream.bytes_written;
   
   if (status) {
     const uint8_t header[] = {kHeader1, kHeader2, static_cast<uint8_t>(length%255), static_cast<uint8_t>(length/255), static_cast<uint8_t>(MessageId::LOG)};
     write_func_(header, 5);
     write_func_(write_buffer_, length);
   }
 }

//  auto log_debug = log<Log_Level_DEBUG>;
//  auto log_info = log<Log_Level_INFO>;
//  auto log_warn = log<Log_Level_WARN>;
//  auto log_erro = log<Log_Level_ERROR>;

};

}  // namespace pigeon