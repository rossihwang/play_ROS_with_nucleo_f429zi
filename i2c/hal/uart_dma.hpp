// Copyright <2019> [Copyright rossihwang@gmail.com]

#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <cstring>

extern UART_HandleTypeDef huart3;
constexpr uint16_t kRxbufLength = 512;  // NOTE: must be power of 2
constexpr uint16_t kTxBufLength = 512;  // NOTE: must be power of 2

namespace hal {

class UartDma {
 protected:
  UART_HandleTypeDef *huart_;

  uint8_t rx_buf_[kRxbufLength];
  uint32_t rx_index_;
  inline uint32_t get_dma_read_index() {
    return (kRxbufLength - huart_->hdmarx->Instance->NDTR) & (kRxbufLength - 1);  // NDTR: Number of available bytes in recevie buffer

  }

  uint8_t tx_buf_[kTxBufLength];
  uint32_t tx_write_index_;  // index for last data write in buffer
  uint32_t tx_flush_index_;  // index for last data dma transfered

  bool flush_mtx_;

 public:
  explicit UartDma(UART_HandleTypeDef *huart = &huart3)
    : huart_(huart),
      rx_index_(0),
      tx_write_index_(0),
      tx_flush_index_(0),
      flush_mtx_(false) {
  }

  void init(const char *port=nullptr) {
    reset_rbuf();
  }

  void reset_rbuf(void) {
    HAL_UART_Receive_DMA(huart_, rx_buf_, kRxbufLength);
  }

  int read() {
    int c = -1;
    if (rx_index_ != get_dma_read_index()) {
      c = rx_buf_[rx_index_++];
      rx_index_ = rx_index_ & (kRxbufLength - 1);
    }
    return c;
  }

  void flush() {
    if ((huart_->gState == HAL_UART_STATE_READY) && !flush_mtx_) {
      flush_mtx_ = true;

      if (tx_write_index_ != tx_flush_index_) {
        uint16_t len = tx_flush_index_ < tx_write_index_ ? (tx_write_index_ - tx_flush_index_) : (kTxBufLength - tx_flush_index_);
        HAL_UART_Transmit_DMA(huart_, &(tx_buf_[tx_flush_index_]), len);
        tx_flush_index_ = (tx_flush_index_ + len) & (kTxBufLength - 1);
      }

      flush_mtx_ = false;
    }
  }

  void write(const uint8_t *data, int length) {
    uint16_t n = length;
    n = (n <= kTxBufLength) ? n : kTxBufLength;

    int n_tail = (n <= (kTxBufLength - tx_write_index_)) ? n : kTxBufLength - tx_write_index_;
    std::memcpy(&(tx_buf_[tx_write_index_]), data, n_tail);
    tx_write_index_ = (tx_write_index_ + n) & (kTxBufLength - 1);

    if (n != n_tail) {
      std::memcpy(tx_buf_, &(data[n_tail]), n - n_tail);
    }

    flush();
  }

  unsigned long time() {return HAL_GetTick();}

  void loopback_test() {
    int data;
    data = read();
    if (data != -1) {
      write(reinterpret_cast<const uint8_t*>(&data), 1);
    }
  }
};

}  // namespace hal
