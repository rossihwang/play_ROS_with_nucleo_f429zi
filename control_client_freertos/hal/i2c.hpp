// Copyright <2020> [Copyright rossihwang@gmail.com]
#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#include <vector>

namespace hal {

class I2cInterface {
  static std::vector<I2cInterface *> i2c_instance_list;

 protected:
  I2C_HandleTypeDef *hi2c_;
  I2cInterface *instance;
  I2cInterface(I2C_HandleTypeDef *hi2c) : hi2c_(hi2c) {}

 public:
  static I2cInterface *GetInstance(I2C_HandleTypeDef *hi2c) {
    // Check whether the interface is already in used
    for (auto i : i2c_instance_list) {
      if (hi2c->Instance == i->get_base()) {  // FIXME: Not test
        return i;
      }
    }
    I2cInterface *instance = new I2cInterface(hi2c);
    i2c_instance_list.push_back(instance);

    return instance;
  }
  I2C_TypeDef *get_base() const { return hi2c_->Instance; }
  bool Read(uint8_t slave_address, uint8_t *data, uint16_t size,
            uint32_t timeout = 100) {
    if (HAL_I2C_Master_Receive(hi2c_, slave_address << 1, data, size,
                               timeout) == HAL_OK) {
      return true;
    }
    return false;
  }
  bool Write(uint8_t slave_address, uint8_t *data, uint16_t size,
             uint32_t timeout = 100) {
    if (HAL_I2C_Master_Transmit(hi2c_, slave_address << 1, data, size,
                                timeout) == HAL_OK) {
      return true;
    }
    return false;
  }
  bool ReadReg(uint8_t slave_address, uint16_t reg, uint8_t reg_nbits,
               uint8_t *data, uint16_t size, uint32_t timeout = 100) {
    uint16_t reg_nbits_type;
    assert_param(reg_nbits == 8 || reg_nbits == 16);
    if (reg_nbits == 8) {
      reg_nbits_type = I2C_MEMADD_SIZE_8BIT;
    } else {
      reg_nbits_type = I2C_MEMADD_SIZE_16BIT;
    }
    if (HAL_I2C_Mem_Read(hi2c_, slave_address << 1, reg, reg_nbits_type, data,
                         size, timeout) == HAL_OK) {
      return true;
    }
    return false;
  }
  bool WriteReg(uint8_t slave_address, uint16_t reg, uint8_t reg_nbits,
                uint8_t *data, uint16_t size, uint32_t timeout = 100) {
    assert_param(reg_nbits == 8 || reg_nbits == 16);
    uint16_t reg_nbits_type;
    if (reg_nbits == 8) {
      reg_nbits_type = I2C_MEMADD_SIZE_8BIT;
    } else {
      reg_nbits_type = I2C_MEMADD_SIZE_16BIT;
    }
    if (HAL_I2C_Mem_Write(hi2c_, slave_address << 1, reg, reg_nbits_type, data,
                          size, timeout) == HAL_OK) {
      return true;
    }
    return false;
  }
  uint8_t SingleByteReadReg(uint8_t slave_address, uint8_t reg,
                         uint8_t reg_nbits) {
    // NOTE: To simplify, not handle any read exception
    uint8_t data;

    ReadReg(slave_address, reg, reg_nbits, &data, 1);
    return data;
  }
  bool BurstReadReg(uint8_t slave_address, uint8_t reg, uint8_t reg_nbits,
                 uint8_t *data, uint16_t size) {
    if (!ReadReg(slave_address, reg, reg_nbits, data, size)) {
      return false;
    }
    return true;
  }
  bool SingleByteWriteReg(uint8_t slave_address, uint8_t reg, uint8_t reg_nbits,
                       uint8_t data) {
    if (!WriteReg(slave_address, reg, reg_nbits, &data, 1)) {
      return false;
    }
    return true;
  }
  bool BurstWriteReg(uint8_t slave_address, uint8_t reg, uint8_t reg_nbits,
                  uint8_t *data, uint16_t size) {
    if (!WriteReg(slave_address, reg, reg_nbits, data, size)) {
      return false;
    }
    return true;
  }
};

std::vector<I2cInterface *> I2cInterface::i2c_instance_list;

}  // namespace hal
