// Copyright <2020> [Copyright rossihwang@gmail.com]
// Inspired by https://github.com/kriswiner/MPU6050

#pragma once

#include <cmath>
#include <functional>

#include "../hal/i2c.hpp"
#include "../hal/utility.hpp"
#include "../hal/quaternion.hpp"

using hal::utility::DelayMs;

namespace module {

// RW Registers
// constexpr uint8_t kRegAuxVddio = 0x01;  // Revision 3.2
constexpr uint8_t kRegXaOffsetH = 0x06;  // Not in datasheet
constexpr uint8_t kRegYaOffsetH = 0x08;  // Not in datasheet
constexpr uint8_t kRegZaOffsetH = 0x0A;  // Not in datasheet
constexpr uint8_t kRegSelfTestX = 0x0D;
constexpr uint8_t kRegSelfTestY = 0x0E;
constexpr uint8_t kRegSelfTestZ = 0x0F;
constexpr uint8_t kRegSelfTestA = 0x10;
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegFfThr = 0x1D;
constexpr uint8_t kRegFfDur = 0x1E;
constexpr uint8_t kRegMotThr = 0x1F;
constexpr uint8_t kRegMotDur = 0x20;
constexpr uint8_t kRegZrmotThr = 0x21;
constexpr uint8_t kRegZrmotDur = 0x22;
constexpr uint8_t kRegFifoEn = 0x23;
constexpr uint8_t kRegI2cMstCtrl = 0x24;
constexpr uint8_t kRegI2cSlv0Addr = 0x25;
constexpr uint8_t kRegI2cSlv0Reg = 0x26;
constexpr uint8_t kRegI2cSlv0Ctrl = 0x27;
constexpr uint8_t kRegI2cSlv1Addr = 0x28;
constexpr uint8_t kRegI2cSlv1Reg = 0x29;
constexpr uint8_t kRegI2cSlv1Ctrl = 0x2A;
constexpr uint8_t kRegI2cSlv2Addr = 0x2B;
constexpr uint8_t kRegI2cSlv2Reg = 0x2C;
constexpr uint8_t kRegI2cSlv2Ctrl = 0x2D;
constexpr uint8_t kRegI2cSlv3Addr = 0x2E;
constexpr uint8_t kRegI2cSlv3Reg = 0x2F;
constexpr uint8_t kRegI2cSlv3Ctrl = 0x30;
constexpr uint8_t kRegI2cSlv4Addr = 0x31;
constexpr uint8_t kRegI2cSlv4Reg = 0x32;
constexpr uint8_t kRegI2cSlv4Do = 0x33;
constexpr uint8_t kRegI2cSlv4Ctrl = 0x34;
constexpr uint8_t kRegI2cSlv4Di = 0x35;     // RO
constexpr uint8_t kRegI2cMstStatus = 0x36;  // RO
constexpr uint8_t kRegIntPinCfg = 0x37;
constexpr uint8_t kRegIntEnable = 0x38;
constexpr uint8_t kRegIntStatus = 0x3A;      // RO
constexpr uint8_t kRegAccelXoutH = 0x3B;     // RO
constexpr uint8_t kRegAccelXoutL = 0x3C;     // RO
constexpr uint8_t kRegAccelYoutH = 0x3D;     // RO
constexpr uint8_t kRegAccelYoutL = 0x3E;     // RO
constexpr uint8_t kRegAccelZoutH = 0x3F;     // RO
constexpr uint8_t kRegAccelZoutL = 0x40;     // RO
constexpr uint8_t kRegTempOutH = 0x41;       // RO
constexpr uint8_t kRegTempOutL = 0x42;       // RO
constexpr uint8_t kRegGyroXoutH = 0x43;      // RO
constexpr uint8_t kRegGyroXoutL = 0x44;      // RO
constexpr uint8_t kRegGyroYoutH = 0x45;      // RO
constexpr uint8_t kRegGyroYoutL = 0x46;      // RO
constexpr uint8_t kRegGyroZoutH = 0x47;      // RO
constexpr uint8_t kRegGyroZoutL = 0x48;      // RO
constexpr uint8_t kRegExtSensData00 = 0x49;  // RO
constexpr uint8_t kRegExtSensData01 = 0x4A;  // RO
constexpr uint8_t kRegExtSensData02 = 0x4B;  // RO
// ...
constexpr uint8_t kRegMotDetectStatus = 0x61;  // RO
constexpr uint8_t kRegI2cSlv0Do = 0x63;
constexpr uint8_t kRegI2cSlv1Do = 0x64;
constexpr uint8_t kRegI2cSlv2Do = 0x65;
constexpr uint8_t kRegI2cSlv3Do = 0x66;
constexpr uint8_t kRegI2cMstDelayCtrl = 0x67;
constexpr uint8_t kRegSignalPathReset = 0x68;
constexpr uint8_t kRegMotDetectCtrl = 0x69;
constexpr uint8_t kRegUserCtrl = 0x6A;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegPwrMgmt2 = 0x6C;
constexpr uint8_t kRegFifoCountH = 0x72;
constexpr uint8_t kRegFifoCountL = 0x73;
constexpr uint8_t kRegFifoRW = 0x74;
constexpr uint8_t kRegWhoAmI = 0x75;  // RO

constexpr float PI = 3.14159265358979323846f;

using namespace std::placeholders;

class Mpu6050 {
  enum class GFullScale {
    DPS250 = 0,
    DPS500,
    DPS1K,
    DPS2K
  };  // Gyroscope full-scale range
  enum class AFullScale {
    G2 = 0,
    G4,
    G8,
    G16
  };  // accelerometer full-scale range
  const std::array<float, 4> GResLUT = {
      250.0 / 32768.0, 500.0 / 32768.0, 1000.0 / 32768.0,
      2000.0 / 32768.0};  // Gyroscope resolution lookup table
  const std::array<float, 4> AResLUT = {
      2.0 / 32768.0, 4.0 / 32768.0, 8.0 / 32768.0,
      16.0 / 32768.0};  // Accelerometer resolution lookup table

  hal::I2cInterface *i2c_if_;
  uint8_t address_;
  std::function<uint8_t(uint8_t)> SingleByteReadWrp;
  std::function<bool(uint8_t, uint8_t *, uint16_t)> BurstReadWrp;
  std::function<void(uint8_t, uint8_t)> SingleByteWriteWrp;
  std::function<bool(uint8_t, uint8_t *, uint16_t)> BurstWriteWrp;
  GFullScale g_scale_;
  AFullScale a_scale_;
  float g_res_;
  float a_res_;
  uint8_t reg_nbits_;
  std::array<float, 3> gyro_offset_;
  std::array<float, 3> accel_offset_;
  Quaternion<float> q_est_;
  uint32_t last_timestamp_;

 public:
  Mpu6050(I2C_HandleTypeDef *hi2c, uint8_t dev_addr, uint8_t reg_nbits = 8)
      : address_(dev_addr),
        g_scale_(GFullScale::DPS250),
        a_scale_(AFullScale::G2),
        reg_nbits_(reg_nbits),
        gyro_offset_(),
        accel_offset_(),
        q_est_(1, 0, 0, 0),
        last_timestamp_(0) {
    i2c_if_ = hal::I2cInterface::GetInstance(hi2c);
    SingleByteReadWrp = std::bind(&hal::I2cInterface::SingleByteReadReg,
                                  i2c_if_, address_, _1, reg_nbits_);
    BurstReadWrp = std::bind(&hal::I2cInterface::BurstReadReg, i2c_if_,
                             address_, _1, reg_nbits_, _2, _3);
    SingleByteWriteWrp = std::bind(&hal::I2cInterface::SingleByteWriteReg,
                                   i2c_if_, address_, _1, reg_nbits_, _2);
    BurstWriteWrp = std::bind(&hal::I2cInterface::BurstWriteReg, i2c_if_,
                              address_, _1, reg_nbits_, _2, _3);
    g_res_ = GResLUT[static_cast<int>(g_scale_)];
    a_res_ = AResLUT[static_cast<int>(a_scale_)];
    gyro_offset_ = {0, 0, 0};
    accel_offset_ = {0, 0, 0};
  }
  void Init() {
    SingleByteWriteWrp(kRegPwrMgmt1,
                       0x00);  // Clear sleep mode bit (6), enbale all sensors
    DelayMs(100);
    SingleByteWriteWrp(kRegPwrMgmt1, 0x01);  //
    SingleByteWriteWrp(kRegConfig, 0x03);
    SingleByteWriteWrp(kRegSmplrtDiv, 0x04);

    uint8_t data;
    data = SingleByteReadWrp(kRegGyroConfig);
    SingleByteWriteWrp(kRegGyroConfig, data & ~0xE0);
    SingleByteWriteWrp(kRegGyroConfig, data & ~0x18);
    SingleByteWriteWrp(kRegGyroConfig, data | static_cast<uint8_t>(g_scale_)
                                                  << 3);

    data = SingleByteReadWrp(kRegAccelConfig);
    SingleByteWriteWrp(kRegAccelConfig, data & ~0xE0);
    SingleByteWriteWrp(kRegAccelConfig, data & ~0x18);
    SingleByteWriteWrp(kRegAccelConfig, data | static_cast<uint8_t>(a_scale_)
                                                   << 3);

    SingleByteWriteWrp(kRegIntPinCfg, 0x22);
    SingleByteWriteWrp(kRegIntEnable, 0x01);
  }
  void Reset() {
    SingleByteWriteWrp(kRegPwrMgmt1, 0x80);
    DelayMs(100);
  }
  //
  void ComputeCalibrateParams() {
    Reset();

    SingleByteWriteWrp(kRegPwrMgmt1, 0x01);
    SingleByteWriteWrp(kRegPwrMgmt2, 0x00);
    DelayMs(200);

    SingleByteWriteWrp(kRegIntEnable, 0x00);
    SingleByteWriteWrp(kRegFifoEn, 0x00);
    SingleByteWriteWrp(kRegPwrMgmt1, 0x00);
    SingleByteWriteWrp(kRegI2cMstCtrl, 0x00);
    SingleByteWriteWrp(kRegUserCtrl, 0x00);
    SingleByteWriteWrp(kRegUserCtrl, 0x0C);
    DelayMs(15);

    SingleByteWriteWrp(kRegConfig, 0x01);
    SingleByteWriteWrp(kRegSmplrtDiv, 0x00);
    SingleByteWriteWrp(kRegGyroConfig, 0x00);
    SingleByteWriteWrp(kRegAccelConfig, 0x00);

    uint16_t gyro_sensitivity = 131;     // 250 / 32768 * 131 ~= 1dps
    uint16_t accel_sensitivity = 16384;  // 2 / 32768 * 16384 = 1G

    SingleByteWriteWrp(kRegUserCtrl, 0x40);
    SingleByteWriteWrp(kRegFifoEn, 0x78);
    DelayMs(80);

    uint8_t data[12] = {0};
    SingleByteWriteWrp(kRegFifoEn, 0x00);
    BurstReadWrp(kRegFifoCountH, data, 2);
    uint16_t fifo_count = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    uint16_t packet_count = fifo_count / 12;

    for (int i = 0; i < packet_count; ++i) {
      std::array<int16_t, 3> accel_temp;
      std::array<int16_t, 3> gyro_temp;
      BurstReadWrp(kRegFifoRW, data, 12);
      accel_temp[0] = (static_cast<int16_t>(data[0]) << 8) | data[1];
      accel_temp[1] = (static_cast<int16_t>(data[2]) << 8) | data[3];
      accel_temp[2] = (static_cast<int16_t>(data[4]) << 8) | data[5];
      gyro_temp[0] = (static_cast<int16_t>(data[6] << 8) | data[7]);
      gyro_temp[1] = (static_cast<int16_t>(data[8] << 8) | data[9]);
      gyro_temp[2] = (static_cast<int16_t>(data[10] << 8) | data[11]);

      accel_offset_[0] += static_cast<int32_t>(accel_temp[0]);
      accel_offset_[1] += static_cast<int32_t>(accel_temp[1]);
      accel_offset_[2] += static_cast<int32_t>(accel_temp[2]);
      gyro_offset_[0] += static_cast<int32_t>(gyro_temp[0]);
      gyro_offset_[1] += static_cast<int32_t>(gyro_temp[1]);
      gyro_offset_[2] += static_cast<int32_t>(gyro_temp[2]);
    }
    accel_offset_[0] /= packet_count;
    accel_offset_[1] /= packet_count;
    accel_offset_[2] /= packet_count;
    gyro_offset_[0] /= packet_count;
    gyro_offset_[1] /= packet_count;
    gyro_offset_[2] /= packet_count;

    // Remove gravity from the z-axis accelerometer bias calculation
    if (0 < accel_offset_[2]) {
      accel_offset_[2] -= accel_sensitivity;
    } else {
      accel_offset_[2] += accel_sensitivity;
    }

    for (int i = 0; i < 3; ++i) {             // for manual subtraction
      accel_offset_[i] /= accel_sensitivity;  // unit in dps
      gyro_offset_[i] /= gyro_sensitivity;    // unit in G
    }
  }

  // HOW: actively actuate to the sensor, and compare the results with the
  // expected ones The self-test returns a ratio in percentage, which should be
  // less than 14%. Please refer to the "SELF TEST RESPONSE" session in
  // datasheet
  void SelfTest(std::array<float, 6> &ratio) {
    uint8_t data[4] = {0};
    std::array<uint8_t, 6> self_test;
    std::array<float, 6> factory_trim;

    SingleByteWriteWrp(kRegAccelConfig,
                       0xF0);  // Enable self test on all three axes and set
                               // accelerometer range to +/- 8G
    SingleByteWriteWrp(kRegGyroConfig,
                       0xE0);  // Enable self test on all three axes and set
                               // gyro range to +/- 250 degrees/s
    DelayMs(250);

    data[0] = SingleByteReadWrp(kRegSelfTestX);  // X-axis self-test results
    data[1] = SingleByteReadWrp(kRegSelfTestY);  // Y-axis self-test results
    data[2] = SingleByteReadWrp(kRegSelfTestZ);  // Z-axis self-test results
    data[3] = SingleByteReadWrp(kRegSelfTestA);  // Mixed-axis self-test results
    // Extract the acceleration test results
    self_test[0] =
        (data[0] >> 3) |
        (data[3] & 0x30) >> 4;  // XA_TEST result is a five-bit unsigned integer
    self_test[1] =
        (data[1] >> 3) |
        (data[3] & 0x0C) >> 2;  // YA_TEST result is a five-bit unsigned integer
    self_test[2] =
        (data[2] >> 3) |
        (data[3] & 0x03) >> 0;  // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results
    self_test[3] = data[0] & 0x1F;
    self_test[4] = data[1] & 0x1F;
    self_test[5] = data[2] & 0x1F;
    // Process results to allow final comparison with factory set values
    if (self_test[0] != 0) {
      factory_trim[0] =
          (4096.0 * 0.34) *
          (std::pow((0.92 / 0.34), ((self_test[0] - 1.0) /
                                    30.0)));  // FT[Xa] factory trim calculation
    } else {
      factory_trim[0] = 0;
    }
    if (self_test[1] != 0) {
      factory_trim[1] =
          (4096.0 * 0.34) *
          (std::pow((0.92 / 0.34), ((self_test[1] - 1.0) /
                                    30.0)));  // FT[Ya] factory trim calculation
    } else {
      factory_trim[1] = 0;
    }
    if (self_test[2] != 0) {
      factory_trim[2] =
          (4096.0 * 0.34) *
          (std::pow((0.92 / 0.34), ((self_test[2] - 1.0) /
                                    30.0)));  // FT[Za] factory trim calculation
    } else {
      factory_trim[2] = 0;
    }
    if (self_test[3] != 0) {
      factory_trim[3] =
          (25.0 * 131.0) *
          (std::pow(1.046,
                    (self_test[3] - 1.0)));  // FT[Xg] factory trim calculation
    } else {
      factory_trim[3] = 0;
    }
    if (self_test[4] != 0) {
      factory_trim[4] =
          (-25.0 * 131.0) *
          (std::pow(1.046,
                    (self_test[4] - 1.0)));  // FT[Yg] factory trim calculation
    } else {
      factory_trim[4] = 0;
    }
    if (self_test[5] != 0) {
      factory_trim[5] =
          (25.0 * 131.0) *
          (std::pow(1.046,
                    (self_test[5] - 1.0)));  // FT[Zg] factory trim calculation
    } else {
      factory_trim[5] = 0;
    }

    for (int i = 0; i < 6; ++i) {
      ratio[i] = 
          100.0 + 100.0 * (self_test[i] - factory_trim[i]) /
          factory_trim[i];  // FIXME: This is wired, please refer to
                             // https://github.com/kriswiner/MPU6050/issues/11
    }
  }
  bool CheckCommunication() {
    constexpr uint8_t kRegWhoAmIDefault = 0x68;
    // constexpr uint8_t kRegPwrMgmt1Default = 0x40;
    uint8_t data;
    data = SingleByteReadWrp(kRegWhoAmI);
    if (data != kRegWhoAmIDefault) {
      return false;
    }
    // data = SingleByteReadWrp(kRegPwrMgmt1);
    // if (data != kRegPwrMgmt1Default) {
    //   return false;
    // }
    return true;
  }
  std::tuple<float, float, float> ReadGyroData(bool is_compensated = true) {
    uint8_t data[6];
    float x, y, z;
    BurstReadWrp(kRegGyroXoutH, data, 6);
    x = static_cast<int16_t>(data[0] << 8 | data[1]) * g_res_;
    y = static_cast<int16_t>(data[2] << 8 | data[3]) * g_res_;
    z = static_cast<int16_t>(data[4] << 8 | data[5]) * g_res_;

    if (is_compensated) {
      x -= gyro_offset_[0];
      y -= gyro_offset_[1];
      z -= gyro_offset_[2];
    }

    return std::make_tuple(x, y, z);
  }
  std::tuple<float, float, float> ReadAccelData(bool is_compensated = true) {
    uint8_t data[6];
    float x, y, z;
    BurstReadWrp(kRegAccelXoutH, data, 6);
    x = static_cast<int16_t>(data[0] << 8 | data[1]) * a_res_;
    y = static_cast<int16_t>(data[2] << 8 | data[3]) * a_res_;
    z = static_cast<int16_t>(data[4] << 8 | data[5]) * a_res_;

    if (is_compensated) {
      x -= accel_offset_[0];
      y -= accel_offset_[1];
      z -= accel_offset_[2];
    }
    return std::make_tuple(x, y, z);
  }
  float ReadTempData() {
    uint8_t data[2];
    BurstReadWrp(kRegTempOutH, data, 2);
    return static_cast<int16_t>(data[0] << 8 | data[1]) / 340. +
           36.53;  // degree C
  }
  bool IsDataReady() {
    if (SingleByteReadWrp(kRegIntStatus) & 0x01) {
      return true;
    }
    return false;
  }
  // https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
  Quaternion<float> FuseToQuaternion(float ax, float ay, float az, float gx, float gy, float gz, uint32_t timestamp) {

    if (last_timestamp_ == 0) {  // skip update
      last_timestamp_ = timestamp;
      return q_est_;  
    }
    Quaternion<float> w(0, ax, ay, az);
    w /= w.Norm();

    float f1 = 2 * q_est_.q2 * q_est_.q4 - 2 * q_est_.q1 * q_est_.q3 - w.q2;
    float f2 = 2 * q_est_.q1 * q_est_.q2 + 2 * q_est_.q3 * q_est_.q4 - w.q3; 
    float f3 = 1.0 - 2 * q_est_.q2 * q_est_.q2 - 2 * q_est_.q3 * q_est_.q3 - w.q4;
    float j_11or24 = 2 * q_est_.q3;
    float j_12or23 = 2 * q_est_.q4;
    float j_13or22 = 2 * q_est_.q1;
    float j_14or21 = 2 * q_est_.q2;
    float j_32 = 2 * j_14or21;
    float j_33 = 2 * j_11or24;

    Quaternion<float> dot_prod;  // orientation from vector observations
    dot_prod.q1 = j_14or21 * f2 - j_11or24 * f1;
    dot_prod.q2 = j_12or23 * f1 + j_13or22 * f2 - j_32 * f3;
    dot_prod.q3 = j_12or23 * f2 - j_33* f3 - j_13or22 * f1;
    dot_prod.q4 = j_14or21 * f1 + j_11or24 * f2;

    dot_prod /= dot_prod.Norm();

    Quaternion<float> half_q_est(q_est_ * 0.5);
    Quaternion<float> derivative;
    derivative.q1 = -half_q_est.q2 * gx - half_q_est.q3 * gy - half_q_est.q4 * gz;
    derivative.q2 = half_q_est.q1 * gx + half_q_est.q3 * gz - half_q_est.q4 * gy;
    derivative.q3 = half_q_est.q1 * gy - half_q_est.q2 * gz + half_q_est.q4 * gx;
    derivative.q4 = half_q_est.q1 * gz + half_q_est.q2 * gy - half_q_est.q3 * gx;

    constexpr float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    constexpr float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta

    float delta_t = (timestamp - last_timestamp_) / 1000000000.0f;
    q_est_.q1 += (derivative.q1 - (beta * dot_prod.q1)) * delta_t;
    q_est_.q2 += (derivative.q2 - (beta * dot_prod.q2)) * delta_t;
    q_est_.q3 += (derivative.q3 - (beta * dot_prod.q3)) * delta_t;
    q_est_.q4 += (derivative.q4 - (beta * dot_prod.q4)) * delta_t;

    q_est_ /= q_est_.Norm();

    return q_est_;
  }
};

}  // namespace sensor