/* MPU6050 register access over the Arduino core's Wire (I2C) driver.
   No DMP, no scaling -- wake the chip, pin its configuration, and hand back
   raw accel/gyro counts. Fusion/scaling is the ROS2 host's job.

   The host polls `i` from inside its 30 Hz control loop, on the same serial
   line as the encoder read, so nothing in here may block for long: the bus
   timeout is IMU_I2C_TIMEOUT_MS per transaction and a persistently dead bus
   is latched off rather than retried every cycle. */

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "mpu6050.h"

static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t WHO_AM_I_VALUE = 0x68; /* MPU6050's fixed device ID */

static bool imuOk = false;
static uint8_t failCount = 0;
static uint8_t whoAmI = 0;

static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

/* Repeated-start register read: write the address pointer, then read len
   bytes without releasing the bus in between. */
static bool readRegs(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)IMU_I2C_ADDR, (int)len, (int)true) != (int)len) {
    return false;
  }
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool initIMU() {
  imuOk = false;
  failCount = 0;
  whoAmI = 0;

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_FREQ_HZ);
  Wire.setTimeOut(IMU_I2C_TIMEOUT_MS);

  if (!readRegs(REG_WHO_AM_I, &whoAmI, 1)) return false;
  if (whoAmI != WHO_AM_I_VALUE) return false;

  /* PWR_MGMT_1 comes up in sleep mode at power-on; clear SLEEP and select
     the gyro X axis as the clock source (more stable than the internal
     8 MHz RC oscillator that CLKSEL=0 would leave running). */
  if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;

  /* The ranges and filter below are ALSO the chip's power-on defaults, and
     are written anyway. The host resets the ESP32 by pulsing EN on every
     connect, and that does not power-cycle an MPU hanging off the ESP32's
     3V3 rail -- whatever configuration the chip was left in, it keeps. Four
     writes make the scale the header documents a fact rather than an
     assumption about the previous boot. */
  if (!writeReg(REG_CONFIG, IMU_DLPF_CFG & 0x07)) return false;
  if (!writeReg(REG_GYRO_CONFIG, 0x00)) return false;   /* FS_SEL=0:  +/-250 deg/s */
  if (!writeReg(REG_ACCEL_CONFIG, 0x00)) return false;  /* AFS_SEL=0: +/-2 g */
  /* With the DLPF on, the internal rate is 1 kHz; divider 0 keeps it there
     so the data registers always hold the freshest filtered sample. */
  if (!writeReg(REG_SMPLRT_DIV, 0x00)) return false;

  imuOk = true;
  return true;
}

uint8_t imuWhoAmI() { return whoAmI; }

bool readIMU(ImuSample *out) {
  if (!imuOk) return false;

  /* ACCEL_XOUT_H..GYRO_ZOUT_L is one contiguous block: 6 bytes accel, 2
     bytes temperature (skipped), 6 bytes gyro. */
  uint8_t buf[14];
  if (!readRegs(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
    if (++failCount >= IMU_MAX_FAILS) imuOk = false;
    return false;
  }
  failCount = 0;

  out->ax = (int16_t)((buf[0] << 8) | buf[1]);
  out->ay = (int16_t)((buf[2] << 8) | buf[3]);
  out->az = (int16_t)((buf[4] << 8) | buf[5]);
  out->gx = (int16_t)((buf[8] << 8) | buf[9]);
  out->gy = (int16_t)((buf[10] << 8) | buf[11]);
  out->gz = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}
