/* MPU6050 register access over the Arduino core's Wire (I2C) driver.
   No DMP, no on-chip filtering, no scaling -- just wake the chip and hand
   back raw accel/gyro counts. Fusion/scaling is the ROS2 host's job.
*/

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "mpu6050.h"

static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t WHO_AM_I_VALUE = 0x68; /* MPU6050's fixed device ID */

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
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_FREQ_HZ);

  uint8_t who = 0;
  if (!readRegs(REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_VALUE) return false;

  /* PWR_MGMT_1 comes up in sleep mode at power-on; clear SLEEP and select
     the gyro X axis as the clock source (more stable than the internal
     8 MHz RC oscillator that CLKSEL=0 would leave running). */
  return writeReg(REG_PWR_MGMT_1, 0x01);
}

bool readIMU(ImuSample *out) {
  /* ACCEL_XOUT_H..GYRO_ZOUT_L is one contiguous block: 6 bytes accel, 2
     bytes temperature (skipped), 6 bytes gyro. */
  uint8_t buf[14];
  if (!readRegs(REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

  out->ax = (int16_t)((buf[0] << 8) | buf[1]);
  out->ay = (int16_t)((buf[2] << 8) | buf[3]);
  out->az = (int16_t)((buf[4] << 8) | buf[5]);
  out->gx = (int16_t)((buf[8] << 8) | buf[9]);
  out->gy = (int16_t)((buf[10] << 8) | buf[11]);
  out->gz = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}
