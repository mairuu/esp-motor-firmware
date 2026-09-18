/* MPU6050 IMU, as wired on a GY-521 breakout, read over I2C. */

#pragma once

#include <stdint.h>

struct ImuSample {
  int16_t ax, ay, az; /* raw accelerometer counts, +/-2g full scale (16384/g) */
  int16_t gx, gy, gz; /* raw gyro counts, +/-250 deg/s full scale (131/deg/s) */
};

/* Wakes the chip out of its power-on sleep state, confirms it responds on
   the bus, and writes the full-scale ranges and low-pass filter explicitly
   (see config.h). Returns false on a WHO_AM_I mismatch or I2C error.
   Re-arms the failure latch. */
bool initIMU();

/* The WHO_AM_I byte read by the last initIMU(), or 0 if the read itself
   failed. A genuine MPU6050 answers 0x68; a GY-521 carrying an MPU6500 die
   answers 0x70, an MPU9250 0x71. Printed in the boot banner on failure so
   "wrong chip" and "wrong wiring" are distinguishable in one boot. */
uint8_t imuWhoAmI();

/* Reads one accel+gyro sample. Returns false (leaving *out untouched) on an
   I2C transfer error, or instantly if the IMU has been latched off after
   IMU_MAX_FAILS consecutive failures. No filtering or scaling beyond the
   on-chip DLPF -- the host does that. */
bool readIMU(ImuSample *out);
