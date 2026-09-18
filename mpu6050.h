/* MPU6050 IMU, as wired on a GY-521 breakout, read over I2C. */

#pragma once

#include <stdint.h>

struct ImuSample {
  int16_t ax, ay, az; /* raw accelerometer counts, +/-2g full scale */
  int16_t gx, gy, gz; /* raw gyro counts, +/-250 deg/s full scale */
};

/* Wakes the chip out of its power-on sleep state and confirms it responds
   on the bus. Returns false on a WHO_AM_I mismatch or I2C error (wiring or
   address fault). */
bool initIMU();

/* Reads one accel+gyro sample. Returns false (leaving *out untouched) on
   an I2C transfer error. No filtering or scaling -- the host does that. */
bool readIMU(ImuSample *out);
