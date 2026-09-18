/* Compile-time configuration: pins, rates and limits.

   Everything you might need to change for a different robot lives here.
   Pin assignments match the wiring on the actual robot -- see
   ARCHITECTURE.md before changing them.
*/

#pragma once

#include <stdint.h>

/* ---- Serial link ------------------------------------------------------ */

static const uint32_t BAUDRATE = 57600;

/* Print one line at boot with the reset reason. Useful for spotting a
   silent reboot loop (see ROADMAP.md "Known issue to watch for"). The
   host driver ignores it; set to 0 for a completely silent boot. */
#define PRINT_BOOT_BANNER 1

/* ---- Control loop ----------------------------------------------------- */

static const uint32_t PID_RATE_HZ = 30;
static const uint32_t PID_INTERVAL_MS = 1000 / PID_RATE_HZ;

/* Stop the motors if no 'o' or 'm' command arrives within this window. */
static const uint32_t AUTO_STOP_INTERVAL_MS = 2000;

/* ---- Motor driver (L298N) --------------------------------------------- */

static const int MAX_PWM = 255;

/* 20 kHz keeps the switching whine out of the audible band. The L298N is
   a slow bipolar device -- don't push this much higher or switching
   losses start to dominate. */
static const uint32_t MOTOR_PWM_FREQ_HZ = 20000;
static const uint8_t MOTOR_PWM_BITS = 8;  /* must match MAX_PWM = 2^bits - 1 */

/* PWM is applied to the direction inputs (IN1..IN4) while the enable pins
   are held high -- this is how the robot is already wired. */
static const int RIGHT_MOTOR_FORWARD = 14;
static const int RIGHT_MOTOR_BACKWARD = 12; /* GPIO12 is the MTDI strapping pin */
static const int RIGHT_MOTOR_ENABLE = 13;

static const int LEFT_MOTOR_FORWARD = 25;
static const int LEFT_MOTOR_BACKWARD = 26;
static const int LEFT_MOTOR_ENABLE = 27;

/* ---- Encoders --------------------------------------------------------- */

/* GPIO34/35 are input-only and have no internal pull-ups, so the encoder
   must drive these lines actively (typical of hall-effect motor encoders). */
static const int LEFT_ENC_PIN_A = 34;
static const int LEFT_ENC_PIN_B = 35;

static const int RIGHT_ENC_PIN_A = 23;
static const int RIGHT_ENC_PIN_B = 22;

/* Flip these if a wheel's count runs backwards relative to the direction
   the motor is driven. Encoder sign MUST agree with motor sign or the PID
   will run away to full PWM instead of settling.

   BOTH MEASURED false ON THE ROBOT, 8 Sep 2026. RIGHT_ENC_INVERT was true
   here (commit 8b745d3, "Invert RIGHT_ENC_INVERT to true") and that was
   wrong: driven under `o` with true flashed, the right count ran backwards
   against its own motor. false, reflashed, both sides agree and both wheels
   turn forward. Do not restore true on the strength of the old commit or of
   capstone-docs reference/firmware-protocol.md, which still records it --
   re-measure with cap_ws/src/my_bot/scripts/motor_check.py instead. */
static const bool LEFT_ENC_INVERT = false;
static const bool RIGHT_ENC_INVERT = false;

/* Pulses shorter than this are rejected by the PCNT hardware filter. */
static const uint32_t ENC_GLITCH_FILTER_NS = 1000;

/* ---- IMU (GY-521 / MPU6050) -------------------------------------------- */

/* GPIO21 is the ESP32's usual default SDA and is free on this robot. The
   usual default SCL (GPIO22) is not -- it's RIGHT_ENC_PIN_B -- so SCL is
   moved to GPIO19 (also free) and both are passed explicitly to Wire.begin()
   rather than relying on the core's defaults. */
static const int IMU_SDA_PIN = 21;
static const int IMU_SCL_PIN = 19;

static const uint32_t IMU_I2C_FREQ_HZ = 400000; /* MPU6050 supports fast-mode I2C */

/* Bus timeout for ONE I2C transaction. The core's default is 50 ms and
   readIMU() does two transactions, so a shorted or unplugged bus would stall
   loop() for ~100 ms per `i` command -- three missed PID frames and three
   missed encoder replies, which the host reads as an encoder fault. The
   host polls `i` inside its 30 Hz control loop, so this must stay well
   under PID_INTERVAL_MS. A healthy 14-byte read at 400 kHz takes ~0.4 ms. */
static const uint16_t IMU_I2C_TIMEOUT_MS = 10;

/* After this many consecutive failed reads the IMU is latched off and `i`
   answers "IMU Error" instantly, without touching the bus, until the next
   reset. A dead bus then costs nothing per cycle instead of a timeout each. */
static const uint8_t IMU_MAX_FAILS = 10;

/* Digital low-pass filter, CONFIG register DLPF_CFG.
   4 = 20 Hz gyro / 21 Hz accel bandwidth, 8.3 ms group delay.

   THIS SETTING IS CHOSEN AGAINST THE HOST'S POLL RATE, NOT FOR ITS OWN SAKE.
   The host reads one sample per 30 Hz control frame, so Nyquist is 15 Hz and
   anything the chip passes above that folds back into the reading as noise.
   The power-on default of 0 (filter OFF, 256 Hz) is the worst case and is
   why this register is written at all.

   Was 3 (42 Hz) until 18 Sep 2026, which was still nearly 3x Nyquist: the
   measured gyro noise on the running stack was sigma 0.01053 rad/s
   (0.6 deg/s), 11x the variance seen at rest with the drive unpowered, and
   the host's EKF had to be told so. 4 is the sweet spot.

   DO NOT GO PAST 5. The bandwidth/delay pairs are 3: 42 Hz/4.8 ms,
   4: 20 Hz/8.3 ms, 5: 10 Hz/13.4 ms, 6: 5 Hz/18.6 ms. 5 is the first value
   properly below Nyquist but 13.4 ms is nearly half a 33 ms control frame,
   and that lag lands in the heading estimate that Nav2 steers on. Trading
   noise for phase delay stops paying somewhere between 4 and 5.

   RE-MEASURE AFTER CHANGING THIS: `imu_check.py` for the rest figure, then
   the running stack for the figure the covariance actually wants. See
   cap_ref records/calibration.md "IMU".

   With DLPF_CFG nonzero the gyro output rate is 1 kHz, so SMPLRT_DIV stays
   0 and the data registers always hold the freshest filtered sample. */
static const uint8_t IMU_DLPF_CFG = 4;

/* GY-521 ties AD0 low on the board, giving the 0x68 address. Only pull AD0
   high (0x69) if a second MPU6050 shares this bus. */
static const uint8_t IMU_I2C_ADDR = 0x68;
