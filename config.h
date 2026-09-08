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
