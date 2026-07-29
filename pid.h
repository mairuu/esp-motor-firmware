/* Per-wheel PID speed control.

   Structure and anti-kick / anti-windup handling taken from Mike
   Ferguson's ArbotiX code by way of the upstream ros_arduino_bridge.
*/

#pragma once

/* Current gains, exposed so the main sketch can report/replace them. */
extern int Kp, Kd, Ki, Ko;

void setPidGains(int kp, int kd, int ki, int ko);

/* Clear all PID state and seed it from the current encoder counts, so
   starting from rest doesn't produce an output spike. */
void resetPID();

/* Target speed in encoder ticks per PID frame. Zero on both wheels stops
   the motors and disables the loop. */
void setPidTargets(long leftTicksPerFrame, long rightTicksPerFrame);

/* Hand control of the motors back to the caller (used by raw-PWM mode
   and by the auto-stop timeout). Does not itself change motor output. */
void disablePid();

/* Run one PID frame. No-op while the loop is disabled. */
void updatePID();
