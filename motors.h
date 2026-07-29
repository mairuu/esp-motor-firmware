/* L298N motor driver. */

#pragma once

void initMotors();

/* Set one motor. i is LEFT or RIGHT, spd is clamped to +/-MAX_PWM. */
void setMotorSpeed(int i, int spd);

void setMotorSpeeds(int leftSpeed, int rightSpeed);
