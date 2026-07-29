#include <Arduino.h>

#include "commands.h"
#include "config.h"
#include "motors.h"

/* Drive one H-bridge half. The opposite input is always zeroed first so
   the bridge never sees both inputs driven at once. */
static void applyPwm(int forwardPin, int backwardPin, int spd) {
  spd = constrain(spd, -MAX_PWM, MAX_PWM);

  if (spd >= 0) {
    ledcWrite(backwardPin, 0);
    ledcWrite(forwardPin, spd);
  } else {
    ledcWrite(forwardPin, 0);
    ledcWrite(backwardPin, -spd);
  }
}

void initMotors() {
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);

  ledcAttach(LEFT_MOTOR_FORWARD, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  ledcAttach(LEFT_MOTOR_BACKWARD, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  ledcAttach(RIGHT_MOTOR_FORWARD, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  ledcAttach(RIGHT_MOTOR_BACKWARD, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);

  setMotorSpeeds(0, 0);
}

void setMotorSpeed(int i, int spd) {
  if (i == LEFT) {
    applyPwm(LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, spd);
  } else {
    applyPwm(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, spd);
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
