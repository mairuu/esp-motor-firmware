#include <Arduino.h>

#include "commands.h"
#include "config.h"
#include "encoders.h"
#include "motors.h"
#include "pid.h"

int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

typedef struct {
  double TargetTicksPerFrame;
  long Encoder;
  long PrevEnc;

  /* Previous input rather than previous error, to avoid derivative kick
     when the setpoint changes. */
  int PrevInput;

  /* Integrated term rather than integrated error, so gain changes take
     effect without re-scaling the accumulated history. */
  int ITerm;

  long output;
} SetPointInfo;

static SetPointInfo leftPID, rightPID;

/* False means the loop is idle and leaves the motors alone. */
static bool moving = false;

void setPidGains(int kp, int kd, int ki, int ko) {
  Kp = kp;
  Kd = kd;
  Ki = ki;
  Ko = ko;
}

void resetPID() {
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = readEncoder(LEFT);
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;

  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.Encoder = readEncoder(RIGHT);
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.output = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
}

void setPidTargets(long leftTicksPerFrame, long rightTicksPerFrame) {
  if (leftTicksPerFrame == 0 && rightTicksPerFrame == 0) {
    setMotorSpeeds(0, 0);
    resetPID();
    moving = false;
    return;
  }

  /* Seed from the current counts if we're starting from rest, otherwise
     the first frame sees a huge apparent error. */
  if (!moving) resetPID();

  leftPID.TargetTicksPerFrame = leftTicksPerFrame;
  rightPID.TargetTicksPerFrame = rightTicksPerFrame;
  moving = true;
}

void disablePid() {
  resetPID();
  moving = false;
}

static void doPID(SetPointInfo *p) {
  int input = p->Encoder - p->PrevEnc;
  long Perror = p->TargetTicksPerFrame - input;

  long output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;

  /* Clamp, and stop winding up the integrator once we saturate. */
  if (output >= MAX_PWM) {
    output = MAX_PWM;
  } else if (output <= -MAX_PWM) {
    output = -MAX_PWM;
  } else {
    p->ITerm += Ki * Perror;
  }

  p->output = output;
  p->PrevInput = input;
}

void updatePID() {
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  if (!moving) {
    /* Re-seed once after coming to a stop so the next start is clean.
       PrevInput is a good proxy for "has already been reset". */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  doPID(&leftPID);
  doPID(&rightPID);

  setMotorSpeeds(leftPID.output, rightPID.output);
}
