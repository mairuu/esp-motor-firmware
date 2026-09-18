/* ESP32 base controller for a differential-drive robot.

   Speaks a plain line-oriented serial protocol to a ROS2 host, which
   does the /cmd_vel and /odom translation on its side. This is not a
   ROS2 node and does not use micro-ROS.

   57600 baud, 8N1, commands terminated by carriage return:

     e                    -> "<left> <right>"  encoder counts
     r                    -> "OK"              zero both encoders
     o <pwm_l> <pwm_r>    -> "OK"              raw PWM, bypasses the PID
     m <tick_l> <tick_r>  -> "OK"              closed-loop ticks per frame
     u <Kp>:<Kd>:<Ki>:<Ko> -> "OK"             replace the PID gains
     i                    -> "<ax> <ay> <az> <gx> <gy> <gz>"  raw IMU counts

   See ARCHITECTURE.md for the hardware and the reasoning behind the
   scope, and config.h for pins and tuning.
*/

#include <Arduino.h>

#include "commands.h"
#include "config.h"
#include "encoders.h"
#include "motors.h"
#include "mpu6050.h"
#include "pid.h"

/* ---- Command buffer --------------------------------------------------- */

static const size_t ARG_LEN = 16;

static char cmd;
static char argv1[ARG_LEN];
static char argv2[ARG_LEN];
static uint8_t argSlot;   /* 0 = still reading the command letter */
static size_t argIndex;

static uint32_t lastMotorCommand;
static uint32_t nextPID;

static void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  argSlot = 0;
  argIndex = 0;
}

/* Parse "<Kp>:<Kd>:<Ki>:<Ko>" into gains. Returns false unless all four
   values are present, so a malformed update leaves the gains untouched. */
static bool parsePidGains(char *s, int gains[4]) {
  char *cursor = s;
  char *token;
  int count = 0;

  while ((token = strtok_r(cursor, ":", &cursor)) != NULL) {
    if (count >= 4) return false;
    gains[count++] = atoi(token);
  }

  return count == 4;
}

static void runCommand() {
  long arg1 = atol(argv1);
  long arg2 = atol(argv2);
  int gains[4];

  switch (cmd) {
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(' ');
      Serial.println(readEncoder(RIGHT));
      break;

    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;

    case MOTOR_RAW_PWM:
      lastMotorCommand = millis();
      disablePid();
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK");
      break;

    case MOTOR_SPEEDS:
      lastMotorCommand = millis();
      setPidTargets(arg1, arg2);
      Serial.println("OK");
      break;

    case UPDATE_PID:
      if (!parsePidGains(argv1, gains)) {
        Serial.println("Invalid Command");
        break;
      }
      setPidGains(gains[0], gains[1], gains[2], gains[3]);
      Serial.println("OK");
      break;

    case READ_IMU: {
      ImuSample sample;
      if (!readIMU(&sample)) {
        Serial.println("IMU Error");
        break;
      }
      Serial.print(sample.ax);
      Serial.print(' ');
      Serial.print(sample.ay);
      Serial.print(' ');
      Serial.print(sample.az);
      Serial.print(' ');
      Serial.print(sample.gx);
      Serial.print(' ');
      Serial.print(sample.gy);
      Serial.print(' ');
      Serial.println(sample.gz);
      break;
    }

    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* Accumulate bytes until a carriage return, then dispatch. Bare line
   terminators are ignored rather than answered, so a host sending CRLF
   doesn't get a spurious reply. */
static void readSerial() {
  while (Serial.available() > 0) {
    char chr = Serial.read();

    if (chr == '\r' || chr == '\n') {
      if (cmd != 0) runCommand();
      resetCommand();
    } else if (chr == ' ') {
      if (argSlot < 2) argSlot++;
      argIndex = 0;
    } else if (cmd == 0) {
      cmd = chr;
    } else if (argIndex < ARG_LEN - 1) {
      if (argSlot == 1) {
        argv1[argIndex++] = chr;
      } else if (argSlot == 2) {
        argv2[argIndex++] = chr;
      }
    }
  }
}

void setup() {
  Serial.begin(BAUDRATE);

  initMotors();
  bool encodersOk = initEncoders();
  bool imuOk = initIMU();
  resetPID();
  resetCommand();

  lastMotorCommand = millis();
  nextPID = millis() + PID_INTERVAL_MS;

#if PRINT_BOOT_BANNER
  Serial.print("# boot reset=");
  Serial.print((int)esp_reset_reason());
  Serial.print(" encoders=");
  Serial.print(encodersOk ? "ok" : "FAIL");
  Serial.print(" imu=");
  Serial.println(imuOk ? "ok" : "FAIL");
#else
  (void)encodersOk;
  (void)imuOk;
#endif
}

void loop() {
  readSerial();

  uint32_t now = millis();

  /* Signed comparison so the rollover at 2^32 ms is handled. */
  if ((int32_t)(now - nextPID) >= 0) {
    updatePID();
    nextPID += PID_INTERVAL_MS;
    /* If something stalled us for more than a frame, don't try to catch
       up by running a burst of them. */
    if ((int32_t)(now - nextPID) >= 0) nextPID = now + PID_INTERVAL_MS;
  }

  if (now - lastMotorCommand > AUTO_STOP_INTERVAL_MS) {
    lastMotorCommand = now;
    setMotorSpeeds(0, 0);
    disablePid();
  }
}
