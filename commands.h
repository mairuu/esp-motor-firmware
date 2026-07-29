/* Single-letter commands sent by the ROS2 host over the serial link. */

#pragma once

#define READ_ENCODERS  'e'
#define RESET_ENCODERS 'r'
#define MOTOR_RAW_PWM  'o'
#define MOTOR_SPEEDS   'm'
#define UPDATE_PID     'u'

#define LEFT  0
#define RIGHT 1
