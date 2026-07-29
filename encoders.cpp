/* Quadrature decoding on the ESP32's PCNT peripheral.

   Each wheel gets one PCNT unit with two channels. Channel A counts both
   edges of the A line using B as the direction input; channel B does the
   mirror image. That yields full 4x decoding entirely in hardware -- no
   interrupts, and no counts lost at speed.

   The hardware counter is only 16-bit, so the unit is configured to
   accumulate into a wider software counter each time it hits a limit
   (flags.accum_count, which requires watch points on both limits).
*/

#include <Arduino.h>
#include <driver/pulse_cnt.h>

#include "commands.h"
#include "config.h"
#include "encoders.h"

/* Well inside the +/-32767 the hardware counter can hold. */
static const int PCNT_HIGH_LIMIT = 10000;
static const int PCNT_LOW_LIMIT = -10000;

static pcnt_unit_handle_t encoderUnit[2] = {nullptr, nullptr};

static bool setupUnit(pcnt_unit_handle_t *unit, int pinA, int pinB) {
  pcnt_unit_config_t unitConfig = {};
  unitConfig.low_limit = PCNT_LOW_LIMIT;
  unitConfig.high_limit = PCNT_HIGH_LIMIT;
  unitConfig.flags.accum_count = 1;
  if (pcnt_new_unit(&unitConfig, unit) != ESP_OK) return false;

  pcnt_glitch_filter_config_t filterConfig = {};
  filterConfig.max_glitch_ns = ENC_GLITCH_FILTER_NS;
  if (pcnt_unit_set_glitch_filter(*unit, &filterConfig) != ESP_OK) return false;

  pcnt_chan_config_t chanAConfig = {};
  chanAConfig.edge_gpio_num = pinA;
  chanAConfig.level_gpio_num = pinB;
  pcnt_channel_handle_t chanA = nullptr;
  if (pcnt_new_channel(*unit, &chanAConfig, &chanA) != ESP_OK) return false;

  pcnt_chan_config_t chanBConfig = {};
  chanBConfig.edge_gpio_num = pinB;
  chanBConfig.level_gpio_num = pinA;
  pcnt_channel_handle_t chanB = nullptr;
  if (pcnt_new_channel(*unit, &chanBConfig, &chanB) != ESP_OK) return false;

  pcnt_channel_set_edge_action(chanA, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                               PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(chanA, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(chanB, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                               PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(chanB, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  /* Required for accum_count to widen the 16-bit hardware counter. */
  if (pcnt_unit_add_watch_point(*unit, PCNT_HIGH_LIMIT) != ESP_OK) return false;
  if (pcnt_unit_add_watch_point(*unit, PCNT_LOW_LIMIT) != ESP_OK) return false;

  if (pcnt_unit_enable(*unit) != ESP_OK) return false;
  if (pcnt_unit_clear_count(*unit) != ESP_OK) return false;
  if (pcnt_unit_start(*unit) != ESP_OK) return false;

  return true;
}

bool initEncoders() {
  bool ok = setupUnit(&encoderUnit[LEFT], LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
  ok = setupUnit(&encoderUnit[RIGHT], RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B) && ok;
  return ok;
}

long readEncoder(int i) {
  if (encoderUnit[i] == nullptr) return 0;

  int value = 0;
  if (pcnt_unit_get_count(encoderUnit[i], &value) != ESP_OK) return 0;

  const bool invert = (i == LEFT) ? LEFT_ENC_INVERT : RIGHT_ENC_INVERT;
  return invert ? -(long)value : (long)value;
}

void resetEncoders() {
  for (int i = 0; i < 2; i++) {
    if (encoderUnit[i] != nullptr) pcnt_unit_clear_count(encoderUnit[i]);
  }
}
