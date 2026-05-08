#include "wokwi-api.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  pin_t pin_vcc;
  pin_t pin_gnd;
  pin_t pin_viout;
  pin_t pin_filter;
  pin_t pin_ip_plus;
  pin_t pin_ip_minus;

  uint32_t attr_current_a;
  uint32_t attr_vcc_volts;
  uint32_t attr_zero_current_v;
  uint32_t attr_sensitivity_mv_per_a;

  timer_t timer;
} chip_state_t;

static void update_output(chip_state_t *chip) {
  float currentA = attr_read_float(chip->attr_current_a);
  float vccVolts = attr_read_float(chip->attr_vcc_volts);
  float zeroCurrentV = attr_read_float(chip->attr_zero_current_v);
  float sensitivityMvPerA = attr_read_float(chip->attr_sensitivity_mv_per_a);

  float sensitivityVPerA = sensitivityMvPerA / 1000.0f;
  float vout = zeroCurrentV + (currentA * sensitivityVPerA);

  if (vout < 0.0f) vout = 0.0f;
  if (vout > vccVolts) vout = vccVolts;

  pin_dac_write(chip->pin_viout, vout);
}

static void on_timer(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  update_output(chip);
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  chip->pin_vcc = pin_init("VCC", INPUT);
  chip->pin_gnd = pin_init("GND", INPUT);
  chip->pin_viout = pin_init("VIOUT", ANALOG);
  chip->pin_filter = pin_init("FILTER", INPUT);
  chip->pin_ip_plus = pin_init("IP+", INPUT);
  chip->pin_ip_minus = pin_init("IP-", INPUT);

  chip->attr_current_a = attr_init_float("currentA", 0.0f);
  chip->attr_vcc_volts = attr_init_float("vccVolts", 5.0f);
  chip->attr_zero_current_v = attr_init_float("zeroCurrentV", 2.5f);
  chip->attr_sensitivity_mv_per_a = attr_init_float("sensitivityMvPerA", 40.0f);

  const timer_config_t timer_config = {
    .callback = on_timer,
    .user_data = chip,
  };

  chip->timer = timer_init(&timer_config);
  timer_start(chip->timer, 10000, true); // 10 ms, repetitivo

  update_output(chip);
}
