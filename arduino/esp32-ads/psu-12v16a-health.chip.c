#include "wokwi-api.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

typedef struct {
  pin_t pin_vout;
  pin_t pin_gnd;
  pin_t pin_vmon;
  pin_t pin_imon;
  pin_t pin_pgood;

  uint32_t attr_nominal_volts;
  uint32_t attr_load_current_a;
  uint32_t attr_health_percent;
  uint32_t attr_ripple_mv;
  uint32_t attr_power_on;
  uint32_t attr_max_current_a;
  uint32_t attr_vmon_full_scale;
  uint32_t attr_imon_full_scale;
  uint32_t attr_internal_res_mohm;

  timer_t timer;
  float phase;
} chip_state_t;

static float clampf(float x, float minv, float maxv) {
  if (x < minv) return minv;
  if (x > maxv) return maxv;
  return x;
}

static void update_outputs(chip_state_t *chip) {
  float nominal_v = attr_read_float(chip->attr_nominal_volts);
  float load_a = attr_read_float(chip->attr_load_current_a);
  float health = attr_read_float(chip->attr_health_percent) / 100.0f;
  float ripple_mv = attr_read_float(chip->attr_ripple_mv);
  float max_current_a = attr_read_float(chip->attr_max_current_a);
  float vmon_fs = attr_read_float(chip->attr_vmon_full_scale);
  float imon_fs = attr_read_float(chip->attr_imon_full_scale);
  float internal_res_mohm = attr_read_float(chip->attr_internal_res_mohm);
  bool power_on = attr_read(chip->attr_power_on) != 0;

  float vout = 0.0f;
  float current = 0.0f;
  bool pgood = false;

  if (power_on) {
    float source_factor = 0.70f + 0.30f * health;      // 70%..100%
    float base_v = nominal_v * source_factor;
    float droop_v = load_a * (internal_res_mohm / 1000.0f);
    float overload = (max_current_a > 0.0f && load_a > max_current_a)
      ? (load_a - max_current_a) / max_current_a
      : 0.0f;
    float overload_penalty = overload > 0.0f ? (0.8f + 2.0f * overload) : 0.0f;

    chip->phase += 0.35f;
    float ripple_v = (ripple_mv / 1000.0f) * sinf(chip->phase);

    vout = base_v - droop_v - overload_penalty + ripple_v;
    if (vout < 0.0f) vout = 0.0f;
    current = load_a;
    pgood = (vout >= 11.0f) && (load_a <= max_current_a) && (health >= 0.55f);
  }

  float vmon_scaled = (vmon_fs > 0.0f) ? (vout / vmon_fs) * 5.0f : 0.0f;
  float imon_scaled = (imon_fs > 0.0f) ? (current / imon_fs) * 5.0f : 0.0f;

  pin_dac_write(chip->pin_vout, clampf(vmon_scaled, 0.0f, 5.0f));
  pin_dac_write(chip->pin_vmon, clampf(vmon_scaled, 0.0f, 5.0f));
  pin_dac_write(chip->pin_imon, clampf(imon_scaled, 0.0f, 5.0f));
  pin_write(chip->pin_pgood, pgood ? HIGH : LOW);
}

static void on_timer(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  update_outputs(chip);
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  chip->pin_vout = pin_init("VOUT", ANALOG);
  chip->pin_gnd = pin_init("GND", INPUT);
  chip->pin_vmon = pin_init("VMON", ANALOG);
  chip->pin_imon = pin_init("IMON", ANALOG);
  chip->pin_pgood = pin_init("PGOOD", OUTPUT);

  chip->attr_nominal_volts = attr_init_float("nominalVolts", 12.0f);
  chip->attr_load_current_a = attr_init_float("loadCurrentA", 4.0f);
  chip->attr_health_percent = attr_init_float("healthPercent", 100.0f);
  chip->attr_ripple_mv = attr_init_float("rippleMv", 40.0f);
  chip->attr_power_on = attr_init("powerOn", 1);
  chip->attr_max_current_a = attr_init_float("maxCurrentA", 16.0f);
  chip->attr_vmon_full_scale = attr_init_float("vmonFullScale", 15.0f);
  chip->attr_imon_full_scale = attr_init_float("imonFullScale", 16.0f);
  chip->attr_internal_res_mohm = attr_init_float("internalResistancemOhm", 60.0f);

  const timer_config_t timer_config = {
    .callback = on_timer,
    .user_data = chip,
  };

  chip->timer = timer_init(&timer_config);
  timer_start(chip->timer, 20000, true);
  update_outputs(chip);
}


