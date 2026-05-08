#include "wokwi-api.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  pin_t pin_vin;
  pin_t pin_gnd;
  pin_t pin_vout;
  pin_t pin_vmon;
  pin_t pin_imon;
  pin_t pin_cc;
  pin_t pin_src;   /* HIGH = VIN vem do pino; LOW = fallback atributo */

  uint32_t attr_vin_volts;
  uint32_t attr_vset_volts;
  uint32_t attr_load_current_a;
  uint32_t attr_current_limit_a;
  uint32_t attr_power_on;
  uint32_t attr_max_display_volts;
  uint32_t attr_max_display_current;
  uint32_t attr_dropout_volts;

  timer_t timer;

  float actual_vout;
  float actual_current;
  bool cc_mode;
  bool vin_from_pin;   /* true = usando pino real; false = fallback para atributo */
} chip_state_t;

static float clampf(float x, float minv, float maxv) {
  if (x < minv) return minv;
  if (x > maxv) return maxv;
  return x;
}

static void update_outputs(chip_state_t *chip) {
  /* Lê VIN do pino analógico (conexão real chip4->chip3).
     O Wokwi escala as saídas DAC do chip da fonte para 0-5 V
     representando 0-vmonFullScale. Revertemos a escala aqui.
     Se o pino não estiver conectado ou leitura for < 0.05 V,
     usa o atributo vinVolts como fallback. */
  float vin_pin_v = pin_adc_read(chip->pin_vin);
  float attr_vin  = attr_read_float(chip->attr_vin_volts);
  float vmon_fs   = 15.0f;   /* deve bater com vmonFullScale do chip da fonte */
  chip->vin_from_pin = (vin_pin_v > 0.05f);
  float vin = chip->vin_from_pin ? (vin_pin_v / 5.0f) * vmon_fs : attr_vin;
  float vset = attr_read_float(chip->attr_vset_volts);
  float load_a = attr_read_float(chip->attr_load_current_a);
  float ilim_a = attr_read_float(chip->attr_current_limit_a);
  float max_disp_v = attr_read_float(chip->attr_max_display_volts);
  float max_disp_i = attr_read_float(chip->attr_max_display_current);
  float dropout = attr_read_float(chip->attr_dropout_volts);
  bool power_on = attr_read(chip->attr_power_on) != 0;

  chip->actual_vout = 0.0f;
  chip->actual_current = 0.0f;
  chip->cc_mode = false;

  if (power_on && vin > dropout) {
    float max_possible_vout = vin - dropout;
    float target_vout = (vset < max_possible_vout) ? vset : max_possible_vout;

    if (load_a <= 0.0001f) {
      chip->actual_vout = target_vout;
      chip->actual_current = 0.0f;
      chip->cc_mode = false;
    } else if (load_a > ilim_a && ilim_a > 0.0f) {
      chip->cc_mode = true;
      chip->actual_current = ilim_a;
      chip->actual_vout = target_vout * (ilim_a / load_a);
    } else {
      chip->cc_mode = false;
      chip->actual_vout = target_vout;
      chip->actual_current = load_a;
    }
  }

  chip->actual_vout = clampf(chip->actual_vout, 0.0f, max_disp_v);
  chip->actual_current = clampf(chip->actual_current, 0.0f, max_disp_i);

  // Saídas analógicas escaladas para 0..5V do Wokwi
  float vout_scaled = (max_disp_v > 0.0f) ? (chip->actual_vout / max_disp_v) * 5.0f : 0.0f;
  float imon_scaled = (max_disp_i > 0.0f) ? (chip->actual_current / max_disp_i) * 5.0f : 0.0f;

  vout_scaled = clampf(vout_scaled, 0.0f, 5.0f);
  imon_scaled = clampf(imon_scaled, 0.0f, 5.0f);

  pin_dac_write(chip->pin_vout, vout_scaled);
  pin_dac_write(chip->pin_vmon, vout_scaled);
  pin_dac_write(chip->pin_imon, imon_scaled);
  pin_write(chip->pin_cc,  chip->cc_mode      ? HIGH : LOW);
  pin_write(chip->pin_src, chip->vin_from_pin ? HIGH : LOW);
}

static void on_timer(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  update_outputs(chip);
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  chip->pin_vin  = pin_init("VIN", ANALOG);  /* leitura analógica real */
  chip->pin_gnd  = pin_init("GND", INPUT);
  chip->pin_vout = pin_init("VOUT", ANALOG);
  chip->pin_vmon = pin_init("VMON", ANALOG);
  chip->pin_imon = pin_init("IMON", ANALOG);
  chip->pin_cc   = pin_init("CC",  OUTPUT);
  chip->pin_src  = pin_init("SRC", OUTPUT);  /* HIGH=pino real, LOW=atributo */

  chip->attr_vin_volts        = attr_init_float("vinVolts", 12.0f);
  chip->attr_vset_volts       = attr_init_float("vsetVolts", 5.0f);
  chip->attr_load_current_a   = attr_init_float("loadCurrentA", 1.0f);
  chip->attr_current_limit_a  = attr_init_float("currentLimitA", 2.0f);
  chip->attr_power_on         = attr_init("powerOn", 1);

  chip->attr_max_display_volts   = attr_init_float("maxDisplayVolts", 30.0f);
  chip->attr_max_display_current = attr_init_float("maxDisplayCurrent", 10.0f);
  chip->attr_dropout_volts       = attr_init_float("dropoutVolts", 1.2f);

  const timer_config_t timer_config = {
    .callback = on_timer,
    .user_data = chip,
  };

  chip->timer = timer_init(&timer_config);
  timer_start(chip->timer, 10000, true); // 10ms repetitivo

  update_outputs(chip);
}

