#include "wokwi-api.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define I2C_ADDRESS 0x48

#define REG_CONVERSION 0x00
#define REG_CONFIG     0x01
#define REG_LO_THRESH  0x02
#define REG_HI_THRESH  0x03

typedef struct {
  i2c_dev_t i2c_dev;
  i2c_config_t i2c_config;

  pin_t pin_a0;
  pin_t pin_a1;
  pin_t pin_a2;
  pin_t pin_a3;
  pin_t pin_endereco;
  pin_t pin_alrt;

  uint16_t regs[4];
  uint8_t reg_ptr;
  uint8_t read_index;
  uint8_t write_phase;
  uint8_t write_msb;
} chip_state_t;

static bool on_i2c_connect(void *user_data, uint32_t address, bool read);
static uint8_t on_i2c_read(void *user_data);
static bool on_i2c_write(void *user_data, uint8_t data);
static void on_i2c_disconnect(void *user_data);

static float fsr_from_config(uint16_t config);
static float read_mux_voltage(chip_state_t *chip, uint8_t mux);
static void update_conversion_register(chip_state_t *chip);
static int16_t voltage_to_code(float volts, float fsr, bool single_ended);

static float fsr_from_config(uint16_t config) {
  uint8_t pga = (config >> 9) & 0x07;

  switch (pga) {
    case 0x00: return 6.144f;
    case 0x01: return 4.096f;
    case 0x02: return 2.048f;
    case 0x03: return 1.024f;
    case 0x04: return 0.512f;
    case 0x05:
    case 0x06:
    case 0x07:
    default:   return 0.256f;
  }
}

static float read_mux_voltage(chip_state_t *chip, uint8_t mux) {
  float v0 = pin_adc_read(chip->pin_a0);
  float v1 = pin_adc_read(chip->pin_a1);
  float v2 = pin_adc_read(chip->pin_a2);
  float v3 = pin_adc_read(chip->pin_a3);

  switch (mux) {
    case 0x00: return v0 - v1; // AIN0 - AIN1
    case 0x01: return v0 - v3; // AIN0 - AIN3
    case 0x02: return v1 - v3; // AIN1 - AIN3
    case 0x03: return v2 - v3; // AIN2 - AIN3
    case 0x04: return v0;      // AIN0 - GND
    case 0x05: return v1;      // AIN1 - GND
    case 0x06: return v2;      // AIN2 - GND
    case 0x07: return v3;      // AIN3 - GND
    default:   return 0.0f;
  }
}

static int16_t voltage_to_code(float volts, float fsr, bool single_ended) {
  float scaled;
  int32_t code;

  if (single_ended) {
    if (volts < 0.0f) volts = 0.0f;
    if (volts > fsr)  volts = fsr;
    scaled = (volts / fsr) * 32767.0f;
  } else {
    if (volts > fsr)  volts = fsr;
    if (volts < -fsr) volts = -fsr;
    scaled = (volts / fsr) * 32768.0f;
  }

  code = (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));

  if (code > 32767)  code = 32767;
  if (code < -32768) code = -32768;

  return (int16_t)code;
}

static void update_conversion_register(chip_state_t *chip) {
  uint16_t config = chip->regs[REG_CONFIG];
  uint8_t mux = (config >> 12) & 0x07;
  bool single_ended = (mux >= 0x04);
  float fsr = fsr_from_config(config);
  float volts = read_mux_voltage(chip, mux);
  int16_t code = voltage_to_code(volts, fsr, single_ended);

  chip->regs[REG_CONVERSION] = (uint16_t)code;

  // marca conversion ready
  chip->regs[REG_CONFIG] |= 0x8000;
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  chip->pin_a0 = pin_init("A0", ANALOG);
  chip->pin_a1 = pin_init("A1", ANALOG);
  chip->pin_a2 = pin_init("A2", ANALOG);
  chip->pin_a3 = pin_init("A3", ANALOG);

  chip->pin_endereco = pin_init("ENDERECO", INPUT);
  chip->pin_alrt     = pin_init("ALRT", OUTPUT);

  chip->i2c_config.address = I2C_ADDRESS;
  chip->i2c_config.scl = pin_init("SCL", INPUT);
  chip->i2c_config.sda = pin_init("SDA", INPUT);
  chip->i2c_config.connect = on_i2c_connect;
  chip->i2c_config.read = on_i2c_read;
  chip->i2c_config.write = on_i2c_write;
  chip->i2c_config.disconnect = on_i2c_disconnect;
  chip->i2c_config.user_data = chip;
  chip->i2c_dev = i2c_init(&chip->i2c_config);

  chip->regs[REG_CONVERSION] = 0x0000;
  chip->regs[REG_CONFIG]     = 0x8583;
  chip->regs[REG_LO_THRESH]  = 0x8000;
  chip->regs[REG_HI_THRESH]  = 0x7FFF;

  chip->reg_ptr = REG_CONVERSION;
  chip->read_index = 0;
  chip->write_phase = 0;

  update_conversion_register(chip);
}

static bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  chip_state_t *chip = (chip_state_t *)user_data;

  if (address != I2C_ADDRESS) {
    return false;
  }

  chip->read_index = 0;

  if (read && chip->reg_ptr == REG_CONVERSION) {
    update_conversion_register(chip);
  }

  return true;
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  uint16_t value = chip->regs[chip->reg_ptr & 0x03];

  uint8_t result;
  if (chip->read_index == 0) {
    result = (uint8_t)((value >> 8) & 0xFF);
  } else {
    result = (uint8_t)(value & 0xFF);
  }

  chip->read_index = (chip->read_index + 1) & 0x01;
  return result;
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  if (chip->write_phase == 0) {
    chip->reg_ptr = data & 0x03;
    chip->write_phase = 1;
    return true;
  }

  if (chip->write_phase == 1) {
    chip->write_msb = data;
    chip->write_phase = 2;
    return true;
  }

  chip->regs[chip->reg_ptr & 0x03] = ((uint16_t)chip->write_msb << 8) | data;

  if ((chip->reg_ptr & 0x03) == REG_CONFIG) {
    update_conversion_register(chip);
  }

  chip->write_phase = 0;
  return true;
}

static void on_i2c_disconnect(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->write_phase = 0;
  chip->read_index = 0;
}