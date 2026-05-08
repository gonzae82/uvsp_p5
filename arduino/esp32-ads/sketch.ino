/*
 * Monitor de Saúde – Fonte 12V 16A  (v3)
 * Hardware: ESP32-S2 + ADS1115 + SSD1306 + DS18B20 + MPU6050
 *
 * Novidades v3:
 *  1. Histerese em todos os alarmes (evita flip-flop no limiar)
 *  2. Pesos configuráveis no health score
 *  3. Grace period pós-partida (evita penalidades transitórias)
 *  4. Detecção explícita de sensor inválido vs defeito de fonte
 *  5. Faixas nominais coerentes para fonte 12 V / 16 A
 *  6. Carga percentual (iFonte / 16 A * 100)
 *  7. Páginas OLED redesenhadas com todas as informações
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <stdio.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ═══════════════════════════════════════════════════════════════════════════
//  CONFIGURAÇÃO CENTRAL — ajuste aqui sem mexer no resto do código
// ═══════════════════════════════════════════════════════════════════════════

// Pinos
#define PIN_SDA         21
#define PIN_SCL         20
#define PIN_LM2596_CC    4
#define PIN_PSU_PGOOD    5
#define PIN_DS18B20      6

// Display
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDR     0x3C

// ── Escalas ADC ─────────────────────────────────────────────────────────────
#define SCALE_PSU_VOLT   (15.0f / 5.0f)   // A0: 0-5 V -> 0-15 V
#define SCALE_PSU_CURR   (16.0f / 5.0f)   // A1: 0-5 V -> 0-16 A
#define SCALE_BUCK_VOLT  (30.0f / 5.0f)   // A2: 0-5 V -> 0-30 V
#define ACS_ZERO_V        1.65f
#define ACS_SENS_VPA      0.040f          // V/A  (ACS758 40 mV/A)
#define MAX_CURRENT_A    16.0f            // corrente maxima da fonte
#define GRAVITY_MS2       9.80665f        // m/s2 — removido da aceleracao para isolar vibracao

// ── Tensao da fonte 12 V / 16 A — calibracao CFTV ──────────────────────────
#define V_GOOD_LO    11.8f
#define V_GOOD_HI    12.6f
#define V_WARN_LO    11.5f
#define V_CRIT_LO    11.2f
#define V_OVER_HI    12.9f

// ── Corrente (~80 % e ~90 % da nominal) ─────────────────────────────────────
#define I_WARN_A     12.8f
#define I_HIGH_A     14.4f

// ── Temperatura (DS18B20 no dissipador / carcaca) ───────────────────────────
#define T_WARN_C     50.0f
#define T_HIGH_C     60.0f
#define T_CRIT_C     70.0f

// ── Vibracao (MPU6050, componente AC apos remover DC da gravidade, m/s2) ───
#define VIB_WARN      0.5f
#define VIB_HIGH      1.2f
#define VIB_CRIT      2.5f

// ── Tendencia de queda de tensao (slopeVps = V/s) ───────────────────────────
#define SLOPE_WARN   -0.03f
#define SLOPE_CRIT   -0.08f

// ── Histerese ────────────────────────────────────────────────────────────────
#define HYST_VOLT     0.10f
#define HYST_CURR     0.30f
#define HYST_TEMP     2.0f
#define HYST_VIB      0.20f
#define HYST_HEALTH   5.0f
#define HYST_SLOPE    0.01f

// ── Pesos do health score ────────────────────────────────────────────────────
#define W_PGOOD      25.0f
#define W_CC         10.0f
#define W_VOLT_CRIT  35.0f
#define W_VOLT_HIGH  22.0f
#define W_VOLT_WARN  10.0f
#define W_VOLT_OVER  10.0f
#define W_CURR_OVER  25.0f
#define W_CURR_HIGH  12.0f
#define W_CURR_WARN   5.0f
#define W_TEMP_CRIT  22.0f
#define W_TEMP_HIGH  12.0f
#define W_TEMP_WARN   5.0f
#define W_VIB_CRIT   14.0f
#define W_VIB_HIGH    7.0f
#define W_VIB_WARN    3.0f
#define W_SLOPE_CRIT  8.0f
#define W_SLOPE_WARN  4.0f

// ── Temporizacao ─────────────────────────────────────────────────────────────
#define PAGE_INTERVAL_MS        3000u
#define READ_INTERVAL_MS         500u
#define DISPLAY_INTERVAL_MS      180u
#define LOG_INTERVAL_MS         2000u
#define DS_ASYNC_DELAY_MS        190u
#define GRACE_PERIOD_MS         8000u
#define THINGSPEAK_INTERVAL_MS 16000u   // envia 1 canal por vez, alternando

// Oversampling ADC
#define ADC_OVERSAMPLE 8

// Debounce digital
#define DEBOUNCE_COUNT 4

// Histórico de tensão para slope
#define VOLT_HIST_SIZE 20

// Wi-Fi
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

// ThingSpeak - Canal 1
const char* TS1_WRITE_KEY = "SVIM5FIQDDX0IHW5";
const long  TS1_CHANNEL_ID = 3347335;

// ThingSpeak - Canal 2
const char* TS2_WRITE_KEY = "RHW4AM00K3BI7OIS";
const long  TS2_CHANNEL_ID = 3347339;

// ═══════════════════════════════════════════════════════════════════════════
//  ESTRUTURAS DE DADOS
// ═══════════════════════════════════════════════════════════════════════════

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;
Adafruit_MPU6050 mpu;
OneWire           oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);

// Buffer circular para oversampling
struct AdcBuffer {
  float   buf[ADC_OVERSAMPLE];
  uint8_t idx  = 0;
  bool    full = false;

  void push(float v) {
    buf[idx] = v;
    idx = (idx + 1) % ADC_OVERSAMPLE;
    if (idx == 0) full = true;
  }

  float mean() const {
    uint8_t n = full ? ADC_OVERSAMPLE : idx;
    if (n == 0) return 0.0f;
    float s = 0;
    for (uint8_t i = 0; i < n; i++) s += buf[i];
    return s / n;
  }

  bool valid(float vmin, float vmax) const {
    if (!full && idx == 0) return false;
    float m = mean();
    return (m >= vmin && m <= vmax);
  }
} adcBuf[4];

// Histórico de tensão para regressão linear
struct VoltHistory {
  float   buf[VOLT_HIST_SIZE];
  uint8_t idx  = 0;
  bool    full = false;

  void push(float v) {
    buf[idx] = v;
    idx = (idx + 1) % VOLT_HIST_SIZE;
    if (idx == 0) full = true;
  }

  float slope() const {
    uint8_t n = full ? VOLT_HIST_SIZE : idx;
    if (n < 4) return 0.0f;

    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (uint8_t i = 0; i < n; i++) {
      float xi = (float)i;
      float yi = buf[(idx - n + i + VOLT_HIST_SIZE) % VOLT_HIST_SIZE];
      sumX  += xi;
      sumY  += yi;
      sumXY += xi * yi;
      sumX2 += xi * xi;
    }

    float d = n * sumX2 - sumX * sumX;
    return (fabsf(d) < 1e-6f) ? 0.0f : (n * sumXY - sumX * sumY) / d;
  }

  float slopeVps() const {
    return slope() * (1000.0f / READ_INTERVAL_MS);
  }
} voltHist;

// Debounce para pinos digitais
struct DebouncePin {
  bool    state   = false;
  uint8_t cnt     = 0;
  bool    pending = false;

  void update(bool raw) {
    if (raw == pending) {
      if (cnt < DEBOUNCE_COUNT) cnt++;
      if (cnt >= DEBOUNCE_COUNT) state = pending;
    } else {
      pending = raw;
      cnt = 1;
    }
  }
} dbPgood, dbCC;

// Histerese
struct Hysteresis {
  bool active = false;

  void updateHigh(float value, float threshold_on, float hyst) {
    if (!active) {
      if (value >= threshold_on) active = true;
    } else {
      if (value <= threshold_on - hyst) active = false;
    }
  }

  void updateLow(float value, float threshold_on, float hyst) {
    if (!active) {
      if (value <= threshold_on) active = true;
    } else {
      if (value >= threshold_on + hyst) active = false;
    }
  }
} hystTempWarn, hystTempHigh, hystTempCrit,
  hystVibWarn,  hystVibHigh,  hystVibCrit,
  hystVoltLow,
  hystVoltOver,
  hystCurrWarn, hystCurrHigh,
  hystSlopeWarn, hystSlopeCrit,
  hystHealth;

// Status dos sensores
struct SensorStatus {
  bool adcOk   = false;
  bool tempOk  = false;
  bool mpuOk   = false;
};

// Leituras consolidadas
struct Leituras {
  float vFonte = 0.0f;
  float iFonte = 0.0f;
  float pFonte = 0.0f;
  float cargaPercent = 0.0f;
  float vBuck = 0.0f;
  float iACS = 0.0f;

  float tempFonte = NAN;
  float tempMpu = NAN;
  float vibracao = 0.0f;
  float slopeVps = 0.0f;

  float health = 0.0f;

  bool lm2596CC = false;
  bool fontePgood = false;
  bool graceActive = true;

  SensorStatus sensors;
};

// ═══════════════════════════════════════════════════════════════════════════
//  ESTADO GLOBAL
// ═══════════════════════════════════════════════════════════════════════════

float    emaVibracao   = 0.0f;
bool     mpuDisponivel = false;
bool     dsDisponivel  = false;
uint32_t bootTime      = 0;

// DS18B20 assíncrono
enum DsState { DS_IDLE, DS_CONVERTING };
DsState  dsState      = DS_IDLE;
uint32_t dsReqTime    = 0;
float    dsCachedTemp = NAN;
bool     dsCachedOk   = false;

// ═══════════════════════════════════════════════════════════════════════════
//  HEALTH SCORE
// ═══════════════════════════════════════════════════════════════════════════

float calcFonteHealth(Leituras &m) {
  bool grace = m.graceActive;
  float score = 100.0f;

  if (!m.fontePgood) score -= W_PGOOD;
  if (m.lm2596CC)    score -= W_CC;

  if      (m.vFonte < V_CRIT_LO) score -= W_VOLT_CRIT;
  else if (m.vFonte < V_WARN_LO) score -= W_VOLT_HIGH;
  else if (m.vFonte < V_GOOD_LO) score -= W_VOLT_WARN;
  else if (m.vFonte > V_OVER_HI) score -= W_VOLT_OVER;
  else if (m.vFonte > V_GOOD_HI) score -= W_VOLT_WARN * 0.5f;

  if      (m.iFonte > MAX_CURRENT_A) score -= W_CURR_OVER;
  else if (m.iFonte > I_HIGH_A)      score -= W_CURR_HIGH;
  else if (m.iFonte > I_WARN_A)      score -= W_CURR_WARN;

  if (m.sensors.tempOk) {
    if      (m.tempFonte > T_CRIT_C) score -= W_TEMP_CRIT;
    else if (m.tempFonte > T_HIGH_C) score -= W_TEMP_HIGH;
    else if (m.tempFonte > T_WARN_C) score -= W_TEMP_WARN;
  }

  if (!grace) {
    if (m.sensors.mpuOk) {
      if      (m.vibracao > VIB_CRIT) score -= W_VIB_CRIT;
      else if (m.vibracao > VIB_HIGH) score -= W_VIB_HIGH;
      else if (m.vibracao > VIB_WARN) score -= W_VIB_WARN;
    }

    if      (m.slopeVps < SLOPE_CRIT) score -= W_SLOPE_CRIT;
    else if (m.slopeVps < SLOPE_WARN) score -= W_SLOPE_WARN;
  }

  score = constrain(score, 0.0f, 100.0f);
  hystHealth.updateLow(score, 60.0f, HYST_HEALTH);
  return score;
}

// ═══════════════════════════════════════════════════════════════════════════
//  STATUS E ALERTAS
// ═══════════════════════════════════════════════════════════════════════════

const char* statusFonte(float score) {
  if (score >= 85.0f) return "BOA";
  if (score >= 60.0f) return "ATENCAO";
  return "FALHA";
}

const char* mensagemAlerta(const Leituras &m) {
  if (!m.sensors.adcOk)                         return "SENSOR ADC FALHOU";
  if (!m.fontePgood && m.vFonte < 1.0f)        return "FONTE DESLIGADA";
  if (!m.fontePgood)                           return "PGOOD EM FALHA";
  if (m.sensors.tempOk && hystTempCrit.active) return "TEMP. MUITO ALTA";
  if (m.sensors.mpuOk && hystVibCrit.active)   return "VIBRACAO CRITICA";
  if (m.vFonte < V_CRIT_LO)                    return "TENSAO MUITO BAIXA";
  if (!m.graceActive && hystSlopeCrit.active)  return "TENSAO EM QUEDA!";
  if (m.lm2596CC)                              return "LM2596 EM CC";
  if (hystCurrHigh.active)                     return "CORRENTE ELEVADA";
  if (!m.sensors.tempOk)                       return "SEM SENSOR TEMP";
  if (!m.sensors.mpuOk)                        return "SEM SENSOR VIB";
  if (m.graceActive)                           return "INICIALIZANDO...";
  return "OPERACAO NORMAL";
}

// ═══════════════════════════════════════════════════════════════════════════
//  WIFI / THINGSPEAK
// ═══════════════════════════════════════════════════════════════════════════

void conectarWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Conectando ao Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Falha ao conectar no Wi-Fi");
  }
}

bool enviarCanal1(const Leituras &m) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String url = "https://api.thingspeak.com/update?api_key=";
  url += TS1_WRITE_KEY;
  url += "&field1=" + String(m.vFonte, 3);
  url += "&field2=" + String(m.iFonte, 3);
  url += "&field3=" + String(m.pFonte, 2);
  url += "&field4=" + String(m.cargaPercent, 1);
  url += "&field5=" + String(m.vBuck, 3);
  url += "&field6=" + String(m.iACS, 3);
  url += "&field7=" + String(m.health, 1);
  url += "&field8=" + String(m.slopeVps, 5);

  http.begin(url);
  int httpCode = http.GET();
  String payload = http.getString();
  http.end();

  Serial.print("ThingSpeak Canal 1 HTTP: ");
  Serial.print(httpCode);
  Serial.print(" | resposta: ");
  Serial.println(payload);

  return (httpCode > 0 && payload != "0");
}

bool enviarCanal2(const Leituras &m) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String url = "https://api.thingspeak.com/update?api_key=";
  url += TS2_WRITE_KEY;
  url += "&field1=" + (m.sensors.tempOk ? String(m.tempFonte, 2) : String(0));
  url += "&field2=" + (m.sensors.mpuOk ? String(m.tempMpu, 2) : String(0));
  url += "&field3=" + String(m.vibracao, 3);
  url += "&field4=" + String(m.fontePgood ? 1 : 0);
  url += "&field5=" + String(m.lm2596CC ? 1 : 0);
  url += "&field6=" + String(m.sensors.adcOk ? 1 : 0);
  url += "&field7=" + String(m.sensors.tempOk ? 1 : 0);
  url += "&field8=" + String(m.sensors.mpuOk ? 1 : 0);

  http.begin(url);
  int httpCode = http.GET();
  String payload = http.getString();
  http.end();

  Serial.print("ThingSpeak Canal 2 HTTP: ");
  Serial.print(httpCode);
  Serial.print(" | resposta: ");
  Serial.println(payload);

  return (httpCode > 0 && payload != "0");
}

// ═══════════════════════════════════════════════════════════════════════════
//  DS18B20 ASSÍNCRONO
// ═══════════════════════════════════════════════════════════════════════════

void dsStartConversion() {
  if (!dsDisponivel) return;
  ds18b20.requestTemperatures();
  dsState = DS_CONVERTING;
  dsReqTime = millis();
}

void dsCheckResult() {
  if (dsState != DS_CONVERTING) return;
  if (millis() - dsReqTime < DS_ASYNC_DELAY_MS) return;

  float t = ds18b20.getTempCByIndex(0);
  dsCachedOk   = (t != DEVICE_DISCONNECTED_C && t > -40.0f && t < 150.0f);
  dsCachedTemp = dsCachedOk ? t : NAN;
  dsState = DS_IDLE;
}

// ═══════════════════════════════════════════════════════════════════════════
//  VIBRAÇÃO (MPU6050)
// ═══════════════════════════════════════════════════════════════════════════

void lerVibracao(Leituras &m) {
  if (!mpuDisponivel) {
    m.sensors.mpuOk = false;
    m.tempMpu = NAN;
    m.vibracao = 0.0f;
    return;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float mod = sqrtf(a.acceleration.x * a.acceleration.x +
                    a.acceleration.y * a.acceleration.y +
                    a.acceleration.z * a.acceleration.z);

  float delta = fabsf(mod - GRAVITY_MS2);
  emaVibracao = emaVibracao * 0.80f + delta * 0.20f;

  m.sensors.mpuOk = (mod > 0.1f && mod < 50.0f);
  m.tempMpu = temp.temperature;
  m.vibracao = emaVibracao;

  hystVibWarn.updateHigh(emaVibracao, VIB_WARN, HYST_VIB);
  hystVibHigh.updateHigh(emaVibracao, VIB_HIGH, HYST_VIB);
  hystVibCrit.updateHigh(emaVibracao, VIB_CRIT, HYST_VIB);
}

// ═══════════════════════════════════════════════════════════════════════════
//  LEITURA PRINCIPAL DOS SENSORES
// ═══════════════════════════════════════════════════════════════════════════

void lerSensores(Leituras &m) {
  uint32_t now = millis();
  m.graceActive = (now - bootTime < GRACE_PERIOD_MS);

  for (uint8_t ch = 0; ch < 4; ch++) {
    adcBuf[ch].push(ads.computeVolts(ads.readADC_SingleEnded(ch)));
  }

  m.sensors.adcOk =
      adcBuf[0].valid(0.0f, 5.2f) &&
      adcBuf[1].valid(0.0f, 5.2f) &&
      adcBuf[2].valid(0.0f, 5.2f) &&
      adcBuf[3].valid(0.0f, 3.4f);

  m.vFonte       = adcBuf[0].mean() * SCALE_PSU_VOLT;
  m.iFonte       = adcBuf[1].mean() * SCALE_PSU_CURR;
  m.vBuck        = adcBuf[2].mean() * SCALE_BUCK_VOLT;
  m.iACS         = max(0.0f, (adcBuf[3].mean() - ACS_ZERO_V) / ACS_SENS_VPA);
  m.pFonte       = m.vFonte * m.iFonte;
  m.cargaPercent = (m.iFonte / MAX_CURRENT_A) * 100.0f;

  if (m.sensors.adcOk) voltHist.push(m.vFonte);
  m.slopeVps = m.graceActive ? 0.0f : voltHist.slopeVps();

  dbPgood.update(digitalRead(PIN_PSU_PGOOD));
  dbCC.update(digitalRead(PIN_LM2596_CC));
  m.fontePgood = dbPgood.state;
  m.lm2596CC   = dbCC.state;

  dsCheckResult();
  m.sensors.tempOk = dsCachedOk;
  m.tempFonte      = dsCachedTemp;
  if (dsState == DS_IDLE) dsStartConversion();

  if (m.sensors.tempOk) {
    hystTempWarn.updateHigh(m.tempFonte, T_WARN_C, HYST_TEMP);
    hystTempHigh.updateHigh(m.tempFonte, T_HIGH_C, HYST_TEMP);
    hystTempCrit.updateHigh(m.tempFonte, T_CRIT_C, HYST_TEMP);
  }

  hystVoltLow.updateLow(m.vFonte, V_GOOD_LO, HYST_VOLT);
  hystVoltOver.updateHigh(m.vFonte, V_GOOD_HI, HYST_VOLT);
  hystCurrWarn.updateHigh(m.iFonte, I_WARN_A, HYST_CURR);
  hystCurrHigh.updateHigh(m.iFonte, I_HIGH_A, HYST_CURR);

  if (!m.graceActive) {
    hystSlopeWarn.updateLow(m.slopeVps, SLOPE_WARN, HYST_SLOPE);
    hystSlopeCrit.updateLow(m.slopeVps, SLOPE_CRIT, HYST_SLOPE);
  }

  lerVibracao(m);
  m.health = calcFonteHealth(m);
}

// ═══════════════════════════════════════════════════════════════════════════
//  HELPERS DE DISPLAY
// ═══════════════════════════════════════════════════════════════════════════

void drawHeader(const char *titulo) {
  display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 1);
  display.print(titulo);
  display.setTextColor(SSD1306_WHITE);
}

void drawHealthBar(float health, int x, int y, int w, int h) {
  int fill = constrain((int)roundf((health / 100.0f) * (w - 2)), 0, w - 2);
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
}

void drawVoltTrend(float slopeVps, int x, int y) {
  display.setCursor(x, y);
  if      (hystSlopeCrit.active) display.print('!');
  else if (slopeVps >  0.005f)   display.print('^');
  else if (slopeVps < -0.005f)   display.print('v');
  else                           display.print('-');
}

// ═══════════════════════════════════════════════════════════════════════════
//  PÁGINAS OLED
// ═══════════════════════════════════════════════════════════════════════════

void drawPageHealth(const Leituras &m, bool blink) {
  char buf[24];
  drawHeader("SAUDE DA FONTE 12V");

  display.setTextSize(2);
  display.setCursor(2, 13);
  snprintf(buf, sizeof(buf), "%3.0f%%", m.health);
  display.print(buf);

  display.setTextSize(1);

  display.setCursor(84, 13);
  display.print(statusFonte(m.health));

  display.setCursor(84, 24);
  display.print("Carga:");

  display.setCursor(84, 33);
  snprintf(buf, sizeof(buf), "%.0f%%", m.cargaPercent);
  display.print(buf);

  display.setCursor(2, 38);
  display.print("V:");
  display.print(m.vFonte, 2);
  display.print("V ");
  drawVoltTrend(m.slopeVps, 84, 38);

  display.setCursor(2, 48);
  display.print("I:");
  display.print(m.iFonte, 2);
  display.print("A");

  drawHealthBar(m.health, 2, 57, 124, 6);

  if (hystHealth.active && blink) {
    display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  }
}

void drawPageElectrical(const Leituras &m) {
  drawHeader("MEDICOES ELETRICAS");
  display.setTextSize(1);

  display.setCursor(2, 13);  display.print("V:");     display.print(m.vFonte, 3); display.print("V");
  display.setCursor(2, 23);  display.print("I:");     display.print(m.iFonte, 3); display.print("A");
  display.setCursor(2, 33);  display.print("P:");     display.print(m.pFonte, 2); display.print("W");
  display.setCursor(2, 43);  display.print("Vbuck:"); display.print(m.vBuck, 2);  display.print("V");
  display.setCursor(2, 53);  display.print("Iacs: "); display.print(m.iACS, 3);   display.print("A");

  char buf[10];
  display.setCursor(92, 13);
  snprintf(buf, sizeof(buf), "%.0f%%", m.cargaPercent);
  display.print(buf);
  drawVoltTrend(m.slopeVps, 110, 23);
}

void drawPageAmbiente(const Leituras &m, bool blink) {
  drawHeader("AMBIENTE");
  display.setTextSize(1);

  display.setCursor(2, 13);
  display.print("TmpFonte:");
  if (m.sensors.tempOk) {
    display.print(m.tempFonte, 1);
    display.print("C");
  } else {
    if (blink) display.print("---FAIL");
    else       display.print("SEM SENSOR");
  }

  display.setCursor(2, 25);
  display.print("TmpMPU  :");
  if (m.sensors.mpuOk) {
    display.print(m.tempMpu, 1);
    display.print("C");
  } else {
    display.print("SEM SENSOR");
  }

  display.setCursor(2, 37);
  display.print("Vib     :");
  display.print(m.vibracao, 3);
  display.print("m/s2");

  display.setCursor(2, 49);
  display.print("Slope   :");
  char buf[12];
  snprintf(buf, sizeof(buf), "%+.4f", m.slopeVps);
  display.print(buf);
  display.print("V/s");
}

void drawPageAlarmes(const Leituras &m, bool blink) {
  drawHeader("STATUS E ALARMES");
  display.setTextSize(1);

  display.setCursor(2, 13);
  display.print("PGOOD  : ");
  display.println(m.fontePgood ? "OK" : "FALHA");

  display.setCursor(2, 24);
  display.print("LM2596 : ");
  display.println(m.lm2596CC ? "LIMITE CC" : "CV NORMAL");

  display.setCursor(2, 35);
  display.print("Sensor T: ");
  display.println(m.sensors.tempOk ? "OK" : "FAIL");

  display.setCursor(2, 46);
  display.print("Sensor V: ");
  display.println(m.sensors.mpuOk ? "OK" : "FAIL");

  display.setCursor(2, 57);
  bool alertaAtivo = (!m.fontePgood || m.lm2596CC ||
                      !m.sensors.tempOk || !m.sensors.mpuOk ||
                      hystHealth.active);
  if (alertaAtivo && blink) display.print(">>> ");
  display.print(mensagemAlerta(m));
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOG CSV SERIAL
// ═══════════════════════════════════════════════════════════════════════════

void printCSVHeader() {
  Serial.println(F("ms,vFonte,iFonte,pFonte,carga%,vBuck,iACS,tempFonte,tempMPU,vibracao,slopeVps,health,pgood,cc,adcOk,tempOk,mpuOk,grace"));
}

void printCSV(const Leituras &m) {
  Serial.print(millis());                  Serial.print(',');
  Serial.print(m.vFonte, 3);              Serial.print(',');
  Serial.print(m.iFonte, 3);              Serial.print(',');
  Serial.print(m.pFonte, 2);              Serial.print(',');
  Serial.print(m.cargaPercent, 1);        Serial.print(',');
  Serial.print(m.vBuck, 3);               Serial.print(',');
  Serial.print(m.iACS, 3);                Serial.print(',');
  if (m.sensors.tempOk) Serial.print(m.tempFonte, 2); else Serial.print("NA");
  Serial.print(',');
  if (m.sensors.mpuOk)  Serial.print(m.tempMpu, 2);   else Serial.print("NA");
  Serial.print(',');
  Serial.print(m.vibracao, 3);            Serial.print(',');
  Serial.print(m.slopeVps, 5);            Serial.print(',');
  Serial.print(m.health, 1);              Serial.print(',');
  Serial.print(m.fontePgood ? 1 : 0);     Serial.print(',');
  Serial.print(m.lm2596CC ? 1 : 0);       Serial.print(',');
  Serial.print(m.sensors.adcOk ? 1 : 0);  Serial.print(',');
  Serial.print(m.sensors.tempOk ? 1 : 0); Serial.print(',');
  Serial.print(m.sensors.mpuOk ? 1 : 0);  Serial.print(',');
  Serial.println(m.graceActive ? 1 : 0);
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(300);
  printCSVHeader();

  pinMode(PIN_LM2596_CC, INPUT);
  pinMode(PIN_PSU_PGOOD, INPUT);
  Wire.begin(PIN_SDA, PIN_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("ERRO: OLED nao encontrado"));
    while (1) {}
  }

  display.clearDisplay();
  drawHeader("MONITOR v3");
  display.setTextSize(1);
  display.setCursor(2, 14);
  display.println("Inicializando...");
  display.display();

  if (!ads.begin(0x48, &Wire)) {
    display.setCursor(2, 26);
    display.println("ADS1115: FAIL");
    display.display();
    Serial.println(F("ERRO: ADS1115"));
    while (1) {}
  }

  ads.setGain(GAIN_TWOTHIRDS);
  display.setCursor(2, 26);
  display.println("ADS1115: OK");
  display.display();

  ds18b20.begin();
  ds18b20.setResolution(10);
  ds18b20.setWaitForConversion(false);
  dsDisponivel = (ds18b20.getDeviceCount() > 0);

  display.setCursor(2, 38);
  display.print("DS18B20: ");
  display.println(dsDisponivel ? "OK" : "FAIL");
  display.display();

  mpuDisponivel = mpu.begin(0x68, &Wire);
  if (mpuDisponivel) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  display.setCursor(2, 50);
  display.print("MPU6050: ");
  display.println(mpuDisponivel ? "OK" : "FAIL");
  display.display();

  if (dsDisponivel) dsStartConversion();

  bootTime = millis();
  delay(1200);

  conectarWiFi();
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  static uint32_t lastRead       = 0;
  static uint32_t lastDisplay    = 0;
  static uint32_t lastLog        = 0;
  static uint32_t lastPageChange = 0;
  static uint32_t lastThingSpeak = 0;
  static uint8_t  page           = 0;
  static bool     sendChannel1Next = true;
  static Leituras m = {};

  uint32_t now = millis();

  if (now - lastRead >= READ_INTERVAL_MS) {
    lastRead = now;
    lerSensores(m);
  }

  if (now - lastLog >= LOG_INTERVAL_MS) {
    lastLog = now;
    printCSV(m);
  }

  if (now - lastDisplay >= DISPLAY_INTERVAL_MS) {
    lastDisplay = now;

    if (now - lastPageChange >= PAGE_INTERVAL_MS) {
      lastPageChange = now;
      page = (page + 1) % 4;
    }

    bool blink = ((now / 350) % 2) == 0;

    display.clearDisplay();
    switch (page) {
      case 0: drawPageHealth(m, blink);     break;
      case 1: drawPageElectrical(m);        break;
      case 2: drawPageAmbiente(m, blink);   break;
      default: drawPageAlarmes(m, blink);   break;
    }
    display.display();
  }

  if (now - lastThingSpeak >= THINGSPEAK_INTERVAL_MS) {
    lastThingSpeak = now;

    if (WiFi.status() != WL_CONNECTED) {
      conectarWiFi();
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (sendChannel1Next) {
        bool ok1 = enviarCanal1(m);
        Serial.print("Envio Canal 1: ");
        Serial.println(ok1 ? "OK" : "FALHOU");
      } else {
        bool ok2 = enviarCanal2(m);
        Serial.print("Envio Canal 2: ");
        Serial.println(ok2 ? "OK" : "FALHOU");
      }

      sendChannel1Next = !sendChannel1Next;
    }
  }
}