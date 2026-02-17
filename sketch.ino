// ════════════════════════════════════════════════════════════════
//  ESP32 AquaMonitor — sketch.ino
//
//  Works in TWO modes — toggle with the #define below:
//
//  MODE 1: SIMULATION (default)
//    → Potentiometers on GPIO34/35/32 simulate sensors
//    → Use this in Wokwi or on a breadboard with pots
//
//  MODE 2: PHYSICAL (uncomment #define PHYSICAL_MODE)
//    → Real pH module on GPIO34
//    → Real TDS module on GPIO35
//    → Real turbidity sensor on GPIO32
//    → Real DS18B20 probe on GPIO4
//    → Just uncomment one line and re-upload — done!
//
//  PIN SUMMARY:
//    GPIO4   DS18B20 data (4.7kΩ pull-up to 3.3V required)
//    GPIO34  pH sensor analog output      (input-only ADC)
//    GPIO35  TDS sensor analog output     (input-only ADC)
//    GPIO32  Turbidity sensor analog out  (ADC)
//    GPIO13  GREEN LED  (SAFE)
//    GPIO12  YELLOW LED (MARGINAL)
//    GPIO14  RED LED    (UNSAFE)
//
//  SERIAL: 115200 baud
// ════════════════════════════════════════════════════════════════

// ── TOGGLE THIS LINE to switch from simulation to physical ──────
// #define PHYSICAL_MODE
// ────────────────────────────────────────────────────────────────

#include <OneWire.h>
#include <DallasTemperature.h>

// ── Pin Definitions ─────────────────────────────────────────────
#define PH_PIN          34
#define TDS_PIN         35
#define TURBIDITY_PIN   32
#define TEMP_PIN         4
#define LED_SAFE        13
#define LED_WARN        12
#define LED_BAD         14

// ── Calibration (Physical mode only) ────────────────────────────
// After calibrating with buffer solutions, update these values:
#define PH_NEUTRAL_VOLTAGE  1500.0   // mV when probe is in pH 7.00 buffer
#define PH_ACID_VOLTAGE     2032.0   // mV when probe is in pH 4.00 buffer
#define VREF                3300.0   // mV — ESP32 ADC reference

// ── WHO Safe Limits ─────────────────────────────────────────────
#define PH_MIN    6.5
#define PH_MAX    8.5
#define TDS_MIN   50.0
#define TDS_MAX   500.0
#define TURB_MAX  4.0
#define TEMP_MAX  35.0
#define TEMP_MIN  5.0

// ── DS18B20 setup ────────────────────────────────────────────────
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);
float gTemperature = 25.0;  // global, used for TDS compensation

// ════════════════════════════════════════════════════════════════
//  SIMULATION MODE HELPERS
//  In simulation, pots give raw 0–4095. We map them to
//  realistic sensor ranges so the output looks right.
// ════════════════════════════════════════════════════════════════
#ifndef PHYSICAL_MODE

float simReadPH() {
  int raw = analogRead(PH_PIN);
  return (raw / 4095.0) * 14.0;          // 0–14 pH
}

float simReadTDS() {
  int raw = analogRead(TDS_PIN);
  return (raw / 4095.0) * 1000.0;        // 0–1000 ppm
}

float simReadTurbidity() {
  int raw = analogRead(TURBIDITY_PIN);
  return (raw / 4095.0) * 3000.0;        // 0–3000 NTU
}

float simReadTemperature() {
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C || t < -100) return 25.0;
  return t;
}

#endif

// ════════════════════════════════════════════════════════════════
//  PHYSICAL MODE SENSORS
//  Real calibrated readings from actual sensor modules
// ════════════════════════════════════════════════════════════════
#ifdef PHYSICAL_MODE

float physReadTemperature() {
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C || t == -127.0) {
    Serial.println("[WARN] DS18B20 not detected — defaulting to 25C");
    return 25.0;
  }
  return t;
}

float physReadPH() {
  // Average 10 samples to reduce noise
  long sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(PH_PIN); delay(10); }
  float voltage = (sum / 10.0) * VREF / 4095.0;  // mV
  // 2-point calibration formula
  float slope = (7.0 - 4.0) / (PH_NEUTRAL_VOLTAGE - PH_ACID_VOLTAGE);
  float ph = 7.0 + slope * (PH_NEUTRAL_VOLTAGE - voltage);
  return constrain(ph, 0.0, 14.0);
}

float physReadTDS() {
  // Collect 30 samples, average
  long sum = 0;
  for (int i = 0; i < 30; i++) { sum += analogRead(TDS_PIN); delay(10); }
  float avgVoltage = (sum / 30.0) * VREF / 4095.0;
  // Temperature compensation (0.02 per °C from 25°C)
  float compCoeff = 1.0 + 0.02 * (gTemperature - 25.0);
  float compVoltage = avgVoltage / compCoeff;
  // Empirical polynomial for this sensor type
  float tds = (133.42 * pow(compVoltage/1000.0, 3)
             - 255.86 * pow(compVoltage/1000.0, 2)
             + 857.39 * (compVoltage/1000.0)) * 0.5;
  return constrain(tds, 0.0, 2000.0);
}

float physReadTurbidity() {
  long sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(TURBIDITY_PIN); delay(10); }
  float voltage = (sum / 10.0) * VREF / 4095.0;  // mV
  // Turbidity is INVERTED: high voltage = clear water
  // Empirical formula — adjust if your sensor differs
  float ntu;
  if (voltage >= 3200.0) {
    ntu = 0.0;
  } else {
    float v = voltage / 1000.0;  // convert to V for formula
    ntu = -1120.4 * v * v + 5742.3 * v - 4353.8;
  }
  return constrain(ntu, 0.0, 3000.0);
}

#endif

// ════════════════════════════════════════════════════════════════
//  UNIFIED READ FUNCTIONS
//  Calls either sim or physical depending on #define
// ════════════════════════════════════════════════════════════════
float readTemperature() {
#ifdef PHYSICAL_MODE
  return physReadTemperature();
#else
  return simReadTemperature();
#endif
}

float readPH()       {
#ifdef PHYSICAL_MODE
  return physReadPH();
#else
  return simReadPH();
#endif
}

float readTDS()      {
#ifdef PHYSICAL_MODE
  return physReadTDS();
#else
  return simReadTDS();
#endif
}

float readTurbidity(){
#ifdef PHYSICAL_MODE
  return physReadTurbidity();
#else
  return simReadTurbidity();
#endif
}

// ════════════════════════════════════════════════════════════════
//  ASSESSMENT
// ════════════════════════════════════════════════════════════════
int calcWQI(float ph, float tds, float turb, float temp) {
  int score = 100;
  if (ph < PH_MIN || ph > PH_MAX)         score -= 25;
  else if (ph < 7.0 || ph > 8.0)          score -= 8;
  if (tds > TDS_MAX)                       score -= 25;
  else if (tds > 300)                      score -= 8;
  if (turb > TURB_MAX)                     score -= 30;
  else if (turb > 2)                       score -= 10;
  if (temp > TEMP_MAX || temp < TEMP_MIN)  score -= 20;
  return max(0, score);
}

int countIssues(float ph, float tds, float turb, float temp) {
  int n = 0;
  if (ph < PH_MIN || ph > PH_MAX)          n++;
  if (tds < TDS_MIN || tds > TDS_MAX)      n++;
  if (turb > TURB_MAX)                     n++;
  if (temp > TEMP_MAX || temp < TEMP_MIN)  n++;
  return n;
}

void setLEDs(int issues) {
  digitalWrite(LED_SAFE, issues == 0 ? HIGH : LOW);
  digitalWrite(LED_WARN, issues == 1 ? HIGH : LOW);
  digitalWrite(LED_BAD,  issues >= 2 ? HIGH : LOW);
}

// ════════════════════════════════════════════════════════════════
//  SERIAL REPORT  (same format in both modes)
// ════════════════════════════════════════════════════════════════
void printReport(float ph, float tds, float turb, float temp) {
  int wqi    = calcWQI(ph, tds, turb, temp);
  int issues = countIssues(ph, tds, turb, temp);

#ifdef PHYSICAL_MODE
  String modeStr = "PHYSICAL";
#else
  String modeStr = "SIMULATION";
#endif

  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════╗");
  Serial.print  ("║  ESP32 AquaMonitor — ");
  Serial.print  (modeStr);
  Serial.println(" MODE                   ║");
  Serial.println("╠══════════════════════════════════════════════════╣");

  // Temperature
  Serial.print("║  Temperature : "); Serial.print(temp, 1); Serial.print(" C  → ");
  if      (temp > TEMP_MAX) Serial.println("HOT [bacterial risk]");
  else if (temp < TEMP_MIN) Serial.println("COLD");
  else                      Serial.println("NORMAL");

  // pH
  Serial.print("║  pH Level    : "); Serial.print(ph, 2); Serial.print("    → ");
  if      (ph < PH_MIN)    Serial.println("ACIDIC [below 6.5]");
  else if (ph > PH_MAX)    Serial.println("ALKALINE [above 8.5]");
  else                     Serial.println("NORMAL");

  // TDS
  Serial.print("║  TDS         : "); Serial.print(tds, 0); Serial.print(" ppm → ");
  if      (tds < TDS_MIN)  Serial.println("TOO LOW [lacks minerals]");
  else if (tds > TDS_MAX)  Serial.println("HIGH [possible contaminants]");
  else                     Serial.println("NORMAL");

  // Turbidity
  Serial.print("║  Turbidity   : "); Serial.print(turb, 1); Serial.print(" NTU → ");
  if      (turb > TURB_MAX) Serial.println("CLOUDY [not safe]");
  else if (turb > 2.0)      Serial.println("SLIGHT HAZE");
  else                      Serial.println("CLEAR");

  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.print  ("║  WQI Score   : "); Serial.print(wqi); Serial.println(" / 100");
  Serial.print  ("║  STATUS      : ");
  if      (issues == 0)  Serial.println("✓ SAFE TO DRINK");
  else if (issues == 1)  Serial.println("⚠ MARGINAL — treat before use");
  else                   Serial.println("✗ UNSAFE — do not drink");
  Serial.println("╚══════════════════════════════════════════════════╝");
}

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);        // ESP32 12-bit ADC
  analogSetAttenuation(ADC_11db);  // Full 0–3.3V range

  pinMode(LED_SAFE, OUTPUT);
  pinMode(LED_WARN, OUTPUT);
  pinMode(LED_BAD,  OUTPUT);

  // Quick LED test on boot
  digitalWrite(LED_SAFE, HIGH); delay(300);
  digitalWrite(LED_WARN, HIGH); delay(300);
  digitalWrite(LED_BAD,  HIGH); delay(300);
  digitalWrite(LED_SAFE, LOW);
  digitalWrite(LED_WARN, LOW);
  digitalWrite(LED_BAD,  LOW);

  tempSensor.begin();

  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════╗");
  Serial.println("║       ESP32 AquaMonitor — Booting...            ║");
  Serial.println("╠══════════════════════════════════════════════════╣");

#ifdef PHYSICAL_MODE
  Serial.println("║  MODE: PHYSICAL (real sensors)                  ║");
#else
  Serial.println("║  MODE: SIMULATION (potentiometers)              ║");
  Serial.println("║  To switch: uncomment #define PHYSICAL_MODE     ║");
#endif

  Serial.print  ("║  DS18B20 found: ");
  Serial.println(tempSensor.getDeviceCount());
  Serial.println("║  GPIO34=pH  GPIO35=TDS  GPIO32=Turb  GPIO4=Temp ║");
  Serial.println("║  Reporting every 3 seconds...                   ║");
  Serial.println("╚══════════════════════════════════════════════════╝");
  delay(1000);
}

// ════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
  gTemperature = readTemperature();  // read first for TDS compensation
  float ph   = readPH();
  float tds  = readTDS();
  float turb = readTurbidity();

  setLEDs(countIssues(ph, tds, turb, gTemperature));
  printReport(ph, tds, turb, gTemperature);

  delay(3000);
}
