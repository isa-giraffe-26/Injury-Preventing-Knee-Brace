#include <Wire.h>
#include <math.h>


// ====== QUICK TEST SWITCH ======
static const bool TEST_ALL_AXES = false;   // set true once to verify orientation; then set back false
// =================================


// ---------- ADXL375 I2C addresses ----------
#define ADXL_ADDR_53 0x53   // ALT=GND
#define ADXL_ADDR_1D 0x1D   // ALT=VDDIO


// ---------- ADXL375 registers ----------
#define REG_DEVID             0x00
#define REG_THRESH_SHOCK      0x1D   // 0.780 g/LSB
#define REG_DUR               0x21   // 0.625 ms/LSB
#define REG_LATENT            0x22
#define REG_WINDOW            0x23
#define REG_SHOCK_AXES        0x2A   // D3=Suppress, D2=Shock X en, D1=Y en, D0=Z en
#define REG_ACT_SHOCK_STATUS  0x2B   // RO: D2=SHOCK_X, D1=SHOCK_Y, D0=SHOCK_Z
#define REG_BW_RATE           0x2C   // 0x0C = 400 Hz ODR (~200 Hz BW)
#define REG_POWER_CTL         0x2D
#define REG_INT_ENABLE        0x2E   // 0x40 = SINGLE_SHOCK enable
#define REG_INT_MAP           0x2F   // 0x00 = map all INTs to INT1
#define REG_INT_SOURCE        0x30   // reading clears the latch
#define REG_DATA_FORMAT       0x31
#define REG_DATAX0            0x32   // .. 0x37


// ---------- constants ----------
#define BW_RATE_400HZ         0x0C   // ODR=400 Hz (~200 Hz BW)
#define MEASURE               0x08
#define INT_SINGLE_SHOCK      0x40


// ---------- user configuration ----------
const int INT_PIN = 13;                  // ADXL375 INT1 -> D13 (SAMD21 supports this)
const float G_PER_LSB = 0.049f;          // typ. ~49 mg/LSB for ADXL375 at <=800 Hz
const uint8_t THRESH_5G      = 0x06;     // ~5.0 g (6 * 0.780 g/LSB)
const uint8_t DUR_20ms       = 0x20;     // 20.0 ms (0x20 * 0.625 ms)
const uint8_t AXES_X_ONLY    = 0x04;     // D2=X enable (D1/D0=0)
const uint8_t AXES_ALL       = 0x07;     // X+Y+Z enable (for quick orientation tests)
const uint32_t COLLECTION_DURATION_MS = 20;  // 20ms collection window
const uint32_t POST_WINDOW_MS = 300;     // total sampling window after IRQ


// ---------- globals ----------
uint8_t ADXL_ADDR = ADXL_ADDR_53;
volatile bool g_irq = false;
uint32_t g_int_count = 0, g_print_count = 0, g_reject_count = 0;


// ---------- helpers ----------
static inline void debugPrintReg(uint8_t reg, const char* name) {
  uint8_t v=0;
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(ADXL_ADDR, (uint8_t)1) == 1) {
    v = Wire.read();
    Serial.print(name); Serial.print(" (0x"); Serial.print(reg, HEX);
    Serial.print(") = 0x"); Serial.println(v, HEX);
  }
}


static inline bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}


static inline bool readXYZ_g(float &gx, float &gy, float &gz) {
  uint8_t raw[6];
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(REG_DATAX0);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADXL_ADDR, (uint8_t)6) != 6) return false;
  raw[0]=Wire.read(); raw[1]=Wire.read();
  raw[2]=Wire.read(); raw[3]=Wire.read();
  raw[4]=Wire.read(); raw[5]=Wire.read();
  int16_t rx = (int16_t)((uint16_t)raw[1]<<8 | raw[0]);
  int16_t ry = (int16_t)((uint16_t)raw[3]<<8 | raw[2]);
  int16_t rz = (int16_t)((uint16_t)raw[5]<<8 | raw[4]);
  gx = rx * G_PER_LSB; gy = ry * G_PER_LSB; gz = rz * G_PER_LSB;
  return true;
}


void isrShock() { g_irq = true; }


// ---------- sensor init ----------
bool adxl375_begin() {
  // Probe address
  Wire.beginTransmission(ADXL_ADDR);
  if (Wire.endTransmission(true) != 0) ADXL_ADDR = ADXL_ADDR_1D;


  // Verify DEVID
  uint8_t id=0;
  Wire.beginTransmission(ADXL_ADDR); Wire.write(REG_DEVID);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(ADXL_ADDR, (uint8_t)1) != 1) return false;
  id = Wire.read();
  if (id != 0xE5) return false;


  // ODR 400 Hz (~200 Hz BW), ±200g, start measuring
  if (!writeReg(REG_BW_RATE,     BW_RATE_400HZ)) return false; // 0x0C
  if (!writeReg(REG_DATA_FORMAT, 0x00))          return false; // ±200 g
  if (!writeReg(REG_POWER_CTL,   MEASURE))       return false;


  // ---- Shock detection: 5 g for 20 ms ----
  if (!writeReg(REG_THRESH_SHOCK, THRESH_5G))    return false;  // 0x1D = 0x06
  if (!writeReg(REG_DUR,          DUR_20ms))     return false;  // 0x21 = 0x20 (20 ms)


  // Axes: X-only (or all axes for quick test)
  uint8_t axes = TEST_ALL_AXES ? AXES_ALL : AXES_X_ONLY;
  if (!writeReg(REG_SHOCK_AXES, axes))           return false;  // 0x2A


  // Interrupts: map to INT1, enable SINGLE_SHOCK
  if (!writeReg(REG_INT_MAP,     0x00))             return false;  // all to INT1
  if (!writeReg(REG_INT_ENABLE,  INT_SINGLE_SHOCK)) return false;  // enable SINGLE_SHOCK


  // Clear any pending IRQ
  Wire.beginTransmission(ADXL_ADDR); Wire.write(REG_INT_SOURCE);
  if (Wire.endTransmission(false) == 0) {
    (void)Wire.requestFrom(ADXL_ADDR, (uint8_t)1);
    if (Wire.available()) (void)Wire.read();
  }


  // Diagnostics (read back actual hardware values)
  Serial.println(F("=== ADXL375 register readback ==="));
  debugPrintReg(REG_THRESH_SHOCK, "THRESH_SHOCK"); // expect 0x06
  debugPrintReg(REG_DUR,          "DUR");          // expect 0x20 (20 ms)
  debugPrintReg(REG_SHOCK_AXES,   "SHOCK_AXES");   // expect 0x04 if TEST_ALL_AXES=false
  debugPrintReg(REG_INT_ENABLE,   "INT_ENABLE");   // expect 0x40
  debugPrintReg(REG_BW_RATE,      "BW_RATE");      // expect 0x0C
  Serial.println(F("================================="));
  return true;
}


void setup() {
  Serial.begin(115200); while (!Serial) {}
  Serial.println(F("Nano 33 IoT + ADXL375 — collect 20ms of data, print Y-axis when X reaches ~5g"));


  Wire.begin();
  Wire.setClock(400000);   // I2C 400 kHz


  if (!adxl375_begin()) {
    Serial.println(F("Init failed (check wiring, address, DEVID)."));
    while (1) delay(1000);
  }


  pinMode(INT_PIN, INPUT); // ADXL375 INT is push-pull, active-high
  attachInterrupt(digitalPinToInterrupt(INT_PIN), isrShock, RISING);


  Serial.println(F("Ready. If you see no prints, try TEST_ALL_AXES=true for a quick sanity check."));
}


void loop() {
  if (!g_irq) return;
  g_irq = false;
  g_int_count++;


  // 1) Read ACT_SHOCK_STATUS BEFORE clearing INT_SOURCE (snapshot which axis)
  uint8_t status = 0, src = 0;
  Wire.beginTransmission(ADXL_ADDR); Wire.write(REG_ACT_SHOCK_STATUS);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(ADXL_ADDR, (uint8_t)1) == 1) status = Wire.read();


  // 2) Clear the interrupt latch
  Wire.beginTransmission(ADXL_ADDR); Wire.write(REG_INT_SOURCE);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(ADXL_ADDR, (uint8_t)1) == 1) src = Wire.read();


  // 3) Collect data for exactly 20ms duration
  const float thresh_g = THRESH_5G * 0.780f; // ~5.0 g
  float peakX = 0.0f;
  float y_at_5g_x = 0.0f;  // Y-axis value when X reaches ~5g
  bool found_5g_x = false;
  uint32_t t0 = millis();
  uint32_t sample_count = 0;
 
  Serial.println(F("Interrupt triggered - collecting data for 20ms..."));


  // Collect samples for exactly COLLECTION_DURATION_MS (20ms)
  while ((millis() - t0) < COLLECTION_DURATION_MS) {
    float xg, yg, zg;
    if (!readXYZ_g(xg, yg, zg)) break;
   
    const float ax = fabs(xg);
    sample_count++;
   
    // Track peak X value during collection
    if (ax > peakX) peakX = ax;
   
    // Check if X-axis reached approximately 5g and capture Y-axis value
    if (!found_5g_x && ax >= (thresh_g * 0.95f)) {  // ~95% of 5g threshold for detection
      y_at_5g_x = yg;  // Store the Y-axis value (signed, not absolute)
      found_5g_x = true;
      Serial.print(F("X reached ~5g: Y-axis = "));
      Serial.print(y_at_5g_x, 3);
      Serial.println(F("g"));
    }
   
    // Optional: print current reading (uncomment for debugging)
    // Serial.print(F("Sample ")); Serial.print(sample_count);
    // Serial.print(F(": X=")); Serial.print(xg, 2);
    // Serial.print(F("g, Y=")); Serial.print(yg, 2);
    // Serial.print(F("g, |X|=")); Serial.print(ax, 2);
    // Serial.print(F("g, time=")); Serial.print(millis() - t0);
    // Serial.println(F("ms"));
  }


  // 4) Print results summary
  g_print_count++;
  Serial.print(F("Collection complete - peak|X|="));
  Serial.print(peakX, 2);
  Serial.print(F("g, samples="));
  Serial.print(sample_count);
  Serial.print(F(", duration="));
  Serial.print(millis() - t0);
  Serial.print(F("ms, status(0x2B)=0x"));
  Serial.print(status, HEX);
 
  if (found_5g_x) {
    Serial.print(F(", Y@5gX="));
    Serial.print(y_at_5g_x, 3);
    Serial.print(F("g"));
  } else {
    Serial.print(F(", X never reached 5g"));
  }
 
  Serial.print(F("  [ints=")); Serial.print(g_int_count);
  Serial.print(F(", collections=")); Serial.print(g_print_count);
  Serial.println(F("]"));
}
