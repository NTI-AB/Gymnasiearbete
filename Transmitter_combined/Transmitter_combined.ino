#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>      // Unified BMP180
#include <Adafruit_ADXL345_U.h>     // Unified accelerometer
#include <Adafruit_LTR390.h>        // UV + ALS
#include <RH_RF69.h>

// ===============================
// RADIO CONFIG (unchanged)
// ===============================
#define RF69_FREQ 915.0
#define RFM69_CS   16
#define RFM69_INT  21
#define RFM69_RST  17
#define LED        LED_BUILTIN

RH_RF69 rf69(RFM69_CS, RFM69_INT);

// ===============================
// SENSORS (unified versions)
// ===============================
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_LTR390 ltr;

// Track if sensors initialized OK
bool bmp_ok = false;
bool accel_ok = false;
bool ltr_ok = false;

// =============================================
// FAILSAFE VALUE
// =============================================
#define FAILSAFE_VALUE -999

// =============================================
// SETUP
// =============================================
void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);

  // ---- BMP180 ----
  bmp_ok = bmp.begin();

  // ---- ADXL345 ----
  accel_ok = accel.begin();
  if (accel_ok) accel.setRange(ADXL345_RANGE_4_G);

  // ---- LTR390 ----
  ltr_ok = ltr.begin();
  if (ltr_ok) {
    ltr.setGain(LTR390_GAIN_3);
    ltr.setResolution(LTR390_RESOLUTION_18BIT);
    ltr.setMode(LTR390_MODE_ALS);
  }

  // ---- RF RESET ----
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH); delay(10);
  digitalWrite(RFM69_RST, LOW);  delay(10);

  // ---- RF INIT ----
  if (!rf69.init()) {
    // Radio absolutely required → hard error
    while (1) {
      digitalWrite(LED, HIGH); delay(100);
      digitalWrite(LED, LOW); delay(100);
    }
  }

  rf69.setFrequency(RF69_FREQ);
  rf69.setTxPower(20, true);

  uint8_t key[] = {
    1,2,3,4,5,6,7,8,
    1,2,3,4,5,6,7,8
  };
  rf69.setEncryptionKey(key);
}

// =============================================
// SAFE READ FUNCTIONS FOR EACH SENSOR
// =============================================

// ---- SAFE BMP180 ----
void readBMP(float &tempC, float &pressurePa, float &altM) {
  if (!bmp_ok) {
    tempC = pressurePa = altM = FAILSAFE_VALUE;
    return;
  }

  sensors_event_t event;
  bmp.getEvent(&event);

  if (!event.pressure) {
    tempC = pressurePa = altM = FAILSAFE_VALUE;
    return;
  }

  float t;
  bmp.getTemperature(&t);
  tempC = t;
  pressurePa = event.pressure * 100;   // hPa → Pa
  altM = bmp.pressureToAltitude(1013.25, event.pressure);
}

// ---- SAFE ADXL345 ----
void readADXL(float &ax, float &ay, float &az) {
  if (!accel_ok) {
    ax = ay = az = FAILSAFE_VALUE;
    return;
  }

  sensors_event_t evt;
  accel.getEvent(&evt);

  if (isnan(evt.acceleration.x)) {
    ax = ay = az = FAILSAFE_VALUE;
    return;
  }

  ax = evt.acceleration.x;
  ay = evt.acceleration.y;
  az = evt.acceleration.z;
}

// ---- SAFE LTR390 ----
void readLTR(uint32_t &als, uint32_t &uvs) {
  if (!ltr_ok) {
    als = uvs = FAILSAFE_VALUE;
    return;
  }

  // Read both — using mode switching like your original
  if (ltr.newDataAvailable()) {
    if (ltr.getMode() == LTR390_MODE_ALS) {
      als = ltr.readALS();
    } else {
      uvs = ltr.readUVS();
    }
  }
}

// =============================================
// MAIN LOOP
// =============================================
void loop() {
  unsigned long now = millis();

  // Heartbeat blink
  digitalWrite(LED, HIGH);
  delay(20);
  digitalWrite(LED, LOW);

  // ======================
  // READ SENSORS SAFELY
  // ======================
  float tempC, pressurePa, altM;
  readBMP(tempC, pressurePa, altM);

  float ax, ay, az;
  readADXL(ax, ay, az);

  uint32_t als = FAILSAFE_VALUE;
  uint32_t uvs = FAILSAFE_VALUE;
  readLTR(als, uvs);

  // Force simple alternating mode every 800ms for UV/ALS
  static unsigned long lastSwitch = 0;
  if (ltr_ok && now - lastSwitch > 800) {
    lastSwitch = now;
    if (ltr.getMode() == LTR390_MODE_ALS)
      ltr.setMode(LTR390_MODE_UVS);
    else
      ltr.setMode(LTR390_MODE_ALS);
  }

  // =======================================
  // PACKET FORMAT
  // ONE CONSISTENT PACKET, ALL SENSORS
  // =======================================
  // Example:
  // T:23.4|P:100812|A:12.3|UV:200|L:180|AX:-0.12|AY:0.30|AZ:9.71

  char packet[128];
  snprintf(packet, sizeof(packet),
           "T:%.2f|P:%.0f|A:%.2f|UV:%lu|L:%lu|AX:%.2f|AY:%.2f|AZ:%.2f",
           tempC,
           pressurePa,
           altM,
           (unsigned long)uvs,
           (unsigned long)als,
           ax, ay, az);

  // ======================
  // SEND PACKET
  // ======================
  rf69.send((uint8_t *)packet, strlen(packet));
  rf69.waitPacketSent();

  delay(900); // ~1 second loop
}
