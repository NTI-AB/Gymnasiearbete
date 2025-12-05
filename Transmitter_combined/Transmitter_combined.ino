#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>      // Unified BMP180
#include <Adafruit_ADXL345_U.h>     // Unified accelerometer
#include <Adafruit_LTR390.h>        // UV + ALS
#include <RH_RF69.h>

// ===============================
// TRANSMITTER TOGGLE
// Set to false to disable RF transmission for testing
// ===============================
#define ENABLE_TRANSMITTER false

// ===============================
// I2C DEBUG MODE
// Set to true to scan I2C bus and show detailed info
// ===============================
#define I2C_DEBUG true

// ===============================
// RADIO CONFIG
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

// ADXL345 I2C address
// 0x53 if SDO grounded (default)
// 0x1D if SDO connected to VCC
#define ADXL345_I2C_ADDR 0x1D

// Track if sensors initialized OK
bool bmp_ok = false;
bool accel_ok = false;
bool ltr_ok = false;
bool rf_ok = false;

// =============================================
// FAILSAFE VALUE
// =============================================
#define FAILSAFE_VALUE -999

// =============================================
// I2C SCANNER
// =============================================
void scanI2C() {
  Serial.println("\n=== I2C Bus Scanner ===");
  Serial.println("Scanning...");
  
  byte count = 0;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at address 0x");
      if (i < 16) Serial.print("0");
      Serial.print(i, HEX);
      Serial.print(" (");
      Serial.print(i);
      Serial.print(")");
      
      // Identify common sensors
      if (i == 0x1D) Serial.print(" <- ADXL345 (SDO=HIGH)");
      if (i == 0x53) Serial.print(" <- ADXL345 (SDO=LOW) or LTR390");
      if (i == 0x77) Serial.print(" <- BMP180");
      
      Serial.println();
      count++;
      delay(1);
    }
  }
  
  if (count == 0) {
    Serial.println("ERROR: No I2C devices found!");
    Serial.println("Check:");
    Serial.println("  - SDA/SCL connections");
    Serial.println("  - Power to sensors");
    Serial.println("  - Pull-up resistors (4.7k-10k)");
  } else {
    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" device(s)");
  }
  
  Serial.println("\nCommon I2C Addresses:");
  Serial.println("  ADXL345: 0x53 (default) or 0x1D");
  Serial.println("  BMP180:  0x77");
  Serial.println("  LTR390:  0x53 (default)");
  Serial.println("\n⚠️  ADXL345 and LTR390 both use 0x53!");
  Serial.println("   You need to change one address or use I2C multiplexer\n");
  Serial.println("========================\n");
}

// =============================================
// SETUP
// =============================================
void setup() {
  Serial.begin(9600);
  delay(2000);  // Give serial time to stabilize
  
  pinMode(LED, OUTPUT);

  Serial.println("\n\n=== ESP32 Sensor Suite Starting ===");
  Serial.print("RF Transmitter: ");
  Serial.println(ENABLE_TRANSMITTER ? "ENABLED" : "DISABLED");
  Serial.print("I2C Debug: ");
  Serial.println(I2C_DEBUG ? "ENABLED" : "DISABLED");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000);  // 100kHz - slower but more stable
  delay(100);

  if (I2C_DEBUG) {
    scanI2C();
  }

  // ---- BMP180 ----
  Serial.print("Initializing BMP180 (0x77)... ");
  bmp_ok = bmp.begin();
  if (bmp_ok) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
    Serial.println("  -> Check wiring and I2C address");
  }

  // ---- ADXL345 ----
  Serial.print("Initializing ADXL345 (0x");
  Serial.print(ADXL345_I2C_ADDR, HEX);
  Serial.print(")... ");
  accel_ok = accel.begin(ADXL345_I2C_ADDR);
  if (accel_ok) {
    accel.setRange(ADXL345_RANGE_4_G);
    Serial.println("OK");
    
    // Test read
    sensors_event_t evt;
    accel.getEvent(&evt);
    Serial.print("  -> Test read: X=");
    Serial.print(evt.acceleration.x);
    Serial.print(" Y=");
    Serial.print(evt.acceleration.y);
    Serial.print(" Z=");
    Serial.println(evt.acceleration.z);
  } else {
    Serial.println("FAILED");
    Serial.println("  -> Address conflict with LTR390?");
  }

  // ---- LTR390 ----
  Serial.print("Initializing LTR390 (0x53)... ");
  ltr_ok = ltr.begin();
  if (ltr_ok) {
    ltr.setGain(LTR390_GAIN_3);
    ltr.setResolution(LTR390_RESOLUTION_18BIT);
    ltr.setMode(LTR390_MODE_ALS);
    Serial.println("OK");
    
    delay(100);
    if (ltr.newDataAvailable()) {
      Serial.print("  -> Test read: ALS=");
      Serial.println(ltr.readALS());
    }
  } else {
    Serial.println("FAILED");
    Serial.println("  -> Address conflict with ADXL345?");
  }

  // ---- RF INIT (only if enabled) ----
  if (ENABLE_TRANSMITTER) {
    Serial.print("Initializing RFM69... ");
    
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, HIGH); delay(10);
    digitalWrite(RFM69_RST, LOW);  delay(10);

    rf_ok = rf69.init();
    
    if (!rf_ok) {
      Serial.println("FAILED");
      Serial.println("ERROR: Radio init failed - check wiring!");
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
    
    Serial.println("OK");
  } else {
    Serial.println("RFM69 skipped (transmitter disabled)");
  }

  Serial.println("\n=== Initialization Complete ===");
  
  if (accel_ok && ltr_ok) {
    Serial.println("⚠️  WARNING: Both ADXL345 and LTR390 initialized!");
    Serial.println("   This shouldn't work - they share address 0x53");
    Serial.println("   One may be reading garbage data");
  }
  
  Serial.println("\nStarting main loop...\n");
  delay(1000);
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

  // More thorough NaN check
  if (isnan(evt.acceleration.x) || isnan(evt.acceleration.y) || isnan(evt.acceleration.z)) {
    ax = ay = az = FAILSAFE_VALUE;
    return;
  }

  ax = evt.acceleration.x;
  ay = evt.acceleration.y;
  az = evt.acceleration.z;
}

// ---- SAFE LTR390 ----
// Store last valid readings
static uint32_t last_als = 0;
static uint32_t last_uvs = 0;

void readLTR(uint32_t &als, uint32_t &uvs) {
  if (!ltr_ok) {
    als = uvs = FAILSAFE_VALUE;
    return;
  }

  // Read current mode and update that value
  if (ltr.newDataAvailable()) {
    if (ltr.getMode() == LTR390_MODE_ALS) {
      last_als = ltr.readALS();
    } else {
      last_uvs = ltr.readUVS();
    }
  }
  
  // Always return the last valid readings for both
  als = last_als;
  uvs = last_uvs;
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

  // Alternate LTR390 mode every 800ms for UV/ALS
  static unsigned long lastSwitch = 0;
  if (ltr_ok && now - lastSwitch > 800) {
    lastSwitch = now;
    if (ltr.getMode() == LTR390_MODE_ALS)
      ltr.setMode(LTR390_MODE_UVS);
    else
      ltr.setMode(LTR390_MODE_ALS);
  }

  // =======================================
  // BUILD PACKET
  // =======================================
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
  // OUTPUT TO SERIAL
  // ======================
  Serial.print("Data: ");
  Serial.println(packet);

  // ======================
  // SEND VIA RF (if enabled)
  // ======================
  if (ENABLE_TRANSMITTER && rf_ok) {
    rf69.send((uint8_t *)packet, strlen(packet));
    rf69.waitPacketSent();
    Serial.println("  -> Transmitted");
  }

  delay(900); // ~1 second loop
}