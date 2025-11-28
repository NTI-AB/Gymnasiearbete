#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include <SPI.h>
#include <RH_RF69.h>

// --- RADIO CONFIG ---
#define RF69_FREQ 915.0
#define RFM69_CS   16
#define RFM69_INT  21
#define RFM69_RST  17
#define LED        LED_BUILTIN

RH_RF69 rf69(RFM69_CS, RFM69_INT);

// --- ADXL345 ---
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup() {
  Serial.begin(115200);   // Works even without Serial Monitor, no waiting.
  pinMode(LED, OUTPUT);

  // --- ADXL init ---
  if(!accel.begin()) {
    // No Serial waiting — just fail with LED blink
    for (;;) {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
    }
  }
  accel.setRange(ADXL345_RANGE_4_G);

  // --- RADIO INIT ---
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    for (;;) { // Blink fast if radio fails
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);
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

void loop() {
  // === HEARTBEAT BLINK ===
  digitalWrite(LED, HIGH);
  delay(50);
  digitalWrite(LED, LOW);

  // === READ ADXL345 ===
  sensors_event_t event;
  accel.getEvent(&event);

  char payload[50];
  snprintf(payload, sizeof(payload), "%.2f,%.2f,%.2f",
           event.acceleration.x,
           event.acceleration.y,
           event.acceleration.z);

  rf69.send((uint8_t*)payload, strlen(payload));
  rf69.waitPacketSent();

  delay(950);
}
