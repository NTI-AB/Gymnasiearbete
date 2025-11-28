#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include <SPI.h>
#include <RH_RF69.h>

// === RADIO CONFIG (RP2040 Feather RFM69) ===
#define RF69_FREQ 915.0
#define RFM69_CS   16
#define RFM69_INT  21
#define RFM69_RST  17
#define LED        LED_BUILTIN

RH_RF69 rf69(RFM69_CS, RFM69_INT);

// === BMP180 SENSOR (Unified) ===
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

void setup() {
  Serial.begin(115200); // No waiting
  pinMode(LED, OUTPUT);

  // --- BMP INIT ---
  if (!bmp.begin()) {
    // Blink forever if BMP180 fails
    while (1) {
      digitalWrite(LED, HIGH); delay(200);
      digitalWrite(LED, LOW); delay(200);
    }
  }

  // --- RADIO RESET ---
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  // --- RADIO INIT ---
  if (!rf69.init()) {
    // Blink fast if radio fails
    while (1) {
      digitalWrite(LED, HIGH); delay(100);
      digitalWrite(LED, LOW); delay(100);
    }
  }

  rf69.setFrequency(RF69_FREQ);
  rf69.setTxPower(20, true);

  // AES KEY (must match receiver)
  uint8_t key[] = {
    1,2,3,4,5,6,7,8,
    1,2,3,4,5,6,7,8
  };
  rf69.setEncryptionKey(key);
}

void loop() {
  // === HEARTBEAT BLINK ===
  digitalWrite(LED, HIGH);
  delay(40);
  digitalWrite(LED, LOW);

  // === READ BMP180 (Unified API) ===
  sensors_event_t event;
  bmp.getEvent(&event);
  
  float pressure = 0;
  float temperature = 0;
  float altitude = 0;
  
  if (event.pressure) {
    pressure = event.pressure * 100; // Convert hPa to Pascals
    
    bmp.getTemperature(&temperature);
    
    // Calculate altitude (assuming sea level pressure = 1013.25 hPa)
    altitude = bmp.pressureToAltitude(1013.25, event.pressure);
  }

  // === FORMAT PACKET ===
  // Example: T:23.45,P:100812,A:12.30
  char packet[64];
  snprintf(packet, sizeof(packet),
           "T:%.2f,P:%.0f,A:%.2f",
           temperature,
           pressure,
           altitude);

  // === SEND PACKET ===
  rf69.send((uint8_t*)packet, strlen(packet));
  rf69.waitPacketSent();

  delay(960); // ~1 second total loop time
}