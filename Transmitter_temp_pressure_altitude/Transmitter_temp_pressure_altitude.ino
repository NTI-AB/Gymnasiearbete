#include <Wire.h>
#include <Adafruit_LTR390.h>

#include <SPI.h>
#include <RH_RF69.h>

// === RADIO CONFIG (RP2040 Feather RFM69) ===
#define RF69_FREQ 915.0
#define RFM69_CS   16
#define RFM69_INT  21
#define RFM69_RST  17
#define LED        LED_BUILTIN

RH_RF69 rf69(RFM69_CS, RFM69_INT);

// === LTR390 SENSOR ===
Adafruit_LTR390 ltr = Adafruit_LTR390();

void setup() {
  Serial.begin(115200); // No waiting
  pinMode(LED, OUTPUT);

  // --- LTR390 INIT ---
  if (!ltr.begin()) {
    // Blink forever if sensor fails
    while (1) {
      digitalWrite(LED, HIGH); delay(200);
      digitalWrite(LED, LOW); delay(200);
    }
  }

  // Configure sensor
  ltr.setMode(LTR390_MODE_UVS);    // UV mode
  ltr.setGain(LTR390_GAIN_3);      // Default gain
  ltr.setResolution(LTR390_RES_18BIT); // Good resolution

  // --- RADIO RESET ---
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  // --- RADIO INIT ---
  if (!rf69.init()) {
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

void loop() {
  // === HEARTBEAT BLINK ===
  digitalWrite(LED, HIGH);
  delay(40);
  digitalWrite(LED, LOW);

  // Read raw UVA and ALS values
  uint32_t uvs = ltr.readUVS();
  uint32_t als = ltr.readALS();

  // Format packet: U:<UV RAW>,L:<Light RAW>
  char packet[64];
  snprintf(packet, sizeof(packet),
           "U:%lu,L:%lu",
           uvs, als);

  // Send packet
  rf69.send((uint8_t*)packet, strlen(packet));
  rf69.waitPacketSent();

  delay(960); // ~1s loop time
}
