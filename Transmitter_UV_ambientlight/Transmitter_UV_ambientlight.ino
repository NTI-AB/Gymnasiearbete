#include <Wire.h>
#include <Adafruit_LTR390.h>

#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0
#define RFM69_CS   16
#define RFM69_INT  21
#define RFM69_RST  17
#define LED        LED_BUILTIN

RH_RF69 rf69(RFM69_CS, RFM69_INT);

Adafruit_LTR390 ltr;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  if (!ltr.begin()) {
    while (1) {
      digitalWrite(LED, HIGH); delay(200);
      digitalWrite(LED, LOW); delay(200);
    }
  }

  ltr.setGain(LTR390_GAIN_3);
  ltr.setResolution(LTR390_RESOLUTION_18BIT);

  // Radio init
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

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
  // Heartbeat
  digitalWrite(LED, HIGH); delay(40);
  digitalWrite(LED, LOW);

  // === READ ALS ===
  ltr.setMode(LTR390_MODE_ALS);

  // Wait for new data
  while (!ltr.newDataAvailable()) delay(5);
  uint32_t als = ltr.readALS();


  // === READ UV ===
  ltr.setMode(LTR390_MODE_UVS);

  // Wait for new UV data
  while (!ltr.newDataAvailable()) delay(5);
  uint32_t uvs = ltr.readUVS();


  // === SEND PACKET ===
  char packet[64];
  snprintf(packet, sizeof(packet),
           "U:%lu,L:%lu",
           uvs, als);

  rf69.send((uint8_t*)packet, strlen(packet));
  rf69.waitPacketSent();

  delay(500);
}
