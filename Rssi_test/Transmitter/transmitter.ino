#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ  868.0
#define RFM69_CS   16
#define RFM69_INT  21
#define RFM69_RST  17
#define LED        LED_BUILTIN
#define BURST_SIZE 20

RH_RF69 rf69(RFM69_CS, RFM69_INT);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

uint32_t seq = 0;

void sendBurst() {
  Serial.println("--- BURST START ---");
  for (int i = 0; i < BURST_SIZE; i++) {
    sensors_event_t event;
    accel.getEvent(&event);

    char payload[64];
    snprintf(payload, sizeof(payload), "%lu,%.2f,%.2f,%.2f",
             seq++,
             event.acceleration.x,
             event.acceleration.y,
             event.acceleration.z);

    rf69.send((uint8_t*)payload, strlen(payload));
    rf69.waitPacketSent();

    Serial.print("TX "); Serial.print(seq - 1);
    Serial.print(": "); Serial.println(payload);

    digitalWrite(LED, HIGH); delay(30); digitalWrite(LED, LOW);
    delay(120);  // ~150ms per paket = ~3s för 20 paket
  }
  Serial.println("--- BURST DONE ---");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  pinMode(LED, OUTPUT);

  if (!accel.begin()) {
    for (;;) { digitalWrite(LED, !digitalRead(LED)); delay(200); }
  }
  accel.setRange(ADXL345_RANGE_4_G);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH); delay(10);
  digitalWrite(RFM69_RST, LOW);  delay(10);

  if (!rf69.init()) {
    for (;;) { digitalWrite(LED, !digitalRead(LED)); delay(100); }
  }

  rf69.setFrequency(RF69_FREQ);
  rf69.setTxPower(20, true);

  uint8_t key[] = { 1,2,3,4,5,6,7,8, 1,2,3,4,5,6,7,8 };
  rf69.setEncryptionKey(key);

  Serial.println("TX READY — skriv vad som helst + Enter för att skicka burst");
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();  // töm buffern
    sendBurst();
  }
}