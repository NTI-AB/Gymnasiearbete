#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ  868.0
#define RFM69_CS   16
#define RFM69_INT  21
#define RFM69_RST  17
#define LED        LED_BUILTIN
#define BURST_SIZE 20
#define BURST_TIMEOUT_MS 5000

RH_RF69 rf69(RFM69_CS, RFM69_INT);

void runBurst() {
  uint32_t received  = 0;
  uint32_t lost      = 0;
  uint32_t lastSeq   = UINT32_MAX;
  int32_t  rssiSum   = 0;
  int16_t  rssiMin   = 0;
  int16_t  rssiMax   = -200;
  uint32_t deadline  = millis() + BURST_TIMEOUT_MS;

  Serial.println("--- BURST INCOMING ---");

  while (millis() < deadline) {
    if (rf69.available()) {
      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf69.recv(buf, &len)) {
        buf[len] = 0;
        received++;
        deadline = millis() + BURST_TIMEOUT_MS;  // reset timeout på varje paket

        uint32_t seq = atol((char*)buf);
        int16_t rssi = rf69.lastRssi();

        rssiSum += rssi;
        if (rssi < rssiMin) rssiMin = rssi;
        if (rssi > rssiMax) rssiMax = rssi;

        if (lastSeq != UINT32_MAX && seq > lastSeq + 1) {
          lost += seq - lastSeq - 1;
        }
        lastSeq = seq;

        Serial.print("  PKT "); Serial.print(seq);
        Serial.print(" RSSI "); Serial.print(rssi); Serial.println(" dBm");

        digitalWrite(LED, HIGH); delay(30); digitalWrite(LED, LOW);
      }
    }
  }

  // --- SAMMANFATTNING ---
  uint32_t total     = received + lost;
  float    lossRate  = total > 0 ? (100.0f * lost / total) : 0.0f;
  float    rssiAvg   = received > 0 ? (float)rssiSum / received : 0;

  Serial.println("--- BURST RESULTAT ---");
  Serial.print("  Mottagna:    "); Serial.print(received); Serial.print(" / "); Serial.println(total);
  Serial.print("  Packet loss: "); Serial.print(lossRate, 1); Serial.println(" %");
  Serial.print("  RSSI avg:    "); Serial.print(rssiAvg, 1); Serial.println(" dBm");
  Serial.print("  RSSI min:    "); Serial.print(rssiMin); Serial.println(" dBm");
  Serial.print("  RSSI max:    "); Serial.print(rssiMax); Serial.println(" dBm");
  Serial.println("----------------------");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  pinMode(LED, OUTPUT);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH); delay(10);
  digitalWrite(RFM69_RST, LOW);  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 INIT FAILED");
    for (;;) { digitalWrite(LED, !digitalRead(LED)); delay(100); }
  }

  rf69.setFrequency(RF69_FREQ);
  rf69.setTxPower(20, true);

  uint8_t key[] = { 1,2,3,4,5,6,7,8, 1,2,3,4,5,6,7,8 };
  rf69.setEncryptionKey(key);

  Serial.println("RX READY — väntar på burst...");
}

void loop() {
  // Vänta på första paketet i en ny burst
  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      // Lägg tillbaka i kön via en wrapper — istället hanterar vi det direkt
      buf[len] = 0;

      uint32_t seq   = atol((char*)buf);
      int16_t  rssi  = rf69.lastRssi();

      // Starta burst-logik med detta första paket redan inläst
      uint32_t received  = 1;
      uint32_t lost      = 0;
      uint32_t lastSeq   = seq;
      int32_t  rssiSum   = rssi;
      int16_t  rssiMin   = rssi;
      int16_t  rssiMax   = rssi;
      uint32_t deadline  = millis() + BURST_TIMEOUT_MS;

      Serial.println("--- BURST INCOMING ---");
      Serial.print("  PKT "); Serial.print(seq);
      Serial.print(" RSSI "); Serial.print(rssi); Serial.println(" dBm");
      digitalWrite(LED, HIGH); delay(30); digitalWrite(LED, LOW);

      while (millis() < deadline) {
        if (rf69.available()) {
          uint8_t buf2[RH_RF69_MAX_MESSAGE_LEN];
          uint8_t len2 = sizeof(buf2);
          if (rf69.recv(buf2, &len2)) {
            buf2[len2] = 0;
            received++;
            deadline = millis() + BURST_TIMEOUT_MS;

            uint32_t seq2  = atol((char*)buf2);
            int16_t  rssi2 = rf69.lastRssi();

            rssiSum += rssi2;
            if (rssi2 < rssiMin) rssiMin = rssi2;
            if (rssi2 > rssiMax) rssiMax = rssi2;

            if (seq2 > lastSeq + 1) lost += seq2 - lastSeq - 1;
            lastSeq = seq2;

            Serial.print("  PKT "); Serial.print(seq2);
            Serial.print(" RSSI "); Serial.print(rssi2); Serial.println(" dBm");
            digitalWrite(LED, HIGH); delay(30); digitalWrite(LED, LOW);
          }
        }
      }

      uint32_t total    = received + lost;
      float    lossRate = total > 0 ? (100.0f * lost / total) : 0.0f;
      float    rssiAvg  = received > 0 ? (float)rssiSum / received : 0;

      Serial.println("--- BURST RESULTAT ---");
      Serial.print("  Mottagna:    "); Serial.print(received); Serial.print(" / "); Serial.println(total);
      Serial.print("  Packet loss: "); Serial.print(lossRate, 1); Serial.println(" %");
      Serial.print("  RSSI avg:    "); Serial.print(rssiAvg, 1); Serial.println(" dBm");
      Serial.print("  RSSI min:    "); Serial.print(rssiMin); Serial.println(" dBm");
      Serial.print("  RSSI max:    "); Serial.print(rssiMax); Serial.println(" dBm");
      Serial.println("----------------------");
    }
  }
}