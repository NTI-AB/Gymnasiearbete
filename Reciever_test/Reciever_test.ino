#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0
#define RFM69_CS   16
#define RFM69_INT  21
#define RFM69_RST  17

#define LED LED_BUILTIN

RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() {
  Serial.begin(115200);  // NO WAITING

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  Serial.println("Booting RX...");

  if (!rf69.init()) {
    Serial.println("RFM69 INIT FAILED");
    while (1) {
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

  Serial.println("RX READY!");
}

void loop() {
  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf69.recv(buf, &len)) {
      buf[len] = 0;

      Serial.print("Received: ");
      Serial.println((char*)buf);

      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
    }
  }
}
