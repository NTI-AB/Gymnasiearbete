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

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 2000; // Send every 2 seconds
unsigned long lastALSReadTime = 0;
unsigned long lastUVSReadTime = 0;
uint32_t lastALS = 0;
uint32_t lastUVS = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=====================================");
  Serial.println("   RF69 + LTR390 Sensor System");
  Serial.println("=====================================\n");
  
  pinMode(LED, OUTPUT);
  
  // Init sensor
  Serial.print("[1/3] Initializing LTR390 sensor... ");
  if (!ltr.begin()) {
    Serial.println("FAILED!");
    while (1) {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
    }
  }
  Serial.println("OK");
  
  ltr.setGain(LTR390_GAIN_3);
  ltr.setResolution(LTR390_RESOLUTION_18BIT);
  ltr.setMode(LTR390_MODE_ALS);
  Serial.println("    - Gain: 3x");
  Serial.println("    - Resolution: 18-bit\n");
  
  // Radio init
  Serial.print("[2/3] Initializing RF69 radio... ");
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("FAILED!");
    while (1) {
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);
    }
  }
  Serial.println("OK");
  
  rf69.setFrequency(RF69_FREQ);
  rf69.setTxPower(20, true);
  Serial.println("    - Frequency: 915.0 MHz");
  Serial.println("    - TX Power: 20 dBm\n");
  
  uint8_t key[] = {
    1, 2, 3, 4, 5, 6, 7, 8,
    1, 2, 3, 4, 5, 6, 7, 8
  };
  rf69.setEncryptionKey(key);
  
  Serial.print("[3/3] Encryption key... ");
  Serial.println("SET");
  
  Serial.println("\n=====================================");
  Serial.println("   System Ready - Waiting for Data");
  Serial.println("=====================================\n");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Keep reading sensor continuously
  if (ltr.newDataAvailable()) {
    if (ltr.getMode() == LTR390_MODE_ALS) {
      lastALS = ltr.readALS();
      lastALSReadTime = currentTime;
      Serial.print("  [ALS] ");
      Serial.print(lastALS);
      Serial.println(" lux");
    } else {
      lastUVS = ltr.readUVS();
      lastUVSReadTime = currentTime;
      Serial.print("  [UV]  ");
      Serial.print(lastUVS);
      Serial.println(" index");
    }
  }
  
  // Switch modes every 500ms to read both sensors
  static unsigned long lastModeSwitch = 0;
  if (currentTime - lastModeSwitch >= 750) {
    lastModeSwitch = currentTime;
    if (ltr.getMode() == LTR390_MODE_ALS) {
      ltr.setMode(LTR390_MODE_UVS);
      Serial.println("  >> Switching to UV mode");
    } else {
      ltr.setMode(LTR390_MODE_ALS);
      Serial.println("  >> Switching to ALS mode");
    }
  }
  
  // Send data at regular intervals
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    // Heartbeat LED
    digitalWrite(LED, HIGH);
    delay(40);
    digitalWrite(LED, LOW);
    
    // Send packet with last valid readings
    char packet[64];
    snprintf(packet, sizeof(packet), "U:%lu,L:%lu", lastUVS, lastALS);
    
    rf69.send((uint8_t*)packet, strlen(packet));
    rf69.waitPacketSent();
    
    Serial.print("Sent: ");
    Serial.println(packet);
  }
}
