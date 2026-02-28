
#include <SPI.h>
#include <LoRa.h>

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_BAND 433E6

bool imuDisplayEnabled = true;

void applyMode(String mode) {
  if (mode == "fast") { LoRa.setSpreadingFactor(7); LoRa.setSignalBandwidth(250E3); }
  else if (mode == "balanced") { LoRa.setSpreadingFactor(9); LoRa.setSignalBandwidth(125E3); }
  else if (mode == "slow") { LoRa.setSpreadingFactor(12); LoRa.setSignalBandwidth(125E3); }
  LoRa.setCodingRate4(8);
  Serial.println("RX MODE: " + mode);
}

void setup() {
  Serial.begin(115200);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) { Serial.println("LoRa FAIL"); while (1); }
  applyMode("balanced");
  LoRa.receive();
  Serial.println("RX READY");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";
    while (LoRa.available()) msg += (char)LoRa.read();
    
    if (msg.startsWith("IMU:") && imuDisplayEnabled) {
      Serial.println("RECV " + msg); 
    }
    else if (msg == "CTRL:TX_LISTEN") {
      imuDisplayEnabled = false;
      Serial.println("!!! TX IS NOW LISTENING - TYPE CMD NOW !!!");
    }
    else if (msg == "CTRL:IMU_RESUME") {
      imuDisplayEnabled = true;
      Serial.println("!!! IMU RESUMED !!!");
    }
    else if (msg.startsWith("ACK:")) {
      applyMode(msg.substring(4));
      Serial.println("MODE CONFIRMED BY TX");
    }
  }

  // NON-BLOCKING SERIAL READ
  if (!imuDisplayEnabled && Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial.println("SENDING TO TX: " + cmd);
      LoRa.beginPacket();
      LoRa.print(cmd);
      LoRa.endPacket();
      LoRa.receive(); // Go back to listening for the ACK
    }
  }
}
