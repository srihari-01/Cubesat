
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

/* ---------- LORA PINS ---------- */
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_BAND 433E6

/* ---------- TIMING ---------- */
#define IMU_TX_TIME_MS 60000 
#define CMD_LISTEN_TIME_MS 15000 

String currentMode = "balanced";

/* ---------- MPU6050 ---------- */
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define SDA_PIN 21
#define SCL_PIN 22

/* ---------- CALIBRATION DATA ---------- */
float ax_off = 291.30, ay_off = -516.68, az_off = -902.19;
float gx_off = -273.41, gy_off = -193.28, gz_off = -252.62;

int16_t ax, ay, az, gx, gy, gz;
float roll = 0.0, pitch = 0.0;
float initialRoll = 0.0, initialPitch = 0.0; // Home reference added for drift correction
const float alpha = 0.98;
unsigned long lastMicros = 0;

void writeMPU(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

bool readMPU(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, (int)len);
  if (Wire.available() < len) return false;
  for (uint8_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

void applyMode(String mode) {
  if (mode == "fast") { LoRa.setSpreadingFactor(7); LoRa.setSignalBandwidth(250E3); }
  else if (mode == "balanced") { LoRa.setSpreadingFactor(9); LoRa.setSignalBandwidth(125E3); }
  else if (mode == "slow") { LoRa.setSpreadingFactor(12); LoRa.setSignalBandwidth(125E3); }
  LoRa.setCodingRate4(8);
  Serial.println("MODE SET: " + mode);
}

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize I2C with your successful test settings
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); 
  writeMPU(PWR_MGMT_1, 0x00);
  delay(500);

  // 2. DRIFT CORRECTION: Capture "Home" Position (Added logic)
  Serial.println("Calibrating Home Position...");
  float rollSum = 0, pitchSum = 0;
  int readings = 50;
  for(int i=0; i<readings; i++) {
     uint8_t raw[6];
     if(readMPU(ACCEL_XOUT_H, raw, 6)) {
       float axg = ((int16_t)(raw[0]<<8|raw[1]) - ax_off) / 16384.0;
       float ayg = ((int16_t)(raw[2]<<8|raw[3]) - ay_off) / 16384.0;
       float azg = ((int16_t)(raw[4]<<8|raw[5]) - az_off) / 16384.0;
       rollSum += atan2(ayg, azg) * 57.2958;
       pitchSum += atan2(-axg, sqrt(ayg * ayg + azg * azg)) * 57.2958;
     }
     delay(10);
  }
  initialRoll = rollSum / readings;
  initialPitch = pitchSum / readings;
  roll = initialRoll;
  pitch = initialPitch;
  Serial.println("Home Set!");
  
  // 3. LoRa Initialization
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) { Serial.println("LoRa FAIL"); while (1); }
  
  applyMode(currentMode);
  lastMicros = micros();
  Serial.println("TX READY");
}

void loop() {
  // --- PHASE 1: IMU TRANSMIT ---
  unsigned long imuStart = millis();
  
  // RESTORED YOUR EXACT TIMING LOGIC
  int sendDelayMs = 1000; // default 1 packet/sec
  if (currentMode == "fast") {
    sendDelayMs = 100;      // 10 packets/sec
  } else if (currentMode == "balanced") {
    sendDelayMs = 200;      // ~5 packets/sec
  } else if (currentMode == "slow") {
    sendDelayMs = 1000;     // 1 packet/sec
  }
  
  Serial.print("Send delay set to: ");
  Serial.print(sendDelayMs);
  Serial.println(" ms");

  while (millis() - imuStart < IMU_TX_TIME_MS) {
    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6;
    lastMicros = now;

    uint8_t raw[14];
    if (readMPU(ACCEL_XOUT_H, raw, 14)) {
      ax = (raw[0] << 8) | raw[1]; ay = (raw[2] << 8) | raw[3]; az = (raw[4] << 8) | raw[5];
      gx = (raw[8] << 8) | raw[9]; gy = (raw[10] << 8) | raw[11]; gz = (raw[12] << 8) | raw[13];
      
      float ax_g = (ax - ax_off) / 16384.0, ay_g = (ay - ay_off) / 16384.0, az_g = (az - az_off) / 16384.0;
      float gx_dps = (gx - gx_off) / 131.0, gy_dps = (gy - gy_off) / 131.0;

      // Drift Correction Logic from your successful servo code
      roll = alpha * (roll + gx_dps * dt) + (1.0 - alpha) * (atan2(ay_g, az_g) * 57.2958);
      pitch = alpha * (pitch + gy_dps * dt) + (1.0 - alpha) * (atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 57.2958);

      // DRIFT CORRECTION CALCULATION
      float finalRoll = roll - initialRoll;
      float finalPitch = pitch - initialPitch;

      LoRa.beginPacket();
      LoRa.print("IMU:"); LoRa.print(finalRoll, 1); LoRa.print(","); LoRa.print(finalPitch, 1);
      LoRa.endPacket();
      
      Serial.print("TX Roll: "); Serial.print(finalRoll, 1); Serial.print(" Pitch: "); Serial.println(finalPitch, 1);
    }
    delay(sendDelayMs); 
  }

  // --- PHASE 2: LISTEN MODE (Your Original Logic) ---
  Serial.println(">>> TX LISTENING...");
  for(int i=0; i<3; i++) {
    LoRa.beginPacket();
    LoRa.print("CTRL:TX_LISTEN");
    LoRa.endPacket();
    delay(200);
  }

  unsigned long listenStart = millis();
  LoRa.receive(); 
  while (millis() - listenStart < CMD_LISTEN_TIME_MS) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String cmd = "";
      while (LoRa.available()) cmd += (char)LoRa.read();
      cmd.trim();
      if (cmd == "fast" || cmd == "balanced" || cmd == "slow") {
        currentMode = cmd;
        LoRa.beginPacket();
        LoRa.print("ACK:" + cmd);
        LoRa.endPacket();
        delay(1500);
        applyMode(currentMode);
        break; 
      }
    }
  }

  LoRa.beginPacket();
  LoRa.print("CTRL:IMU_RESUME");
  LoRa.endPacket();
  Serial.println(">>> RESUMING IMU");
  LoRa.receive(); 
}
