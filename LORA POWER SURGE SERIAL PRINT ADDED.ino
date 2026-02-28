#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <ESP32Servo.h>

#define SERVO_ROLL_PIN 13
#define SERVO_PITCH_PIN 12

Servo testservo_roll; 
Servo testservo_pitch; 

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

/* ---------- CALIBRATION & DATA ---------- */
float ax_off = 291.30, ay_off = -516.68, az_off = -902.19;
float gx_off = -273.41, gy_off = -193.28, gz_off = -252.62;

int16_t ax, ay, az, gx, gy, gz;
float roll = 0.0, pitch = 0.0;
float initialRoll = 0.0, initialPitch = 0.0; 

// FIXED: These must be GLOBAL so applyMode() can use them
float finalRoll = 0.0;
float finalPitch = 0.0;

const float alpha = 0.98;
unsigned long lastMicros = 0;

void checkLoRaHealth() {
  byte version = LoRa.readRegister(0x42);

  Serial.print("LoRa Version: 0x");
  Serial.println(version, HEX);

  if (version != 0x12) {
    Serial.println("⚠️ LoRa NOT responding! Possible power surge!");
    
    Serial.println("Reinitializing LoRa...");
    LoRa.end();
    delay(200);
    
    if (!LoRa.begin(433E6)) {   // change freq if needed
      Serial.println("LoRa reinit FAILED!");
    } else {
      Serial.println("LoRa reinit SUCCESS");
    }
  }
}


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
  if (mode == "fast") { 
    LoRa.setSpreadingFactor(7); 
    LoRa.setSignalBandwidth(250E3); 
  }
  else if (mode == "balanced") { 
    LoRa.setSpreadingFactor(9); 
    LoRa.setSignalBandwidth(125E3); 
  }
  else if (mode == "slow") { 
    LoRa.setSpreadingFactor(12); 
    LoRa.setSignalBandwidth(125E3); 
  }
  else if (mode == "stabilize"){
      // Real-time Example: Mapping IMU movement to Servo movement
      int servo_angle_roll = 90 + (int)finalRoll;   
      int servo_angle_pitch = 90 - (int)finalPitch;    
      
      // Keep angles within safe 0-180 range
      servo_angle_roll = constrain(servo_angle_roll, 0, 180);
      servo_angle_pitch = constrain(servo_angle_pitch, 0, 180);

      testservo_roll.write(servo_angle_roll);  
      testservo_pitch.write(servo_angle_pitch); 
      Serial.println("Stabilize Command Applied to Servos");

  }
  LoRa.setCodingRate4(8);
  Serial.println("MODE SET: " + mode);
}

void setup() {
  Serial.begin(115200);
  
  // FIXED: Attach servos ONCE in setup to prevent ESP32 memory crashes
  testservo_roll.attach(SERVO_ROLL_PIN, 500, 2400); 
  testservo_pitch.attach(SERVO_PITCH_PIN, 500, 2400);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); 
  writeMPU(PWR_MGMT_1, 0x00);
  delay(500);

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
  
  Serial.println("LoRa: SPI Begin");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  
  Serial.println("LoRa: Set Pins");
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(LORA_BAND)) { Serial.println("LoRa FAIL"); while (1); }
  Serial.println("LoRa BEGIN SUCCESS");

  applyMode(currentMode);
  lastMicros = micros();
  Serial.println("TX READY");
}

void loop() {
  // --- PHASE 1: IMU TRANSMIT (60 Seconds) ---
  unsigned long imuStart = millis();
  
  int sendDelayMs = 200; 
  if (currentMode == "fast") sendDelayMs = 100;
  else if (currentMode == "slow") sendDelayMs = 1000;
  
  Serial.print("Send delay: "); Serial.println(sendDelayMs);

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

      roll = alpha * (roll + gx_dps * dt) + (1.0 - alpha) * (atan2(ay_g, az_g) * 57.2958);
      pitch = alpha * (pitch + gy_dps * dt) + (1.0 - alpha) * (atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 57.2958);

      finalRoll = roll - initialRoll;
      finalPitch = pitch - initialPitch;

      // Real-time stabilization if mode is set
      if (currentMode == "stabilize") {
          testservo_roll.write(90 + (int)finalRoll);
          testservo_pitch.write(90 - (int)finalPitch);
      }
    
      Serial.println("LoRa TX: BeginPacket");
      int txStatus = LoRa.beginPacket();
      checkLoRaHealth();


      if (!txStatus) {
         Serial.println("LoRa ERROR: beginPacket failed");
      }

      LoRa.print("IMU:"); LoRa.print(finalRoll, 1); LoRa.print(","); LoRa.print(finalPitch, 1);
      Serial.println("LoRa TX: EndPacket START");
      int result = LoRa.endPacket();
      checkLoRaHealth();

      Serial.println("LoRa TX: EndPacket RETURNED");
      if (result == 0) {
      Serial.println("LoRa ERROR: endPacket failed");
      } else {
      Serial.println("LoRa TX SUCCESS");
      }
      
      Serial.print("TX Roll: "); Serial.print(finalRoll, 1); Serial.print(" Pitch: "); Serial.println(finalPitch, 1);
    }
    delay(sendDelayMs); 
  }

  // --- PHASE 2: LISTEN MODE (15 Seconds) ---
  Serial.println(">>> TX LISTENING...");
  for(int i=0; i<3; i++) {
    LoRa.beginPacket();
    LoRa.print("CTRL:TX_LISTEN");
    LoRa.endPacket();
    delay(200);
  }

  unsigned long listenStart = millis();
  Serial.println("LoRa: Switching to RX mode");
  LoRa.receive(); 
  Serial.println("LoRa: RX mode set");
  
  while (millis() - listenStart < CMD_LISTEN_TIME_MS) {
    Serial.println("LoRa: Checking for packet...");
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String cmd = "";
      while (LoRa.available()) {cmd += (char)LoRa.read();
      cmd.trim();}
      if (cmd == "fast" || cmd == "balanced" || cmd == "slow" || cmd == "stabilize") {
        currentMode = cmd;
        LoRa.beginPacket();
        LoRa.print("ACK:" + cmd);
        LoRa.endPacket();
        

        delay(1000); // Reduced from 1500 to prevent long blocking
        applyMode(currentMode);
        break; 
      }
    }
  }
  Serial.println("LoRa: Preparing to return to TX");
  LoRa.beginPacket();
  checkLoRaHealth();

  Serial.println("LoRa: BeginPacket test");
if (!LoRa.beginPacket()) {
  Serial.println("LoRa ERROR: Cannot re-enter TX mode");
}

  LoRa.print("CTRL:IMU_RESUME");
  Serial.println("LoRa: EndPacket test START");
LoRa.endPacket();
checkLoRaHealth();

Serial.println("LoRa: EndPacket test DONE");
  Serial.println(">>> RESUMING IMU");
  
}