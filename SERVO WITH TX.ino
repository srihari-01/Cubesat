
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>

/* =========================================================================
   1. HARDWARE PINS
   ========================================================================= */
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_BAND 433E6

#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define SDA_PIN 21
#define SCL_PIN 22

#define SERVO_ROLL_PIN  13
#define SERVO_PITCH_PIN 12

/* =========================================================================
   2. SHARED DATA & AHRS GLOBALS
   ========================================================================= */
volatile float sharedRoll = 0.0;
volatile float sharedPitch = 0.0;
volatile bool systemReady = false; 

// Mahony Filter & Orientation
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float roll = 0, pitch = 0;
float initialRoll = 0, initialPitch = 0;
const float dt_const = 0.01; 

// 270 Degree Servo Settings
int servoCenter = 135; 
int servoMin = 0;
int servoMax = 270;
float Kp_roll = 1.5;
float Kp_pitch = 2.8;
float rollOffsetCorrection = 26.0;
float commandFilter = 0.4;

/* =========================================================================
   3. HELPER FUNCTIONS
   ========================================================================= */
void writeMPU(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(data); Wire.endTransmission();
}

void setLoRaParams(String mode, int &interval) {
  if (mode == "fast") { 
    LoRa.setSpreadingFactor(7); LoRa.setSignalBandwidth(250E3); interval = 50; 
  } else if (mode == "balanced") { 
    LoRa.setSpreadingFactor(9); LoRa.setSignalBandwidth(125E3); interval = 200; 
  } else if (mode == "slow") { 
    LoRa.setSpreadingFactor(12); LoRa.setSignalBandwidth(125E3); interval = 1000; 
  }
  LoRa.setCodingRate4(8);
}

void updateMahony() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  if(Wire.available() != 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip Temp
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  float ax_g = ax / 16384.0f;
  float ay_g = ay / 16384.0f;
  float az_g = az / 16384.0f;
  float gx_r = gx * (PI / 180.0f) / 131.0f;
  float gy_r = gy * (PI / 180.0f) / 131.0f;
  float gz_r = gz * (PI / 180.0f) / 131.0f;

  float norm = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  if (norm == 0) return;
  ax_g /= norm; ay_g /= norm; az_g /= norm;

  float vx = 2 * (q1 * q3 - q0 * q2);
  float vy = 2 * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = (ay_g * vz - az_g * vy);
  float ey = (az_g * vx - ax_g * vz);
  float ez = (ax_g * vy - ay_g * vx);

  float twoKp = 4.0f; // 2 * Kp
  gx_r += twoKp * ex; gy_r += twoKp * ey; gz_r += twoKp * ez;

  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gx_r - qc * gy_r - q3 * gz_r) * 0.5f * dt_const;
  q1 += (qa * gx_r + qc * gz_r - q3 * gy_r) * 0.5f * dt_const;
  q2 += (qa * gy_r - qb * gz_r + q3 * gx_r) * 0.5f * dt_const;
  q3 += (qa * gz_r + qb * gy_r - qc * gx_r) * 0.5f * dt_const;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

  roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 57.2958f;
  pitch = asin(2*(q0*q2 - q3*q1)) * 57.2958f;
}

/* =========================================================================
   4. CORE 1 TASK: SERVO & AHRS (NEW LOGIC)
   ========================================================================= */
void TaskServoIMU(void *pvParameters) {
  Servo rollS, pitchS;
  rollS.attach(SERVO_ROLL_PIN, 500, 2400);
  pitchS.attach(SERVO_PITCH_PIN, 500, 2400);

  float rollFilt = servoCenter;
  float pitchFilt = servoCenter;
  unsigned long lastMicros = micros();

  while(!systemReady) { vTaskDelay(100); }

  Serial.println("[Core 1] >>> 270° MAHONY SERVO TASK STARTED <<<");

  while(true) {
    if(micros() - lastMicros >= 10000) { // 100Hz Loop
      lastMicros = micros();
      updateMahony();

      float rollError  = roll  - initialRoll;
      float pitchError = pitch - initialPitch;

      // Deadband
      if(abs(rollError) < 0.4) rollError = 0;
      if(abs(pitchError) < 0.4) pitchError = 0;

      float rCmd = servoCenter + (Kp_roll * rollError);
      float pCmd = servoCenter - (Kp_pitch * pitchError);

      rCmd = constrain(rCmd, servoMin, servoMax);
      pCmd = constrain(pCmd, servoMin, servoMax);

      // Exponential Filter
      rollFilt = (commandFilter * rollFilt) + ((1.0 - commandFilter) * rCmd);
      pitchFilt = (commandFilter * pitchFilt) + ((1.0 - commandFilter) * pCmd);

      rollS.write((int)rollFilt);
      pitchS.write((int)pitchFilt);

      // Update shared data for LoRa transmission
      sharedRoll = rollError;
      sharedPitch = pitchError;
    }
    vTaskDelay(1); 
  }
}

/* =========================================================================
   5. CORE 0 TASK: LORA COMM (UNTOUCHED)
   ========================================================================= */
void TaskLoRa(void *pvParameters) {
  esp_task_wdt_delete(NULL); 
  String currentMode = "balanced";
  int sendInterval = 200;
  
  while(!systemReady) { vTaskDelay(100); }
  setLoRaParams(currentMode, sendInterval);
  
  unsigned long phaseStartTime = millis();
  unsigned long lastTxTime = 0;
  bool isTxPhase = true;
  
  while(true) {
    unsigned long currentTime = millis();
    if (isTxPhase) {
      if (currentTime - phaseStartTime > 60000) {
        isTxPhase = false; phaseStartTime = currentTime;
        for(int i=0; i<3; i++) { LoRa.beginPacket(); LoRa.print("SYNC:LISTEN_START"); LoRa.endPacket(); delay(50); }
        LoRa.receive(); 
      } else {
        if (currentTime - lastTxTime > sendInterval) {
          LoRa.beginPacket();
          LoRa.print("IMU:"); LoRa.print(sharedRoll, 1); LoRa.print(","); LoRa.print(sharedPitch, 1);
          LoRa.endPacket();
          lastTxTime = currentTime;
        }
      }
    } else {
      if (currentTime - phaseStartTime > 15000) {
        isTxPhase = true; phaseStartTime = currentTime;
        LoRa.beginPacket(); LoRa.print("SYNC:RESUME"); LoRa.endPacket();
        setLoRaParams(currentMode, sendInterval); 
      } else {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
          String cmd = ""; while (LoRa.available()) cmd += (char)LoRa.read();
          cmd.trim();
          if (cmd == "fast" || cmd == "balanced" || cmd == "slow") {
            LoRa.beginPacket(); LoRa.print("ACK:" + cmd); LoRa.endPacket();
            currentMode = cmd; setLoRaParams(currentMode, sendInterval);
            delay(50); LoRa.receive(); 
          }
        }
      }
    }
    vTaskDelay(10); 
  }
}

/* =========================================================================
   6. MAIN SETUP
   ========================================================================= */
void setup() {
  Serial.begin(115200);
  esp_task_wdt_init(30, true);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  writeMPU(PWR_MGMT_1, 0x00);
  
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) { Serial.println("LoRa FAIL!"); while(1); }

  Serial.println("Calibrating Mahony AHRS...");
  for(int i=0; i<1500; i++) {
    updateMahony();
    delay(2);
  }
  initialRoll = roll + rollOffsetCorrection;
  initialPitch = pitch;

  systemReady = true;
  xTaskCreatePinnedToCore(TaskServoIMU, "ServoTask", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskLoRa, "LoRaTask", 10000, NULL, 1, NULL, 0);
}

void loop() { vTaskDelete(NULL); }

SERVO RX
#include <SPI.h>
#include <LoRa.h>

/* =========================================================================
   1. HARDWARE PINS
   ========================================================================= */
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_BAND 433E6 

/* =========================================================================
   2. VARIABLES
   ========================================================================= */
String currentMode = "balanced";
bool txIsListening = false;
unsigned long lastPacketTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n========================================");
  Serial.println("   MAHONY AHRS LORA RECEIVER READY      ");
  Serial.println("========================================\n");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("[ERROR] LoRa Module Fail! Check wiring.");
    while (1);
  }
  
  // Start with "Balanced" settings to match Transmitter
  LoRa.setSpreadingFactor(9); 
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();       
  
  LoRa.receive();
  Serial.println("[SYSTEM] Monitoring Core 1 Stability Data...");
}

void loop() {
  // --- A. PACKET HANDLING ---
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    lastPacketTime = millis();
    String msg = "";
    while (LoRa.available()) msg += (char)LoRa.read();
    msg.trim();

    int rssi = LoRa.packetRssi();

    // 1. Check for Synchronization Messages
    if (msg == "SYNC:LISTEN_START") {
      txIsListening = true;
      Serial.println("\n>>> [WINDOW OPEN] TX is listening. Commands: 'fast', 'balanced', 'slow'");
    }
    else if (msg == "SYNC:RESUME") {
      txIsListening = false;
      Serial.println("\n>>> [WINDOW CLOSED] TX resuming stabilization.");
    }
    // 2. Check for Mode Acknowledgements
    else if (msg.startsWith("ACK:")) {
      String newMode = msg.substring(4);
      Serial.print("\n>>> [MODE CHANGED]: "); Serial.println(newMode);
      updateRadioSettings(newMode);
      LoRa.receive(); 
    }
    // 3. Parse IMU Data (IMU:Roll,Pitch)
    else if (msg.startsWith("IMU:")) {
      parseIMUData(msg.substring(4), rssi);
    }
  }

  // --- B. SERIAL COMMAND INPUT ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (txIsListening) {
       Serial.print("[TX] Sending Command -> "); Serial.println(input);
       LoRa.beginPacket();
       LoRa.print(input);
       LoRa.endPacket();
       LoRa.receive(); // Go back to listening
    } else {
       Serial.println("[!] TX BUSY. Wait for 'SYNC:LISTEN_START'");
    }
  }

  // --- C. TIMEOUT ALERT ---
  if (millis() - lastPacketTime > 5000 && lastPacketTime != 0) {
    static unsigned long lastWarn = 0;
    if(millis() - lastWarn > 2000) {
       Serial.println("[ALERT] Signal Lost - Check Transmitter Power");
       lastWarn = millis();
    }
  }
}

/* =========================================================================
   HELPER: Parse and Display IMU Data
   ========================================================================= */
void parseIMUData(String data, int rssi) {
  // Format: "Roll,Pitch"
  int commaIndex = data.indexOf(',');
  if (commaIndex > 0) {
    String r = data.substring(0, commaIndex);
    String p = data.substring(commaIndex + 1);
    
    Serial.print("RSSI: "); Serial.print(rssi);
    Serial.print("dBm | ROLL: "); Serial.print(r);
    Serial.print(" | PITCH: "); Serial.println(p);
  }
}

/* =========================================================================
   HELPER: Update Radio Parameters
   ========================================================================= */
void updateRadioSettings(String mode) {
  if (mode == "fast") {
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(250E3);
  } else if (mode == "balanced") {
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
  } else if (mode == "slow") {
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
  }
}
