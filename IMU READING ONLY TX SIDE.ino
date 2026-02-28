
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

/* =========================
   MPU6050 DEFINITIONS
   ========================= */
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

/* =========================
   ESP32 I2C PINS
   ========================= */
#define SDA_PIN 21
#define SCL_PIN 22

/* =========================
   LORA PINS
   ========================= */
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS   27
#define LORA_RST  14
#define LORA_DIO0 26

#define LORA_FREQ 433E6

/* =========================
   IMU OFFSETS (YOURS)
   ========================= */
float ax_off = 2776.09;
float ay_off = 872.52;
float az_off = 2.95;

float gx_off = 3306.80;
float gy_off = -11.05;
float gz_off = 53.65;

/* =========================
   RAW VARIABLES
   ========================= */
int16_t ax, ay, az;
int16_t gx, gy, gz;

/* =========================
   FUSION VARIABLES
   ========================= */
float roll = 0.0;
float pitch = 0.0;

unsigned long lastMicros = 0;
const float alpha = 0.98;

/* =========================
   MPU LOW LEVEL
   ========================= */
void writeMPU(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void readMPU(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, len);

  for (uint8_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }
}

void setup() {
  Serial.begin(115200);

  /* -------- I2C INIT -------- */
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  writeMPU(PWR_MGMT_1, 0x00);
  delay(100);

  /* -------- LORA INIT -------- */
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed");
    while (1);
  }

  lastMicros = micros();
  Serial.println("TX READY WITH FUSION");
}

void loop() {

  /* =========================
     LOOP TIMING
     ========================= */
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6;
  lastMicros = now;

  if (dt <= 0 || dt > 0.1) return;

  /* =========================
     READ RAW IMU
     ========================= */
  uint8_t rawData[14];
  readMPU(ACCEL_XOUT_H, rawData, 14);

  ax = (rawData[0] << 8) | rawData[1];
  ay = (rawData[2] << 8) | rawData[3];
  az = (rawData[4] << 8) | rawData[5];

  gx = (rawData[8] << 8) | rawData[9];
  gy = (rawData[10] << 8) | rawData[11];
  gz = (rawData[12] << 8) | rawData[13];

  /* =========================
     OFFSET + SCALE
     ========================= */
  float ax_g = (ax - ax_off) / 16384.0;
  float ay_g = (ay - ay_off) / 16384.0;
  float az_g = (az - az_off) / 16384.0;

  float gx_dps = (gx - gx_off) / 131.0;
  float gy_dps = (gy - gy_off) / 131.0;
  float gz_dps = (gz - gz_off) / 131.0;

  /* =========================
     ACCEL ANGLES
     ========================= */
  float roll_acc  = atan2(ay_g, az_g) * 57.2958;
  float pitch_acc = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 57.2958;

  /* =========================
     GYRO INTEGRATION
     ========================= */
  float roll_gyro  = roll  + gx_dps * dt;
  float pitch_gyro = pitch + gy_dps * dt;

  /* =========================
     SENSOR FUSION
     ========================= */
  roll  = alpha * roll_gyro  + (1.0 - alpha) * roll_acc;
  pitch = alpha * pitch_gyro + (1.0 - alpha) * pitch_acc;

  /* =========================
     SEND VIA LORA (FINAL ANGLES)
     ========================= */
  LoRa.beginPacket();
  LoRa.print(roll, 2);  LoRa.print(",");
  LoRa.print(pitch, 2);
  LoRa.endPacket();

  /* =========================
     DEBUG
     ========================= */
  Serial.print("ROLL: ");
  Serial.print(roll, 2);
  Serial.print("  PITCH: ");
  Serial.println(pitch, 2);

  delay(10);   // ~100 Hz fusion loop
}
