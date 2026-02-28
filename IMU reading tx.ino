
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

/* =========================
   MPU6050 DEFINITIONS
   ========================= */
#define MPU_ADDR       0x68   // CHANGE TO 0x69 IF AD0 = HIGH
#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define WHO_AM_I       0x75

#define SDA_PIN 21
#define SCL_PIN 22

/* =========================
   LORA PIN DEFINITIONS
   ========================= */
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS    27
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  433E6

/* =========================
   RAW VARIABLES
   ========================= */
int16_t ax, ay, az, gx, gy, gz;

/* =========================
   MPU FUNCTIONS
   ========================= */
void writeMPU(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

bool readMPU(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t count = Wire.requestFrom(MPU_ADDR, len);
  if (count != len) return false;

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

/* =========================
   LORA MODES
   ========================= */
void applyMode(String mode) {
  if (mode == "fast") {
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(250E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(14);
  }
  else if (mode == "balanced") {
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(6);
    LoRa.setTxPower(17);
  }
  else if (mode == "slow") {
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(8);
    LoRa.setTxPower(20);
  }
}

/* =========================
   SETUP
   ========================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  /* ---------- I2C INIT ---------- */
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  /* ---------- WHO_AM_I CHECK ---------- */
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);

  if (!Wire.available()) {
    Serial.println("ERROR: MPU6050 not responding");
    while (1);
  }

  uint8_t who = Wire.read();
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  if (who != 0x68) {
    Serial.println("ERROR: Invalid MPU6050 ID");
    while (1);
  }

  /* ---------- MPU CONFIG ---------- */
  writeMPU(PWR_MGMT_1, 0x00);     // Wake up
  delay(100);
  writeMPU(GYRO_CONFIG, 0x00);    // ±250 dps
  writeMPU(ACCEL_CONFIG, 0x00);   // ±2g
  delay(100);

  /* ---------- LORA INIT ---------- */
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed");
    while (1);
  }

  applyMode("balanced");
  LoRa.receive();

  Serial.println("SYSTEM READY");
}

/* =========================
   LOOP
   ========================= */
void loop() {

  /* ---------- COMMAND RX ---------- */
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String cmd = "";
    while (LoRa.available()) cmd += (char)LoRa.read();
    cmd.trim();
    cmd.toLowerCase();

    Serial.print("CMD RX: ");
    Serial.println(cmd);

    LoRa.idle();
    LoRa.beginPacket();
    LoRa.print("ACK:" + cmd);
    LoRa.endPacket();

    applyMode(cmd);
    LoRa.receive();
  }

  /* ---------- READ IMU ---------- */
  uint8_t raw[14];
  if (!readMPU(ACCEL_XOUT_H, raw, 14)) {
    Serial.println("ERROR: MPU READ FAILED");
    delay(500);
    return;
  }

  ax = (raw[0] << 8) | raw[1];
  ay = (raw[2] << 8) | raw[3];
  az = (raw[4] << 8) | raw[5];
  gx = (raw[8] << 8) | raw[9];
  gy = (raw[10] << 8) | raw[11];
  gz = (raw[12] << 8) | raw[13];

  /* ---------- CONVERT ---------- */
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float gx_d = gx / 131.0;
  float gy_d = gy / 131.0;
  float gz_d = gz / 131.0;

  /* ---------- TRANSMIT ---------- */
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print(ax_g, 3); LoRa.print(",");
  LoRa.print(ay_g, 3); LoRa.print(",");
  LoRa.print(az_g, 3); LoRa.print(",");
  LoRa.print(gx_d, 3); LoRa.print(",");
  LoRa.print(gy_d, 3); LoRa.print(",");
  LoRa.print(gz_d, 3);
  LoRa.endPacket();
  LoRa.receive();

  /* ---------- SERIAL ---------- */
  Serial.print("TX IMU | ");
  Serial.print(ax_g, 3); Serial.print(" ");
  Serial.print(ay_g, 3); Serial.print(" ");
  Serial.print(az_g, 3); Serial.print(" | ");
  Serial.print(gx_d, 3); Serial.print(" ");
  Serial.print(gy_d, 3); Serial.print(" ");
  Serial.println(gz_d, 3);

  delay(500);
}
