
#include <Wire.h>
#include <ESP32Servo.h>

/* ================= MPU CONFIG ================= */
#define MPU_ADDR        0x68
#define PWR_MGMT_1      0x6B
#define ACCEL_XOUT_H    0x3B
#define SDA_PIN         21
#define SCL_PIN         22

/* ================= SERVO CONFIG ================= */
#define SERVO_ROLL_PIN   13
#define SERVO_PITCH_PIN  12

Servo rollServo;
Servo pitchServo;

/* ===== 270 DEGREE SERVO SETTINGS ===== */
int servoCenter = 135;     // Center for 270° servo
int servoMin = 0;
int servoMax = 270;

/* ================= CONTROL ================= */
float Kp_roll  = 1.5;
float Kp_pitch = 2.8;

float rollOffsetCorrection = 26.0;

float commandFilter = 0.4;

float rollCommandFiltered = 135;
float pitchCommandFiltered = 135;

/* ================= MAHONY FILTER ================= */
float twoKp = 2.0f * 2.0f;
float twoKi = 0.0f;

/* ================= QUATERNION & AHRS ================= */
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float roll = 0;
float pitch = 0;
float initialRoll = 0;
float initialPitch = 0;

const float dt = 0.01;
unsigned long lastLoopTime;

/* ================================================= */

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();

  // Attach with extended pulse range
  rollServo.attach(SERVO_ROLL_PIN, 500, 2400);
  pitchServo.attach(SERVO_PITCH_PIN, 500, 2400);

  rollServo.write(servoCenter);
  pitchServo.write(servoCenter);

  Serial.println("Calibrating... Keep stationary");
  delay(2000);

  for(int i=0; i<1500; i++){
    updateMahony();
    delay(2);
  }

  initialRoll = roll + rollOffsetCorrection;
  initialPitch = pitch;

  rollCommandFiltered = servoCenter;
  pitchCommandFiltered = servoCenter;

  lastLoopTime = millis();

  Serial.println("System Ready - 270 Degree Mode Active");
}

void loop() {
  if(millis() - lastLoopTime >= 10) {
    lastLoopTime += 10;
    updateMahony();

    float rollError  = roll  - initialRoll;
    float pitchError = pitch - initialPitch;

    if(abs(rollError) < 0.4) rollError = 0;
    if(abs(pitchError) < 0.4) pitchError = 0;

    float rollCommand  = servoCenter + (Kp_roll  * rollError);
    float pitchCommand = servoCenter - (Kp_pitch * pitchError);

    // Constrain for 270 degree servo
    rollCommand  = constrain(rollCommand, servoMin, servoMax);
    pitchCommand = constrain(pitchCommand, servoMin, servoMax);

    rollCommandFiltered  = (commandFilter * rollCommandFiltered) +
                           ((1.0 - commandFilter) * rollCommand);

    pitchCommandFiltered = (commandFilter * pitchCommandFiltered) +
                           ((1.0 - commandFilter) * pitchCommand);

    rollServo.write((int)rollCommandFiltered);
    pitchServo.write((int)pitchCommandFiltered);

    static int pCount = 0;
    if(++pCount >= 20){
      pCount = 0;
      Serial.print("Roll_Err: "); Serial.print(rollError, 1);
      Serial.print(" | Pitch_Err: "); Serial.println(pitchError, 1);
    }
  }
}

/* ================= MAHONY FILTER ================= */

void updateMahony() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  if(Wire.available() != 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
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

  gx_r += twoKp * ex;
  gy_r += twoKp * ey;
  gz_r += twoKp * ez;

  float qa = q0, qb = q1, qc = q2;

  q0 += (-qb * gx_r - qc * gy_r - q3 * gz_r) * 0.5f * dt;
  q1 += (qa * gx_r + qc * gz_r - q3 * gy_r) * 0.5f * dt;
  q2 += (qa * gy_r - qb * gz_r + q3 * gx_r) * 0.5f * dt;
  q3 += (qa * gz_r + qb * gy_r - qc * gx_r) * 0.5f * dt;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

  roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 57.2958f;
  pitch = asin(2*(q0*q2 - q3*q1)) * 57.2958f;
}
