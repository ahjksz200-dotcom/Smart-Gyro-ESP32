#include <Wire.h>

// --- Cấu hình Pin ---
#define MPU_ADDR 0x68
#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

// --- Cấu hình PWM cho Core 3.x ---
const int FREQ = 50;
const int RES = 16;
const int MAX_DUTY = 65535; // 2^16 - 1

// --- Cấu hình PID (Có thể tinh chỉnh tùy loại máy bay) ---
struct PID { float kp, ki, kd, i, last; };
PID pP = {18.0, 0.04, 2.2}; // Pitch: Tăng P một chút để đầm hơn
PID rP = {16.0, 0.04, 2.0}; // Roll
PID yP = {14.0, 0.02, 1.2}; // Yaw (Kháng gió ngang)

// --- Biến hệ thống ---
float ax, ay, az, gx, gy, gz, oX, oY, oZ, r = 0, p = 0;
unsigned long lT;
const float deadband = 2.0; // Vùng đệm nhiễu cho tay điều khiển

void setup() {
  Serial.begin(115200);
  delay(2000); // Ổn định nguồn tránh lỗi "invalid header"
  Wire.begin(21, 22);
  Wire.setClock(400000); // Tăng tốc độ I2C lên 400kHz để mượt hơn
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();

  // 1. Cân bằng (Calibration): Tính toán sai số Gyro
  Serial.println(">>> CALIBRATING... DO NOT MOVE!");
  float sX=0, sY=0, sZ=0;
  for(int i=0; i<400; i++){ // Tăng số mẫu để Calib chính xác hơn
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    sX += (int16_t)(Wire.read()<<8|Wire.read())/131.0;
    sY += (int16_t)(Wire.read()<<8|Wire.read())/131.0;
    sZ += (int16_t)(Wire.read()<<8|Wire.read())/131.0;
    delay(3);
  }
  oX=sX/400; oY=sY/400; oZ=sZ/400;

  // 2. Thiết lập PWM v3.x
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);

  pinMode(P_IN, INPUT); pinMode(R_IN, INPUT);
  lT = micros();
  Serial.println(">>> SYSTEM ARMED & READY!");
}

float calcPID(float t, float a, PID &d, float dt) {
  float e = t - a;
  if (abs(e) < 0.5) e = 0; // Vùng đệm nhỏ để servo không bị run (jitter)
  d.i = constrain(d.i + e * dt, -150, 150); // Giới hạn I để tránh vọt lố
  float dv = (e - d.last) / dt;
  d.last = e;
  return (e * d.kp) + (d.i * d.ki) + (dv * d.kd);
}

void loop() {
  float dt = (micros() - lT) / 1000000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.005; // Giới hạn dt để PID không bị nhảy loạn
  lT = micros();

  // 3. Đọc dữ liệu MPU6500 và Lọc Complementary
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  ax = (int16_t)(Wire.read()<<8|Wire.read())/16384.0;
  ay = (int16_t)(Wire.read()<<8|Wire.read())/16384.0;
  az = (int16_t)(Wire.read()<<8|Wire.read())/16384.0;
  Wire.read(); Wire.read();
  gx = ((int16_t)(Wire.read()<<8|Wire.read())/131.0)-oX;
  gy = ((int16_t)(Wire.read()<<8|Wire.read())/131.0)-oY;
  gz = ((int16_t)(Wire.read()<<8|Wire.read())/131.0)-oZ;

  p = 0.98*(p + gy*dt) + 0.02*(atan2(ax, sqrt(ay*ay+az*az))*180/PI);
  r = 0.98*(r + gx*dt) + 0.02*(atan2(ay, az)*180/PI);

  // 4. Failsafe: Tự động giữ thăng bằng khi mất sóng
  long pI = pulseIn(P_IN, HIGH, 30000); // 30ms timeout
  long rI = pulseIn(R_IN, HIGH, 30000);
  
  float tP = 0, tR = 0;
  if (pI > 900 && pI < 2100) tP = map(pI, 1000, 2000, -35, 35);
  if (rI > 900 && rI < 2100) tR = map(rI, 1000, 2000, -35, 35);

  // 5. Tính toán PID cho 3 trục
  float oP = calcPID(tP, p, pP, dt);
  float oR = calcPID(tR, r, rP, dt);
  float oY = calcPID(0, gz, yP, dt); // Kháng Yaw tự động

  // 6. Mixer Elevon với bảo vệ Servo
  float vL = constrain(1500 + oP + oR + oY, 1050, 1950);
  float vR = constrain(1500 - oP + oR + oY, 1050, 1950);

  // Xuất PWM với độ phân giải cao
  ledcWrite(L_PIN, (vL / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (vR / 20000.0) * MAX_DUTY);

  delay(4); // Tốc độ vòng lặp ổn định ~250Hz
}
