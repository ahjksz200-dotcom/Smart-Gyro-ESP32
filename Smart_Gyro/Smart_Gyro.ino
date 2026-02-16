#include <Wire.h>

// --- Cấu hình Pin ---
#define MPU_ADDR 0x68
#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

// --- Cấu hình PWM Core 3.x ---
const int FREQ = 50;
const int RES = 16;
const int MAX_DUTY = 65535;

struct PID { float kp, ki, kd, i, last; };
PID pP = {16.0, 0.05, 2.0}, rP = {16.0, 0.05, 2.0}, yP = {12.0, 0.02, 1.0};

float ax, ay, az, gx, gy, gz, oX, oY, oZ, r = 0, p = 0;
unsigned long lT;

// --- Bộ lọc RX (Tránh Servo quay liên tục) ---
long filterRX(int pin) {
  long p = pulseIn(pin, HIGH, 25000);
  if (p < 900 || p > 2100) return 1500; // Trả về trung tâm nếu nhiễu hoặc mất sóng
  return p;
}

void setup() {
  Serial.begin(115200);
  
  // 1. Khởi động mềm: Giữ Servo ở 1500us trước khi kích hoạt PID
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  ledcWrite(L_PIN, (1500.0 / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (1500.0 / 20000.0) * MAX_DUTY);
  
  delay(2000); 
  Wire.begin(21, 22);
  Wire.setClock(400000);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();

  // 2. Calib Gyro với lọc nhiễu tĩnh
  Serial.println(">>> CALIBRATING...");
  float sX=0, sY=0, sZ=0;
  for(int i=0; i<200; i++){
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    sX += (int16_t)(Wire.read()<<8|Wire.read())/131.0;
    sY += (int16_t)(Wire.read()<<8|Wire.read())/131.0;
    sZ += (int16_t)(Wire.read()<<8|Wire.read())/131.0;
    delay(2);
  }
  oX=sX/200; oY=sY/200; oZ=sZ/200;

  pinMode(P_IN, INPUT_PULLDOWN); // Ép chân RX xuống Low nếu lỏng dây
  pinMode(R_IN, INPUT_PULLDOWN);
  
  lT = micros();
  Serial.println(">>> SYSTEM ARMED & READY!");
}

float calcPID(float t, float a, PID &d, float dt) {
  float e = t - a;
  // Anti-windup: Chỉ tính I khi máy bay chưa kịch lái
  if (abs(e) > 0.5) d.i = constrain(d.i + e * dt, -100, 100);
  else d.i = d.i * 0.95; // Reset I dần khi đã cân bằng để tránh trôi
  
  float dv = (e - d.last) / dt;
  d.last = e;
  return (e * d.kp) + (d.i * d.ki) + (dv * d.kd);
}

void loop() {
  float dt = (micros() - lT) / 1000000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.01;
  lT = micros();

  // 3. Đọc cảm biến
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  ax = (int16_t)(Wire.read()<<8|Wire.read())/16384.0;
  ay = (int16_t)(Wire.read()<<8|Wire.read())/16384.0;
  az = (int16_t)(Wire.read()<<8|Wire.read())/16384.0;
  Wire.read(); Wire.read();
  gx = ((int16_t)(Wire.read()<<8|Wire.read())/131.0)-oX;
  gy = ((int16_t)(Wire.read()<<8|Wire.read())/131.0)-oY;
  gz = ((int16_t)(Wire.read()<<8|Wire.read())/131.0)-oZ;

  // Lọc bổ trợ 98/2
  p = 0.98*(p + gy*dt) + 0.02*(atan2(ax, sqrt(ay*ay+az*az))*180/PI);
  r = 0.98*(r + gx*dt) + 0.02*(atan2(ay, az)*180/PI);

  // 4. Đọc RX với bộ lọc Failsafe mới
  long pI = filterRX(P_IN);
  long rI = filterRX(R_IN);
  
  float tP = map(pI, 1000, 2000, -35, 35);
  float tR = map(rI, 1000, 2000, -35, 35);

  // 5. PID & Mixer
  float oP = calcPID(tP, p, pP, dt);
  float oR = calcPID(tR, r, rP, dt);
  float oY = calcPID(0, gz, yP, dt);

  float vL = constrain(1500 + oP + oR + oY, 1100, 1900);
  float vR = constrain(1500 - oP + oR + oY, 1100, 1900);

  // Xuất xung mượt
  ledcWrite(L_PIN, (vL / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (vR / 20000.0) * MAX_DUTY);

  delay(5);
}
