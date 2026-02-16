#include <HardwareSerial.h>

// --- Cấu hình Pin ---
#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

HardwareSerial FlowSerial(2);

// Thông số PWM
const int FREQ = 50;
const int RES = 16;
const int MAX_DUTY = 65535;

// Biến hỗ trợ mượt mà
float smoothP = 1500, smoothR = 1500;
float fX = 0, fY = 0;
unsigned long lastF = 0;

void setup() {
  Serial.begin(115200);
  
  // 1. Chống xung đột lúc khởi động: Đợi nguồn ổn định
  pinMode(L_PIN, OUTPUT); digitalWrite(L_PIN, LOW);
  pinMode(R_PIN, OUTPUT); digitalWrite(R_PIN, LOW);
  delay(1500); 

  // Cấu hình UART cho Optical Flow
  FlowSerial.begin(19200, SERIAL_8N1, 16, 17); 
  
  // Khởi tạo PWM cho Servo
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  
  pinMode(P_IN, INPUT_PULLDOWN);
  pinMode(R_IN, INPUT_PULLDOWN);

  Serial.println(">>> HE THONG DA SAN SANG (ONLY OPTICAL FLOW)");
}

// Hàm đọc RX Failsafe: Chống Servo quay liên tục khi mất sóng
long getRX(int pin) {
  long p = pulseIn(pin, HIGH, 25000);
  if (p < 950 || p > 2050) return 1500; // Trả về tâm nếu tín hiệu lỗi
  return p;
}

void loop() {
  // 2. Đọc dữ liệu Optical Flow (Giao tiếp UART)
  if (FlowSerial.available() > 0) {
    String data = FlowSerial.readStringUntil('\n');
    int comma = data.indexOf(',');
    if (comma != -1) {
      fX = data.substring(0, comma).toFloat();
      fY = data.substring(comma + 1).toFloat();
      lastF = millis();
    }
  }

  // Tự động ngắt hỗ trợ nếu cảm biến không gửi dữ liệu quá 0.5s
  if (millis() - lastF > 500) { fX = 0; fY = 0; }

  // 3. Đọc và lọc mượt tín hiệu điều khiển
  long rP = getRX(P_IN);
  long rR = getRX(R_IN);
  
  // Lọc nhiễu tần số cao cho Servo
  smoothP = (smoothP * 0.85) + (rP * 0.15);
  smoothR = (smoothR * 0.85) + (rR * 0.15);

  // 4. Tính toán Elevon Mixer
  float tP = map(smoothP, 1000, 2000, -30, 30);
  float tR = map(smoothR, 1000, 2000, -30, 30);

  // Cộng bù từ Optical Flow (Chống trôi)
  float outP = tP - (fY * 0.3);
  float outR = tR + (fX * 0.3);

  // 5. Xuất xung an toàn (Giới hạn hành trình để bảo vệ Servo)
  float vL = constrain(1500 + outP + outR, 1200, 1800);
  float vR = constrain(1500 - outP + outR, 1200, 1800);

  ledcWrite(L_PIN, (vL / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (vR / 20000.0) * MAX_DUTY);

  delay(10); 
}
