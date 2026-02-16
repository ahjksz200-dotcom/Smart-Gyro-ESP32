#include <HardwareSerial.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

HardwareSerial FlowSerial(2);

const int FREQ = 50, RES = 16, MAX_DUTY = 65535;
float fX = 0, fY = 0;
unsigned long lastF = 0;

void setup() {
  Serial.begin(115200);
  
  // Thử thay đổi Baudrate: Đa số module chạy 19200, một số chạy 115200
  FlowSerial.begin(19200, SERIAL_8N1, 16, 17); 
  
  pinMode(L_PIN, OUTPUT); digitalWrite(L_PIN, LOW);
  pinMode(R_PIN, OUTPUT); digitalWrite(R_PIN, LOW);
  delay(1000);

  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  pinMode(P_IN, INPUT_PULLDOWN);
  pinMode(R_IN, INPUT_PULLDOWN);

  Serial.println(">>> CHECKING OPTICAL FLOW DATA...");
}

void loop() {
  // --- PHẦN ĐỌC DỮ LIỆU BYTE (FRAME-BASED) ---
  while (FlowSerial.available() >= 9) { // Giả định Frame 9 byte phổ biến
    if (FlowSerial.read() == 0xFE) {    // Byte bắt đầu (Header) thường là 0xFE hoặc 0xAA
      if (FlowSerial.read() == 0x04) {  // Byte độ dài dữ liệu
        int16_t rawX = (FlowSerial.read() << 8) | FlowSerial.read();
        int16_t rawY = (FlowSerial.read() << 8) | FlowSerial.read();
        fX = rawX / 10.0; 
        fY = rawY / 10.0;
        lastF = millis();
        
        // In ra để kiểm tra ngay trên Terminal
        Serial.print("FlowX: "); Serial.print(fX);
        Serial.print(" | FlowY: "); Serial.println(fY);
      }
    }
  }

  // Ngắt hỗ trợ nếu mất kết nối
  if (millis() - lastF > 200) { fX = 0; fY = 0; }

  // --- ĐIỀU KHIỂN SERVO ---
  long pI = pulseIn(P_IN, HIGH, 25000);
  long rI = pulseIn(R_IN, HIGH, 25000);
  if (pI < 950 || pI > 2050) pI = 1500;
  if (rI < 950 || rI > 2050) rI = 1500;

  float tP = map(pI, 1000, 2000, -30, 30);
  float tR = map(rI, 1000, 2000, -30, 30);

  // Cộng bù Flow (Tăng gain lên 0.8 để thấy rõ sự chuyển động)
  float outP = tP - (fY * 0.8);
  float outR = tR + (fX * 0.8);

  float vL = constrain(1500 + outP + outR, 1200, 1800);
  float vR = constrain(1500 - outP + outR, 1200, 1800);

  ledcWrite(L_PIN, (vL / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (vR / 20000.0) * MAX_DUTY);

  delay(5);
}
