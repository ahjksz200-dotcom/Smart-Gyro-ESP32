#include <HardwareSerial.h>

// --- Cấu hình Pin ---
#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

HardwareSerial FlowSerial(2);

// Cấu hình PWM
const int FREQ = 50, RES = 16, MAX_DUTY = 65535;

// Biến điều khiển mượt
float smoothP = 1500, smoothR = 1500;
float flowX = 0, flowY = 0;
unsigned long lastFlowTime = 0;

void setup() {
  Serial.begin(115200);
  
  // 1. KHỞI ĐỘNG MỀM: Giữ chân Servo ở mức LOW 1 giây để ổn định nguồn
  pinMode(L_PIN, OUTPUT); digitalWrite(L_PIN, LOW);
  pinMode(R_PIN, OUTPUT); digitalWrite(R_PIN, LOW);
  delay(1000); 

  // Cấu hình UART và PWM
  FlowSerial.begin(19200, SERIAL_8N1, 16, 17); 
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  
  pinMode(P_IN, INPUT_PULLDOWN);
  pinMode(R_IN, INPUT_PULLDOWN);

  Serial.println(">>> SYSTEM ARMED: ANTI-CONFLICT ENABLED");
}

// Hàm đọc tín hiệu RX chống nhiễu tuyệt đối
long readFailsafeRX(int pin) {
  long p = pulseIn(pin, HIGH, 25000); // Timeout 25ms
  
  // Nếu không có xung hoặc xung rác (nhỏ hơn 900 hoặc lớn hơn 2100)
  if (p < 900 || p > 2100) {
    return 1500; // Trả về vị trí an toàn nhất
  }
  return p;
}

void loop() {
  // 1. Đọc và lọc dữ liệu Optical Flow
  if (FlowSerial.available() > 0) {
    String data = FlowSerial.readStringUntil('\n');
    int comma = data.indexOf(',');
    if (comma != -1) {
      flowX = data.substring(0, comma).toFloat();
      flowY = data.substring(comma + 1).toFloat();
      lastFlowTime = millis();
    }
  }

  // Nếu quá 200ms không có dữ liệu Flow, reset giá trị về 0 để tránh trôi servo
  if (millis() - lastFlowTime > 200) {
    flowX = 0; flowY = 0;
  }

  // 2. Đọc RX với chế độ Failsafe
  long rawP = readFailsafeRX(P_IN);
  long rawR = readFailsafeRX(R_IN);

  // 3. Lọc mượt tín hiệu (Lọc thông thấp - Low Pass Filter)
  // Giúp servo không bị rung giật khi tín hiệu RX bị nhiễu nhỏ
  smoothP = (smoothP * 0.8) + (rawP * 0.2);
  smoothR = (smoothR * 0.8) + (rawR * 0.2);

  // 4. Mixer & Optical Flow Assist
  float targetP = map(smoothP, 1000, 2000, -35, 35);
  float targetR = map(smoothR, 1000, 2000, -35, 35);

  float gain = 0.35; // Độ nhạy ổn định
  float outP = targetP - (flowY * gain);
  float outR = targetR + (flowX * gain);

  // 5. Tính toán PWM với giới hạn an toàn
  float vL = constrain(1500 + outP + outR, 1150, 1850);
  float vR = constrain(1500 - outP + outR, 1150, 1850);

  // Xuất xung ra Servo
  ledcWrite(L_PIN, (vL / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (vR / 20000.0) * MAX_DUTY);

  delay(10); // Tần số vòng lặp 100Hz ổn định
}
