#include <SPI.h>

// Định nghĩa chân SPI cho ESP32
#define CS_PIN 5

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- MPU6500 SPI DIAGNOSTIC ---");

  // Khởi tạo SPI
  SPI.begin(18, 19, 23, CS_PIN); // SCK, MISO, MOSI, SS
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Thử đọc thanh ghi WHO_AM_I (Địa chỉ 0x75)
  // MPU6500 sẽ trả về giá trị 0x70 nếu còn sống
  byte whoAmI = readRegister(0x75);
  
  Serial.print("WHO_AM_I Register: 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI == 0x70 || whoAmI == 0x71) {
    Serial.println("[SUCCESS] MPU6500 phan hoi qua SPI!");
  } else {
    Serial.println("[FAILED] Khong nhan dang duoc MPU6500 qua SPI.");
    Serial.println("Kiem tra lai day AD0 -> 19 va SCL/SDA -> 18/23");
    // Khong dung lai, cu thu doc tiep xem co so lieu không
  }

  // Danh thuc MPU6500 (Power Management 1)
  writeRegister(0x6B, 0x00);
}

void loop() {
  // Doc gia toc truc Z (vi du)
  int16_t az = (readRegister(0x3F) << 8) | readRegister(0x40);
  
  Serial.print("Accel Z (SPI): ");
  Serial.println(az);
  
  delay(200);
}

// Ham doc thanh ghi qua SPI
byte readRegister(byte reg) {
  byte data;
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80); // Set bit cao nhat de doc
  data = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return data;
}

// Ham ghi thanh ghi qua SPI
void writeRegister(byte reg, byte val) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg & 0x7F); // Bit cao nhat = 0 de ghi
  SPI.transfer(val);
  digitalWrite(CS_PIN, HIGH);
}
