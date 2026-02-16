#include <Wire.h>

#define MPU_ADDR 0x68

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- MPU6500 DIAGNOSTIC START ---");

  Wire.begin(21, 22); // SDA=21, SCL=22
  Wire.setClock(400000);

  // 1. Kiểm tra xem có thiết bị nào trên đường dây I2C không
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("[OK] Da tim thay MPU6500 tai dia chi 0x68");
  } else {
    Serial.println("[ERROR] Khong tim thay MPU6500. Kiem tra day SDA/SCL va Nguon!");
    while (1); // Dung lai neu khong co cam bien
  }

  // 2. Khoi dong MPU6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0x00); // Danh thuc MPU
  if (Wire.endTransmission() != 0) {
    Serial.println("[ERROR] Khong the khoi tao MPU6500!");
    while (1);
  }
  
  Serial.println("[OK] MPU6500 da san sang. Dang doc du lieu...");
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Bat dau tu thanh ghi cam bien gia toc
  if (Wire.endTransmission(false) != 0) {
    Serial.println("\n[!] Mat ket noi voi MPU6500!");
    delay(500);
    return;
  }

  Wire.requestFrom(MPU_ADDR, 6);
  if (Wire.available() == 6) {
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();

    Serial.print("AccX: "); Serial.print(ax);
    Serial.print(" | AccY: "); Serial.print(ay);
    Serial.print(" | AccZ: "); Serial.println(az);
  }

  delay(100); // Doc moi 0.1 giay cho de nhìn
}
