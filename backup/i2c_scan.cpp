#include <Arduino.h>
#include <Wire.h>

static TwoWire IMUWire(1);           // 전용 I2C 컨트롤러 #1 사용 (기본 Wire와 분리)
static const int I2C_SDA_PIN = 38;   // Nano ESP32 D11
static const int I2C_SCL_PIN = 47;   // Nano ESP32 D12

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.printf("\nI2C scan (Nano ESP32)\nUsing SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);

  IMUWire.begin(I2C_SDA_PIN, I2C_SCL_PIN);   // ★ 절대 SDA/SCL 매크로 쓰지 말고 숫자/상수만
  IMUWire.setClock(100000);

  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  Serial.printf("Idle SDA=%d, SCL=%d (expect both 1)\n", digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));

  for (uint8_t a = 1; a < 127; a++) {
    IMUWire.beginTransmission(a);
    if (IMUWire.endTransmission() == 0) {
      Serial.printf("Found 0x%02X\n", a);
    }
  }
  Serial.println("Scan done.");
}

void loop() {
  Serial.println("loop...");
}
