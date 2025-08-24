#include <Arduino.h>

const int ledPin = 26;   // GPIO 핀
const int pwmChannel = 0; // 사용할 PWM 채널 (0~15)
const int pwmFreq = 5000; // PWM 주파수 (Hz)
const int pwmResolution = 8; // 해상도 (8비트: 0~255)

int brightness = 0;
int fadeAmount = 5;

void setup() {
  // PWM 채널 설정
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);

  // 채널을 특정 핀에 연결
  ledcAttachPin(ledPin, pwmChannel);
}

void loop() {
  // 현재 밝기값을 PWM 출력
  ledcWrite(pwmChannel, brightness);

  // 밝기 변경
  brightness += fadeAmount;

  // 범위 체크 (0~255 사이 유지)
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount; // 반전
  }

  delay(50);
}
