/*
 * I2C 스캔 테스트 - MPU9250 감지
 * - 도움말 문구를 A4/A5 기준으로 정리
 * - 스캔 전 라인 상태 출력(붙잡힘 진단)
 * - 필요 시 I2C 버스 복구(9클럭 토글 + STOP)
 */

#include <Arduino.h>
#include <Wire.h>

// ✅ ESP32-WROOM-32E용 LED 핀 정의
#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // ESP32-WROOM-32E 내장 LED는 GPIO2
#endif

// 보드 매크로를 사용해 핀 혼동 방지
#define SDA_PIN 21  // ESP32 기본 SDA
#define SCL_PIN 22  // ESP32 기본 SCL

// ---- I2C 라인 상태/복구 유틸 ----
static inline int readLineHi(int pin) {
  pinMode(pin, INPUT_PULLUP);     // 내부 풀업으로 '하드 LOW' 여부 판별
  delayMicroseconds(5);
  return digitalRead(pin);        // 1=HIGH(정상 혹은 부하없음), 0=LOW(붙잡힘/단락)
}

void printLineState(int scl, int sda, const char* tag) {
  int sdaState = readLineHi(sda);
  int sclState = readLineHi(scl);
  Serial.printf("라인 상태(%s): SDA=%d, SCL=%d (1=HIGH)\n", tag, sdaState, sclState);
}

// SCL을 9펄스 토글한 후 STOP 조건 생성(버스 붙잡힘 해제 시도)
void i2cBusRecover(int scl, int sda) {
  Serial.println("⚠️  버스 복구 시도(9클럭 + STOP)...");
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT);
  digitalWrite(scl, HIGH);
  delayMicroseconds(5);

  for (int i = 0; i < 9; i++) {
    digitalWrite(scl, LOW);  delayMicroseconds(8);
    digitalWrite(scl, HIGH); delayMicroseconds(8);
  }
  // STOP: SCL HIGH 상태에서 SDA 릴리스(풀업 HIGH)
  digitalWrite(scl, HIGH);
  pinMode(sda, INPUT_PULLUP);
  delayMicroseconds(10);
  printLineState(scl, sda, "복구후");
}

// ---- I2C 스캐너 ----
void performI2CScan() {
  Serial.println("\n=================================");
  Serial.println("I2C 버스 스캔 시작");
  Serial.println("=================================\n");

  // 1) 스캔 전 라인 진단 및 필요시 복구
  printLineState(SCL_PIN, SDA_PIN, "사전");
  if (readLineHi(SCL_PIN) == LOW || readLineHi(SDA_PIN) == LOW) {
    i2cBusRecover(SCL_PIN, SDA_PIN);
  }

  // 2) I2C 시작
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeout(50);
  Serial.printf("I2C 설정: SDA=%d, SCL=%d, 100kHz\n", SDA_PIN, SCL_PIN);

  // 3) 주소 스캔
  int found = 0;
  bool mpu9250Found = false;

  for (uint8_t addr = 1; addr < 127; addr++) {
    if ((addr & 0x0F) == 0x00) {
      Serial.printf("스캔 중: 0x%02X - 0x%02X\n", addr, addr + 15);
    }

    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("  ✓ 장치 발견: 0x%02X", addr);

      // MPU-9250 후보 주소
      if (addr == 0x68 || addr == 0x69) {
        Serial.print(" ← MPU9250?");
        // WHO_AM_I(0x75) 읽기
        Wire.beginTransmission(addr);
        Wire.write(0x75);
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom(addr, (uint8_t)1) == 1) {
          uint8_t whoami = Wire.read();
          Serial.printf(" (WHO_AM_I=0x%02X)", whoami);
          // 일반적으로 0x71(9250), 0x70(6500 파생), 일부 모듈은 0x73 보고 사례도 존재
          if (whoami == 0x71 || whoami == 0x70 || whoami == 0x73) {
            Serial.print(" ✓ 정상범위");
            mpu9250Found = true;
          } else if (whoami == 0xFF) {
            Serial.print(" ❌ 응답 없음");
          } else {
            Serial.print(" ⚠️  예상치 못한 ID");
          }
        } else {
          Serial.print(" ⚠️  WHO_AM_I 읽기 실패");
        }
      }

      Serial.println();
      found++;
    }

    delay(2);
  }

  // 4) 결과 요약
  Serial.println("\n=================================");
  Serial.printf("스캔 완료: %d개 장치 발견\n", found);
  Serial.println("=================================\n");

  if (found == 0) {
    Serial.println("❌ I2C 장치가 발견되지 않았습니다!\n");
    Serial.println("점검 체크리스트:");
    Serial.println("  1) 배선: VCC→3.3V, GND→GND");
    Serial.println("  2) 핀  : SCL→GPIO22, SDA→GPIO21");
    Serial.println("  3) NCS : 3.3V에 연결(필수, I2C 모드 고정)");
    Serial.println("  4) AD0 : GND(주소 0x68) 또는 3.3V(0x69)");
    Serial.println("  5) 라인: 스캔 직전 '라인 상태'가 둘 다 1인지 확인");
  } else if (!mpu9250Found) {
    Serial.printf("⚠️  %d개 장치가 보이지만 MPU9250(0x68/0x69)은 없음\n\n", found);
    Serial.println("추가 점검:");
    Serial.println("  - AD0=GND → 0x68, AD0=3.3V → 0x69");
    Serial.println("  - 모듈 실크의 VCC/VIN 의미 확인(3.3V 전용인지, 5V LDO 입력인지)");
  } else {
    Serial.println("✓ MPU9250 감지 성공!\n");
  }
}

void setup() {
  // LED 준비
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); delay(300);
  digitalWrite(LED_BUILTIN, LOW);  delay(300);   // 1회: setup 진입

  // 일반 UART Serial
  Serial.begin(115200);
  
  // ✅ ESP32-WROOM-32E는 Serial.isConnected()가 없으므로 단순 대기
  delay(2000);  // 시리얼 모니터 연결 대기
  
  for (int i = 0; i < 2; i++) {                  // 2회: Serial.begin 완료
    digitalWrite(LED_BUILTIN, HIGH); delay(150);
    digitalWrite(LED_BUILTIN, LOW);  delay(150);
  }

  // 디버그 배너
  Serial.println("\n\n=========================");
  Serial.println("ESP32-WROOM-32E I2C 스캔");
  Serial.println("=========================");
  Serial.printf("보드: ESP32-DevKit (WROOM-32E)\n");
  Serial.printf("시간: %lu ms\n", millis());
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());

  Serial.println("\nLED 신호 해석:");
  Serial.println("  1회 깜빡임 = setup() 진입");
  Serial.println("  2회 깜빡임 = Serial.begin() 완료");

  // 초기 스캔
  performI2CScan();

  Serial.println("\n💡 'r' 입력 후 Enter를 누르면 재스캔");
  Serial.print("> ");
}

void loop() {
  static unsigned long lastStatus = 0;
  static int statusCount = 0;

  // 5초마다 상태 출력
  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    statusCount++;
    Serial.println("\n===== 시스템 상태 =====");
    Serial.printf("가동 시간: %lu 초\n", millis() / 1000);
    Serial.printf("Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("상태 출력 횟수: %d\n", statusCount);
    Serial.println("명령: r=스캔, h=도움말");
    Serial.println("======================\n> ");
  }

  // 시리얼 명령
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();  // 버퍼 비우기
    Serial.println(cmd);                       // 에코

    if (cmd == 'r' || cmd == 'R') {
      Serial.println("\n>>> 재스캔 시작\n");
      performI2CScan();
      Serial.print("\n> ");
    } else if (cmd == 'h' || cmd == 'H') {
      Serial.println("\n=== 도움말 ===");
      Serial.println("r = I2C 재스캔");
      Serial.println("h = 도움말");
      Serial.println("\nMPU9250 배선 (ESP32-WROOM-32E):");
      Serial.println("  VCC → 3.3V");
      Serial.println("  GND → GND");
      Serial.println("  SCL → GPIO22");
      Serial.println("  SDA → GPIO21");
      Serial.println("  NCS → 3.3V (필수!)");
      Serial.println("  AD0 → GND(주소 0x68) 또는 3.3V(0x69)");
      Serial.print("\n> ");
    } else if (cmd == '\n' || cmd == '\r') {
      Serial.print("> ");
    } else {
      Serial.printf("알 수 없는 명령: '%c' (h=도움말)\n> ", cmd);
    }
  }
}