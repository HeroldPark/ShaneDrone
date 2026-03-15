/*
 * pin_test - Arduino Nano ESP32 I2C 핀 확인
 * 
 * 사용법:
 * 1. 이 파일을 src/main.cpp로 복사
 * 2. pio run -t upload -t monitor 실행
 * 3. 시리얼 모니터에서 결과 확인
 * 4. 성공한 핀 조합을 config.h에 적용
 */

#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║  Arduino Nano ESP32 I2C 핀 번호 확인          ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
  
  // 1. 보드 정의 상수 확인
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("1️⃣  보드 기본 I2C 핀 상수");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  #ifdef SDA
    Serial.printf("SDA           = %d\n", SDA);
  #else
    Serial.println("SDA           = 정의 안 됨");
  #endif
  
  #ifdef SCL
    Serial.printf("SCL           = %d\n", SCL);
  #else
    Serial.println("SCL           = 정의 안 됨");
  #endif
  
  #ifdef PIN_WIRE_SDA
    Serial.printf("PIN_WIRE_SDA  = %d\n", PIN_WIRE_SDA);
  #else
    Serial.println("PIN_WIRE_SDA  = 정의 안 됨");
  #endif
  
  #ifdef PIN_WIRE_SCL
    Serial.printf("PIN_WIRE_SCL  = %d\n", PIN_WIRE_SCL);
  #else
    Serial.println("PIN_WIRE_SCL  = 정의 안 됨");
  #endif
  
  // 2. Arduino 디지털 핀 (D11, D12)
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("2️⃣  Arduino 디지털 핀 (I2C 관련)");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  #ifdef D11
    Serial.printf("D11 (SDA?)    = %d\n", D11);
  #else
    Serial.println("D11           = 정의 안 됨");
  #endif
  
  #ifdef D12
    Serial.printf("D12 (SCL?)    = %d\n", D12);
  #else
    Serial.println("D12           = 정의 안 됨");
  #endif
  
  // 3. I2C 초기화 테스트
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("3️⃣  I2C 초기화 테스트");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
  
  int testCount = 1;
  bool foundWorking = false;
  int workingSDA = -1;
  int workingSCL = -1;
  
  // 테스트 1: 기본 Wire.begin()
  Serial.printf("[테스트 %d] Wire.begin() - 기본 핀\n", testCount++);
  if (Wire.begin()) {
    Serial.println("  ✅ 초기화 성공!");
    
    // I2C 스캔
    Serial.println("  🔍 I2C 장치 스캔 중...");
    int nDevices = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("    ✓ 0x%02X 발견", addr);
        if (addr == 0x68 || addr == 0x69) Serial.print(" (MPU9250!)");
        Serial.println();
        nDevices++;
      }
    }
    
    if (nDevices > 0) {
      Serial.println("\n  ⭐⭐⭐ 기본 핀으로 작동! ⭐⭐⭐");
      Serial.println("  config.h 설정:");
      #ifdef SDA
        Serial.printf("    #define MPU_SDA_PIN SDA  // %d\n", SDA);
        Serial.printf("    #define MPU_SCL_PIN SCL  // %d\n", SCL);
        workingSDA = SDA;
        workingSCL = SCL;
      #else
        Serial.println("    Wire.begin() 사용 (핀 지정 불필요)");
      #endif
      foundWorking = true;
    } else {
      Serial.printf("  ⚠️  초기화는 성공했으나 I2C 장치 없음 (%d개)\n", nDevices);
    }
    Wire.end();
  } else {
    Serial.println("  ❌ 초기화 실패");
  }
  Serial.println();
  delay(100);
  
  // 테스트 2: SDA, SCL 상수
  #if defined(SDA) && defined(SCL)
  if (!foundWorking) {
    Serial.printf("[테스트 %d] Wire.begin(SDA, SCL)\n", testCount++);
    Serial.printf("  호출: Wire.begin(%d, %d)\n", SDA, SCL);
    if (Wire.begin(SDA, SCL)) {
      Serial.println("  ✅ 초기화 성공!");
      
      int nDevices = 0;
      for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          Serial.printf("    ✓ 0x%02X 발견\n", addr);
          nDevices++;
        }
      }
      
      if (nDevices > 0) {
        Serial.println("  ⭐⭐⭐ 작동! ⭐⭐⭐");
        Serial.printf("  config.h: MPU_SDA_PIN=%d, MPU_SCL_PIN=%d\n", SDA, SCL);
        workingSDA = SDA;
        workingSCL = SCL;
        foundWorking = true;
      }
      Wire.end();
    } else {
      Serial.println("  ❌ 초기화 실패");
    }
    Serial.println();
  }
  #endif
  delay(100);
  
  // 테스트 3: D11, D12
  #if defined(D11) && defined(D12)
  if (!foundWorking) {
    Serial.printf("[테스트 %d] Wire.begin(D11, D12)\n", testCount++);
    Serial.printf("  호출: Wire.begin(%d, %d)\n", D11, D12);
    if (Wire.begin(D11, D12)) {
      Serial.println("  ✅ 초기화 성공!");
      
      int nDevices = 0;
      for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          Serial.printf("    ✓ 0x%02X 발견\n", addr);
          nDevices++;
        }
      }
      
      if (nDevices > 0) {
        Serial.println("  ⭐⭐⭐ 작동! ⭐⭐⭐");
        Serial.printf("  config.h: MPU_SDA_PIN=D11 (%d), MPU_SCL_PIN=D12 (%d)\n", D11, D12);
        workingSDA = D11;
        workingSCL = D12;
        foundWorking = true;
      }
      Wire.end();
    } else {
      Serial.println("  ❌ 초기화 실패");
    }
    Serial.println();
  }
  #endif
  delay(100);
  
  // 테스트 4-7: 일반적인 ESP32 I2C 핀
  if (!foundWorking) {
    struct {
      int sda;
      int scl;
      const char* name;
    } pinCombos[] = {
      {21, 22, "ESP32 기본"},
      {11, 12, "대체 1"},
      {18, 19, "대체 2"},
      {33, 32, "대체 3"}
    };
    
    for (int i = 0; i < 4; i++) {
      Serial.printf("[테스트 %d] %s: Wire.begin(%d, %d)\n", 
                    testCount++, pinCombos[i].name, pinCombos[i].sda, pinCombos[i].scl);
      
      if (Wire.begin(pinCombos[i].sda, pinCombos[i].scl)) {
        Serial.println("  ✅ 초기화 성공!");
        
        int nDevices = 0;
        for (uint8_t addr = 1; addr < 127; addr++) {
          Wire.beginTransmission(addr);
          if (Wire.endTransmission() == 0) {
            Serial.printf("    ✓ 0x%02X 발견\n", addr);
            nDevices++;
          }
        }
        
        if (nDevices > 0) {
          Serial.println("  ⭐⭐⭐ 작동! ⭐⭐⭐");
          Serial.printf("  config.h: MPU_SDA_PIN=%d, MPU_SCL_PIN=%d\n", 
                       pinCombos[i].sda, pinCombos[i].scl);
          workingSDA = pinCombos[i].sda;
          workingSCL = pinCombos[i].scl;
          foundWorking = true;
          Wire.end();
          break;
        }
        Wire.end();
      } else {
        Serial.println("  ❌ 초기화 실패");
      }
      Serial.println();
      delay(100);
    }
  }
  
  // 최종 결과
  Serial.println("\n╔════════════════════════════════════════════════╗");
  if (foundWorking) {
    Serial.println("║  ✅ 작동하는 I2C 핀 조합 발견!               ║");
    Serial.println("╚════════════════════════════════════════════════╝");
    Serial.println("\n📋 config.h에 다음과 같이 설정하세요:\n");
    Serial.println("┌────────────────────────────────────────────┐");
    Serial.printf("│ #define MPU_SDA_PIN %-2d                     │\n", workingSDA);
    Serial.printf("│ #define MPU_SCL_PIN %-2d                     │\n", workingSCL);
    Serial.println("└────────────────────────────────────────────┘");
  } else {
    Serial.println("║  ❌ 작동하는 I2C 핀 조합을 찾지 못함        ║");
    Serial.println("╚════════════════════════════════════════════════╝");
    Serial.println("\n가능한 원인:");
    Serial.println("  1. MPU9250 모듈이 연결되지 않음");
    Serial.println("  2. NCS 핀이 3.3V에 연결되지 않음");
    Serial.println("  3. 배선 오류 (SDA/SCL 교차, 접촉 불량)");
    Serial.println("  4. 전원 문제 (VCC가 3.3V가 아님)");
  }
  
  Serial.println("\n테스트 완료!");
}

void loop() {
  delay(1000);
}