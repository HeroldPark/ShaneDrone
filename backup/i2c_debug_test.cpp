/*
 * I2C 상세 디버깅 테스트
 * MPU9250이 발견되지 않을 때 사용
 */

#include <Arduino.h>
#include <Wire.h>

// 함수 선언
void testI2CAddress(uint8_t addr);
uint8_t readRegister(uint8_t addr, uint8_t reg);

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║   I2C 상세 디버깅 테스트                      ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
  
  // I2C 초기화
  Serial.println("1️⃣  Wire.begin() 초기화");
  Serial.println("─────────────────────────────────────");
  if (!Wire.begin()) {
    Serial.println("  ✗ Wire.begin() 실패!");
    while(1) delay(1000);
  }
  Serial.println("  ✓ Wire.begin() 성공");
  
  Wire.setClock(100000);
  Serial.println("  ✓ 클럭: 100kHz");
  delay(100);
  
  // 전체 I2C 주소 스캔 (상세 버전)
  Serial.println("\n2️⃣  I2C 버스 상세 스캔");
  Serial.println("─────────────────────────────────────");
  Serial.println("┌────────┬────────┬──────────────────┐");
  Serial.println("│ 주소   │ 결과   │ 비고             │");
  Serial.println("├────────┼────────┼──────────────────┤");
  
  int deviceCount = 0;
  
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    // 중요한 주소만 상세히 표시
    if (error == 0) {
      Serial.printf("│ 0x%02X   │ 발견   │", addr);
      if (addr == 0x68 || addr == 0x69) {
        Serial.print(" MPU9250?");
      } else if (addr == 0x0C) {
        Serial.print(" AK8963?");
      }
      Serial.println("         │");
      deviceCount++;
    } else if (addr == 0x68 || addr == 0x69) {
      // MPU9250 주소는 에러여도 표시
      Serial.printf("│ 0x%02X   │ 없음   │ Error: %d", addr, error);
      Serial.println("        │");
    }
    
    delay(5);
  }
  
  Serial.println("└────────┴────────┴──────────────────┘");
  Serial.printf("\n총 %d개의 I2C 장치 발견\n\n", deviceCount);
  
  // MPU9250 특정 주소 상세 테스트
  Serial.println("3️⃣  MPU9250 주소 상세 테스트");
  Serial.println("─────────────────────────────────────");
  
  // 0x68 테스트
  Serial.println("\n[0x68 테스트 - 10회 반복]");
  testI2CAddress(0x68);
  
  // 0x69 테스트  
  Serial.println("\n[0x69 테스트 - 10회 반복]");
  testI2CAddress(0x69);
  
  // WHO_AM_I 읽기 시도
  Serial.println("\n4️⃣  WHO_AM_I 레지스터 읽기");
  Serial.println("─────────────────────────────────────");
  
  Serial.println("\n[0x68, 레지스터 0x75]");
  uint8_t whoami = readRegister(0x68, 0x75);
  Serial.printf("  응답: 0x%02X\n", whoami);
  
  if (whoami == 0x71 || whoami == 0x73) {
    Serial.println("  ✓✓✓ MPU9250 확인됨! ✓✓✓");
    Serial.println("  → 하드웨어는 정상입니다!");
  } else if (whoami == 0xFF) {
    Serial.println("  ✗ 응답 없음 (0xFF = 풀업 저항 값)");
    Serial.println("  → I2C 버스에 장치 없음");
  } else if (whoami == 0x00) {
    Serial.println("  ✗ 응답 없음 (0x00)");
    Serial.println("  → I2C 버스 문제");
  } else {
    Serial.printf("  ⚠  예상치 못한 ID (예상: 0x71 또는 0x73)\n");
    Serial.println("  → 다른 장치일 수 있음");
  }
  
  Serial.println("\n[0x69, 레지스터 0x75]");
  whoami = readRegister(0x69, 0x75);
  Serial.printf("  응답: 0x%02X\n", whoami);
  
  if (whoami == 0x71 || whoami == 0x73) {
    Serial.println("  ✓✓✓ MPU9250 확인됨! ✓✓✓");
    Serial.println("  → AD0가 VCC에 연결됨 (주소 0x69)");
  } else if (whoami == 0xFF) {
    Serial.println("  ✗ 응답 없음 (0xFF)");
  } else if (whoami == 0x00) {
    Serial.println("  ✗ 응답 없음 (0x00)");
  } else {
    Serial.printf("  ⚠  예상치 못한 ID\n");
  }
  
  // 결론
  Serial.println("\n╔════════════════════════════════════════════════╗");
  if (deviceCount == 0) {
    Serial.println("║   결과: I2C 장치가 전혀 발견되지 않음        ║");
    Serial.println("╚════════════════════════════════════════════════╝");
    Serial.println("\n⚠️  가능한 원인 (가능성 순):");
    Serial.println("\n1. NCS 핀 문제 (60%)");
    Serial.println("   - 초록 와이어가 MPU9250 NCS 핀에서 빠졌거나");
    Serial.println("   - 브레드보드 접촉 불량");
    Serial.println("   조치: 초록 와이어를 다시 확실히 꽂기");
    Serial.println("\n2. 브레드보드 접촉 불량 (30%)");
    Serial.println("   - 와이어가 제대로 꽂히지 않음");
    Serial.println("   조치: 모든 와이어를 뺐다가 다시 꽂기");
    Serial.println("\n3. 전원 문제 (5%)");
    Serial.println("   - VCC 전압 부족");
    Serial.println("   조치: 멀티미터로 VCC 핀 전압 측정 (3.3V?)");
    Serial.println("\n4. MPU9250 모듈 불량 (5%)");
    Serial.println("   - 모듈 자체가 고장");
    Serial.println("   조치: 다른 MPU9250 모듈로 교체");
    
  } else if (whoami == 0x71 || whoami == 0x73) {
    Serial.println("║   결과: ✓✓✓ MPU9250 발견됨! ✓✓✓            ║");
    Serial.println("╚════════════════════════════════════════════════╝");
    Serial.println("\n하드웨어 연결이 정상입니다!");
    Serial.println("원래 드론 코드를 업로드하면 작동할 것입니다.");
    
  } else {
    Serial.println("║   결과: I2C 장치 발견, 하지만 MPU9250 아님  ║");
    Serial.println("╚════════════════════════════════════════════════╝");
    Serial.println("\n다른 I2C 장치가 연결되어 있습니다.");
    Serial.println("MPU9250 연결을 다시 확인하세요.");
  }
  
  Serial.println("\n═══════════════════════════════════════════════");
  Serial.println("테스트 완료! 위 결과를 공유해주세요.");
  Serial.println("═══════════════════════════════════════════════\n");
}

void loop() {
  delay(1000);
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 헬퍼 함수들
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void testI2CAddress(uint8_t addr) {
  Serial.printf("  주소: 0x%02X\n", addr);
  
  // 10회 반복 테스트
  int successCount = 0;
  Serial.print("  테스트: ");
  
  for (int i = 0; i < 10; i++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("✓");
      successCount++;
    } else {
      Serial.print("✗");
    }
    
    delay(10);
  }
  
  Serial.printf(" (%d/10)\n", successCount);
  
  if (successCount == 0) {
    Serial.println("  결과: ✗ 전혀 응답 없음");
  } else if (successCount < 5) {
    Serial.println("  결과: ⚠  매우 불안정 (접촉 불량?)");
  } else if (successCount < 10) {
    Serial.println("  결과: ⚠  간헐적 연결 (접촉 불량 의심)");
  } else {
    Serial.println("  결과: ✓ 안정적 연결");
  }
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  
  if (Wire.endTransmission(false) != 0) {
    return 0xFF;  // 에러
  }
  
  Wire.requestFrom(addr, (uint8_t)1);
  
  if (Wire.available()) {
    return Wire.read();
  }
  
  return 0xFF;  // 에러
}