/*
 * MPU9250 진단 테스트 스케치
 * 
 * 배선:
 * GY-9250 VCC → 3.3V (빨간선)
 * GY-9250 GND → GND (검은선)
 * GY-9250 SCL → D12 (GPIO47)
 * GY-9250 SDA → D11 (GPIO38)
 * GY-9250 NCS → 3.3V (필수! - I2C 모드 활성화)
 * GY-9250 AD0 → GND (권장 - 0x68 주소 사용)
 */

#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 38  // D11
#define SCL_PIN 47  // D12
#define MPU9250_ADDR 0x68
#define MPU9250_ADDR_ALT 0x69

// ✅ 함수 선언 (forward declarations)
void scanI2C();
void testMPU9250();
bool initMPU9250();
bool readSensorData(int16_t* ax, int16_t* ay, int16_t* az, 
                    int16_t* gx, int16_t* gy, int16_t* gz);
uint8_t readRegister(uint8_t addr, uint8_t reg);
void writeRegister(uint8_t addr, uint8_t reg, uint8_t data);

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n=================================");
  Serial.println("MPU9250 하드웨어 진단 테스트");
  Serial.println("=================================\n");

  // I2C 초기화
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100kHz (안정성)
  
  Serial.printf("I2C 초기화: SDA=GPIO%d, SCL=GPIO%d\n\n", SDA_PIN, SCL_PIN);
  delay(100);

  // 1단계: I2C 버스 스캔
  Serial.println("=== 1단계: I2C 버스 스캔 ===");
  scanI2C();
  
  // 2단계: MPU9250 WHO_AM_I 확인
  Serial.println("\n=== 2단계: MPU9250 감지 ===");
  testMPU9250();
  
  // 3단계: 센서 데이터 읽기 테스트
  Serial.println("\n=== 3단계: 센서 데이터 읽기 ===");
  if (initMPU9250()) {
    Serial.println("초기화 성공! 센서 데이터 모니터링 시작...\n");
  } else {
    Serial.println("초기화 실패!");
  }
}

void loop() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 500) {  // 0.5초마다
    lastPrint = millis();
    
    // 센서 데이터 읽기
    int16_t ax, ay, az, gx, gy, gz;
    if (readSensorData(&ax, &ay, &az, &gx, &gy, &gz)) {
      Serial.printf("Accel: %6d %6d %6d | Gyro: %6d %6d %6d\n", 
                    ax, ay, az, gx, gy, gz);
      
      // 간단한 상태 체크
      if (abs(az) < 1000) {
        Serial.println("  ⚠️  WARNING: Z축 가속도 비정상 (센서 방향 확인)");
      }
    } else {
      Serial.println("❌ 센서 읽기 실패!");
    }
  }
}

// =================================
// 함수 구현
// =================================

void scanI2C() {
  uint8_t error, address;
  int nDevices = 0;
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("  ✓ I2C 장치 발견: 0x%02X", address);
      
      // 알려진 장치 식별
      if (address == 0x68 || address == 0x69) {
        Serial.print(" (MPU9250 예상)");
      }
      Serial.println();
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("  ❌ I2C 장치가 발견되지 않았습니다!");
    Serial.println("\n하드웨어 점검 사항:");
    Serial.println("  1. GY-9250 모듈에 전원이 공급되는지 확인 (LED 점등?)");
    Serial.println("  2. SDA/SCL 연결 확인 (교차 연결 여부)");
    Serial.println("  3. **NCS 핀을 반드시 3.3V에 연결** (I2C 모드 활성화)");
    Serial.println("  4. 점퍼선 불량 여부 확인");
    Serial.println("  5. 브레드보드 접촉 불량 확인");
  } else {
    Serial.printf("\n총 %d개의 I2C 장치 발견\n", nDevices);
  }
}

void testMPU9250() {
  uint8_t whoami_68 = readRegister(MPU9250_ADDR, 0x75);
  uint8_t whoami_69 = readRegister(MPU9250_ADDR_ALT, 0x75);
  
  Serial.printf("WHO_AM_I @ 0x68: 0x%02X ", whoami_68);
  if (whoami_68 == 0x71 || whoami_68 == 0x73) {
    Serial.println("✓ (MPU9250 정상)");
  } else if (whoami_68 == 0xFF) {
    Serial.println("❌ (응답 없음)");
  } else {
    Serial.println("❌ (잘못된 ID)");
  }
  
  Serial.printf("WHO_AM_I @ 0x69: 0x%02X ", whoami_69);
  if (whoami_69 == 0x71 || whoami_69 == 0x73) {
    Serial.println("✓ (MPU9250 정상)");
  } else if (whoami_69 == 0xFF) {
    Serial.println("❌ (응답 없음)");
  } else {
    Serial.println("❌ (잘못된 ID)");
  }
  
  if (whoami_68 != 0x71 && whoami_68 != 0x73 && 
      whoami_69 != 0x71 && whoami_69 != 0x73) {
    Serial.println("\n❌ MPU9250을 찾을 수 없습니다!");
    Serial.println("\n문제 해결:");
    Serial.println("  1. **NCS 핀이 3.3V에 연결되었는지 확인**");
    Serial.println("     → NCS가 플로팅 상태면 SPI 모드로 동작하여 I2C 불가");
    Serial.println("  2. AD0 핀 확인:");
    Serial.println("     → GND 연결 시 주소 0x68");
    Serial.println("     → VCC 연결 시 주소 0x69");
    Serial.println("  3. 모듈 전원 확인 (3.3V, 최소 50mA)");
  }
}

bool initMPU9250() {
  // 소프트웨어 리셋
  writeRegister(MPU9250_ADDR, 0x6B, 0x80);
  delay(100);
  
  // 클럭 소스 설정
  writeRegister(MPU9250_ADDR, 0x6B, 0x01);
  delay(10);
  
  // 샘플레이트
  writeRegister(MPU9250_ADDR, 0x19, 0x00);
  
  // 자이로 설정 (±2000dps)
  writeRegister(MPU9250_ADDR, 0x1B, 0x18);
  
  // 가속도계 설정 (±8g)
  writeRegister(MPU9250_ADDR, 0x1C, 0x10);
  
  // 확인
  uint8_t whoami = readRegister(MPU9250_ADDR, 0x75);
  return (whoami == 0x71 || whoami == 0x73);
}

bool readSensorData(int16_t* ax, int16_t* ay, int16_t* az, 
                    int16_t* gx, int16_t* gy, int16_t* gz) {
  uint8_t data[14];
  
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H 레지스터
  if (Wire.endTransmission(false) != 0) return false;
  
  if (Wire.requestFrom(MPU9250_ADDR, 14) != 14) return false;
  
  for (int i = 0; i < 14; i++) {
    data[i] = Wire.read();
  }
  
  *ax = (data[0] << 8) | data[1];
  *ay = (data[2] << 8) | data[3];
  *az = (data[4] << 8) | data[5];
  // data[6], data[7] = 온도
  *gx = (data[8] << 8) | data[9];
  *gy = (data[10] << 8) | data[11];
  *gz = (data[12] << 8) | data[13];
  
  return true;
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return 0xFF;
  
  return Wire.read();
}

void writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}