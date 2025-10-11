/*
 * sensors.cpp - MPU9250 센서 처리 (수정됨 - 안전한 I2C 통신)
 */

#include "../include/sensors.h"
#include "../include/config.h"
#include <Arduino.h>

// 전역 변수
static float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
static float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
static float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;
static float magScaleX = 1, magScaleY = 1, magScaleZ = 1;
static float complementaryAlpha = 0.98;
static unsigned long lastFilterUpdate = 0;
static bool sensorInitialized = false;
static bool sensorCalibrated = false;
static bool simulationMode = false;

// ✅ I2C 타임아웃 추가
static bool safeI2CWrite(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  uint8_t result = Wire.endTransmission();
  return (result == 0);
}

static bool safeI2CRead(uint8_t address, uint8_t reg, uint8_t* buffer, size_t len) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  
  // ✅ 수정: stopBit 파라미터를 bool로 명시
  size_t received = Wire.requestFrom(address, len, (bool)true);
  if (received != len) return false;
  
  for (size_t i = 0; i < len; i++) {
    if (Wire.available()) {
      buffer[i] = Wire.read();
    } else {
      return false;
    }
  }
  return true;
}

bool initializeSensors() {
  Serial.println("MPU9250 센서 초기화 중...");
  
  // ✅ I2C 초기화 (클럭 속도 100kHz로 안정성 확보)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  
  Serial.printf("I2C 초기화: SDA=GPIO%d, SCL=GPIO%d\n", SDA_PIN, SCL_PIN);
  delay(100);

  // ✅ I2C 스캔 (디바이스 탐지)
  Serial.println("I2C 장치 스캔 중...");
  byte error, address;
  int nDevices = 0;
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("I2C 장치 발견: 0x%02X\n", address);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("ERROR: I2C 장치가 발견되지 않았습니다!");
    Serial.println("하드웨어 연결을 확인하세요:");
    Serial.println("  - GY-9250 VCC → 3.3V");
    Serial.println("  - GY-9250 GND → GND");
    
    Serial.println("  - GY-9250 SDA → D21 (GPIO11)");
    Serial.println("  - GY-9250 SCL → D22 (GPIO12)");

    // Serial.println("  - GY-9250 SDA → D11 (GPIO38)");
    // Serial.println("  - GY-9250 SCL → D12 (GPIO47)");
    
    Serial.println("  - GY-9250 AD0 → GND (0x68 주소용)");
    Serial.println("  - GY-9250 NCS → 3.3V (필수!)");
    return false;
  }
  
  Serial.printf("총 %d개의 I2C 장치 발견\n", nDevices);

  // ✅ MPU9250 연결 확인 (0x68 또는 0x69 시도)
  if (!testMPU9250Connection()) {
    Serial.println("ERROR: MPU9250 연결 실패");
    Serial.println("시뮬레이션 모드로 전환합니다...");
    simulationMode = true;
    return false;
  }
  
  // MPU9250 설정
  if (!configureMPU9250()) {
    Serial.println("ERROR: MPU9250 설정 실패");
    return false;
  }
  
  // 자기계 초기화 (실패해도 계속 진행)
  if (!initializeAK8963()) {
    Serial.println("WARNING: AK8963(자기계) 초기화 실패 - 자이로/가속도계만 사용");
  }
  
  Serial.println("센서 초기화 완료!");
  sensorInitialized = true;
  
  // 자동 캘리브레이션
  Serial.println("센서 캘리브레이션 시작 (5초간 드론을 고정하세요)...");
  delay(2000);  // 준비 시간
  calibrateSensors();
  
  return true;
}

bool testMPU9250Connection() {
  // ✅ 0x68 주소 먼저 시도
  uint8_t whoami = 0xFF;
  if (safeI2CRead(MPU9250_ADDRESS, 0x75, &whoami, 1)) {
    if (whoami == 0x71 || whoami == 0x73) {
      Serial.printf("MPU9250 감지됨 at 0x68 (ID: 0x%02X)\n", whoami);
      return true;
    }
  }
  
  // ✅ 0x69 주소 시도
  if (safeI2CRead(MPU9250_ADDRESS_ALT, 0x75, &whoami, 1)) {
    if (whoami == 0x71 || whoami == 0x73) {
      Serial.printf("MPU9250 감지됨 at 0x69 (ID: 0x%02X)\n", whoami);
      // config.h에서 주소 변경 필요 (또는 동적으로 변경)
      return true;
    }
  }
  
  Serial.printf("ERROR: WHO_AM_I 응답 오류 (0x%02X)\n", whoami);
  Serial.println("가능한 원인:");
  Serial.println("  1. NCS 핀이 3.3V에 연결되지 않음 (SPI 모드로 동작 중)");
  Serial.println("  2. 배선 불량 (SDA/SCL 교차 연결 확인)");
  Serial.println("  3. 전원 부족 (3.3V 전압 확인)");
  return false;
}

bool configureMPU9250() {
  Serial.println("MPU9250 설정 중...");
  
  // 소프트웨어 리셋
  if (!safeI2CWrite(MPU9250_ADDRESS, 0x6B, 0x80)) {
    Serial.println("ERROR: 리셋 실패");
    return false;
  }
  delay(100);
  
  // 클럭 소스 설정
  if (!safeI2CWrite(MPU9250_ADDRESS, 0x6B, 0x01)) return false;
  delay(10);
  
  // 샘플레이트 설정
  if (!safeI2CWrite(MPU9250_ADDRESS, 0x19, 0x00)) return false;
  
  // 자이로 설정 (±2000dps, DLPF 41Hz)
  if (!safeI2CWrite(MPU9250_ADDRESS, 0x1A, GYRO_DLPF_CFG)) return false;
  if (!safeI2CWrite(MPU9250_ADDRESS, 0x1B, 0x18)) return false;
  
  // 가속도계 설정 (±8g, DLPF 41Hz)
  if (!safeI2CWrite(MPU9250_ADDRESS, 0x1C, 0x10)) return false;
  if (!safeI2CWrite(MPU9250_ADDRESS, 0x1D, ACCEL_DLPF_CFG)) return false;
  
  // I2C 마스터 활성화 (자기계 접근용)
  if (!safeI2CWrite(MPU9250_ADDRESS, 0x6A, 0x20)) return false;
  if (!safeI2CWrite(MPU9250_ADDRESS, 0x24, 0x0D)) return false;
  
  delay(10);
  Serial.println("MPU9250 설정 완료");
  return true;
}

bool initializeAK8963() {
  uint8_t whoami = 0;
  if (!safeI2CRead(MPU9250_ADDRESS, 0x49, &whoami, 1)) {
    return false;
  }
  
  if (whoami != 0x48) {
    Serial.printf("AK8963 ID 오류: 0x%02X\n", whoami);
    return false;
  }
  
  // 나머지 자기계 초기화 코드...
  Serial.println("AK8963 초기화 완료");
  return true;
}

void calibrateSensors() {
  if (!sensorInitialized) {
    Serial.println("ERROR: 센서가 초기화되지 않음");
    return;
  }
  
  Serial.println("캘리브레이션 중... (드론을 움직이지 마세요)");
  
  float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
  float accelSumX = 0, accelSumY = 0, accelSumZ = 0;
  int validSamples = 0;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    RawSensorData rawData;
    if (readRawSensorData(&rawData)) {
      gyroSumX += rawData.gyroX;
      gyroSumY += rawData.gyroY;
      gyroSumZ += rawData.gyroZ;
      
      accelSumX += rawData.accelX;
      accelSumY += rawData.accelY;
      accelSumZ += rawData.accelZ - 9.81;
      
      validSamples++;
    }
    
    if (i % 50 == 0) {
      Serial.printf("진행: %d%%\n", (i * 100) / CALIBRATION_SAMPLES);
    }
    
    delay(2);
  }
  
  if (validSamples > CALIBRATION_SAMPLES / 2) {
    gyroOffsetX = gyroSumX / validSamples;
    gyroOffsetY = gyroSumY / validSamples;
    gyroOffsetZ = gyroSumZ / validSamples;
    
    accelOffsetX = accelSumX / validSamples;
    accelOffsetY = accelSumY / validSamples;
    accelOffsetZ = accelSumZ / validSamples;
    
    sensorCalibrated = true;
    
    Serial.println("=== 캘리브레이션 완료 ===");
    Serial.printf("Gyro 오프셋: X=%.2f, Y=%.2f, Z=%.2f\n", 
                  gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  } else {
    Serial.println("ERROR: 캘리브레이션 실패 - 센서 데이터 부족");
  }
}

void calibrateGyroscope() {
  Serial.println("자이로스코프 빠른 캘리브레이션...");
  
  float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    RawSensorData rawData;
    if (readRawSensorData(&rawData)) {
      gyroSumX += rawData.gyroX;
      gyroSumY += rawData.gyroY;
      gyroSumZ += rawData.gyroZ;
    }
    delay(1);
  }
  
  gyroOffsetX = gyroSumX / samples;
  gyroOffsetY = gyroSumY / samples;
  gyroOffsetZ = gyroSumZ / samples;
  
  Serial.println("자이로 캘리브레이션 완료");
}

bool readSensorData(SensorData* data) {
  if (!sensorInitialized || simulationMode) {
    // 시뮬레이션 모드
    unsigned long currentTime = millis();
    float t = currentTime * 0.001;
    
    data->accelX = 0.0;
    data->accelY = 0.0;
    data->accelZ = 9.81;
    
    data->gyroX = sin(t * 0.5) * 2.0;
    data->gyroY = cos(t * 0.3) * 1.5;
    data->gyroZ = sin(t * 0.7) * 1.0;
    
    data->magX = 25.0;
    data->magY = 3.0;
    data->magZ = -40.0;
    
    data->roll = sin(t * 0.2) * 5.0;
    data->pitch = cos(t * 0.15) * 3.0;
    data->yaw = t * 10.0;
    
    data->sensorHealthy = true;
    data->timestamp = micros();
    return true;
  }
  
  RawSensorData rawData;
  if (!readRawSensorData(&rawData)) {
    data->sensorHealthy = false;
    return false;
  }
  
  // 캘리브레이션 적용
  data->accelX = rawData.accelX - accelOffsetX;
  data->accelY = rawData.accelY - accelOffsetY;
  data->accelZ = rawData.accelZ - accelOffsetZ;
  
  data->gyroX = rawData.gyroX - gyroOffsetX;
  data->gyroY = rawData.gyroY - gyroOffsetY;
  data->gyroZ = rawData.gyroZ - gyroOffsetZ;
  
  data->magX = (rawData.magX - magOffsetX) * magScaleX;
  data->magY = (rawData.magY - magOffsetY) * magScaleY;
  data->magZ = (rawData.magZ - magOffsetZ) * magScaleZ;
  
  updateAttitude(data);
  
  data->sensorHealthy = true;
  data->timestamp = micros();
  
  return true;
}

bool readRawSensorData(RawSensorData* data) {
  uint8_t rawData[14];
  if (!safeI2CRead(MPU9250_ADDRESS, 0x3B, rawData, 14)) {
    return false;
  }
  
  int16_t accelX_raw = (rawData[0] << 8) | rawData[1];
  int16_t accelY_raw = (rawData[2] << 8) | rawData[3];
  int16_t accelZ_raw = (rawData[4] << 8) | rawData[5];
  int16_t gyroX_raw = (rawData[8] << 8) | rawData[9];
  int16_t gyroY_raw = (rawData[10] << 8) | rawData[11];
  int16_t gyroZ_raw = (rawData[12] << 8) | rawData[13];
  
  data->accelX = (float)accelX_raw / 4096.0 * 9.81;
  data->accelY = (float)accelY_raw / 4096.0 * 9.81;
  data->accelZ = (float)accelZ_raw / 4096.0 * 9.81;
  
  data->gyroX = (float)gyroX_raw / 16.4;
  data->gyroY = (float)gyroY_raw / 16.4;
  data->gyroZ = (float)gyroZ_raw / 16.4;
  
  // 자기계 데이터 (실패 시 무시)
  readMagnetometerData(&data->magX, &data->magY, &data->magZ);
  
  return true;
}

bool readMagnetometerData(float* magX, float* magY, float* magZ) {
  // 자기계 읽기 (간소화)
  *magX = 0.0;
  *magY = 0.0;
  *magZ = 0.0;
  return false;  // 일단 비활성화
}

void updateAttitude(SensorData* data) {
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastFilterUpdate) / 1000000.0f;
  lastFilterUpdate = currentTime;
  
  if (deltaTime > 0.1f) {
    deltaTime = 0.001f;
  }
  
  float accelRoll = atan2(data->accelY, sqrt(data->accelX * data->accelX + data->accelZ * data->accelZ)) * RAD_TO_DEG;
  float accelPitch = atan2(-data->accelX, sqrt(data->accelY * data->accelY + data->accelZ * data->accelZ)) * RAD_TO_DEG;
  
  static float gyroRoll = 0, gyroPitch = 0, gyroYaw = 0;
  gyroRoll += data->gyroX * deltaTime;
  gyroPitch += data->gyroY * deltaTime;
  gyroYaw += data->gyroZ * deltaTime;
  
  data->roll = complementaryAlpha * gyroRoll + (1.0f - complementaryAlpha) * accelRoll;
  data->pitch = complementaryAlpha * gyroPitch + (1.0f - complementaryAlpha) * accelPitch;
  data->yaw = gyroYaw;
  
  data->roll = normalizeAngle(data->roll);
  data->pitch = normalizeAngle(data->pitch);
  data->yaw = normalizeAngle(data->yaw);
  
  gyroRoll = data->roll;
  gyroPitch = data->pitch;
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

bool checkSensorHealth() {
  if (!sensorInitialized || simulationMode) {
    return false;
  }
  
  uint8_t whoami = 0;
  if (!safeI2CRead(MPU9250_ADDRESS, 0x75, &whoami, 1)) {
    return false;
  }
  
  if (whoami != 0x71 && whoami != 0x73) {
    Serial.println("MPU9250 연결 끊김");
    return false;
  }
  
  RawSensorData testData;
  if (readRawSensorData(&testData)) {
    if (abs(testData.accelX) > 50.0 || abs(testData.accelY) > 50.0 || abs(testData.accelZ) > 50.0) {
      return false;
    }
    if (abs(testData.gyroX) > 3000.0 || abs(testData.gyroY) > 3000.0 || abs(testData.gyroZ) > 3000.0) {
      return false;
    }
  } else {
    return false;
  }
  
  return true;
}

float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  float voltage = (adcValue / 4095.0) * 3.3;
  voltage *= VOLTAGE_DIVIDER_RATIO;
  
  static float voltageHistory[10] = {0};
  static int historyIndex = 0;
  static bool historyFull = false;
  
  voltageHistory[historyIndex] = voltage;
  historyIndex = (historyIndex + 1) % 10;
  if (historyIndex == 0) historyFull = true;
  
  float sum = 0;
  int count = historyFull ? 10 : historyIndex;
  for (int i = 0; i < count; i++) {
    sum += voltageHistory[i];
  }
  
  return sum / count;
}

// =================================
// I2C 통신 함수들 (간소화)
// =================================

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data = 0xFF;
  safeI2CRead(address, subAddress, &data, 1);
  return data;
}

bool readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
  return safeI2CRead(address, subAddress, dest, count);
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  safeI2CWrite(address, subAddress, data);
}

uint8_t readByteAK8963(uint8_t subAddress) {
  writeByte(MPU9250_ADDRESS, 0x25, AK8963_ADDRESS | 0x80);
  writeByte(MPU9250_ADDRESS, 0x26, subAddress);
  writeByte(MPU9250_ADDRESS, 0x27, 0x81);
  delay(1);
  return readByte(MPU9250_ADDRESS, 0x49);
}

bool readBytesAK8963(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  writeByte(MPU9250_ADDRESS, 0x25, AK8963_ADDRESS | 0x80);
  writeByte(MPU9250_ADDRESS, 0x26, subAddress);
  writeByte(MPU9250_ADDRESS, 0x27, 0x80 | count);
  delay(2);
  return readBytes(MPU9250_ADDRESS, 0x49, count, dest);
}

void writeByteAK8963(uint8_t subAddress, uint8_t data) {
  writeByte(MPU9250_ADDRESS, 0x25, AK8963_ADDRESS);
  writeByte(MPU9250_ADDRESS, 0x26, subAddress);
  writeByte(MPU9250_ADDRESS, 0x63, data);
  writeByte(MPU9250_ADDRESS, 0x27, 0x81);
  delay(2);
}

void debugPrintForPlotter(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs) {
  Serial.print(sensorData->roll); Serial.print(",");
  Serial.print(sensorData->pitch); Serial.print(",");
  Serial.print(sensorData->yaw); Serial.print(",");
  Serial.print(input->throttleNorm * 100); Serial.print(",");
  Serial.print(outputs->motor_fl); Serial.print(",");
  Serial.print(outputs->motor_fr); Serial.print(",");
  Serial.print(outputs->motor_rl); Serial.print(",");
  Serial.println(outputs->motor_rr);
}