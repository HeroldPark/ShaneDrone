/*
 * sensors.cpp - MPU9250 센서 데이터 처리 및 자세 융합
 * IMU 데이터 읽기, 필터링, 캘리브레이션
 */

#include "../include/sensors.h"
#include "../include/config.h"
#include <Arduino.h>

// 전역 변수
static float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
static float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
static float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;
static float magScaleX = 1, magScaleY = 1, magScaleZ = 1;

// 상보 필터 변수
static float complementaryAlpha = 0.98;
static unsigned long lastFilterUpdate = 0;

// 센서 상태
static bool sensorInitialized = false;
static bool sensorCalibrated = false;

// 전역 변수 추가
static bool simulationMode = false;

bool initializeSensors() {
  Serial.println("MPU9250 센서 초기화 중...");
  
  // I2C 초기화
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz
  
  Serial.println("I2C 초기화 완료");
  delay(100);

  // I2C 스캔 추가 (디버그용)
  Serial.println("I2C 장치 스캔 중...");
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C 장치 발견: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("I2C 장치가 발견되지 않았습니다!");
  } else {
    Serial.print("총 ");
    Serial.print(nDevices);
    Serial.println("개의 I2C 장치 발견");
  }

  // MPU9250 연결 확인
  if (!testMPU9250Connection()) {
    Serial.println("ERROR: MPU9250 연결 실패");

    // 대안 I2C 주소 시도
    Serial.println("대안 I2C 주소 시도 중...");
    // 일부 MPU9250 모듈은 0x69 주소를 사용
    uint8_t altAddress = 0x69;
    Wire.beginTransmission(altAddress);
    if (Wire.endTransmission() == 0) {
      Serial.println("0x69 주소에서 장치 발견됨");
      // config.h에서 주소 변경 필요
    }

    return false;
  }
  
  // MPU9250 설정
  if (!configureMPU9250()) {
    Serial.println("ERROR: MPU9250 설정 실패");
    return false;
  }
  
  // 자기계(AK8963) 초기화
  if (!initializeAK8963()) {
    Serial.println("WARNING: AK8963(자기계) 초기화 실패");
    // 자기계 없이도 동작 가능
  }
  
  Serial.println("센서 초기화 완료!");
  sensorInitialized = true;
  
  // 자동 캘리브레이션 수행
  Serial.println("센서 캘리브레이션 시작...");
  calibrateSensors();
  
  return true;
}

bool testMPU9250Connection() {
  uint8_t whoami = readByte(MPU9250_ADDRESS, 0x75); // WHO_AM_I 레지스터
  
  if (whoami == 0x71 || whoami == 0x73) { // MPU9250의 정상 응답값
    Serial.print("MPU9250 감지됨 (ID: 0x");
    Serial.print(whoami, HEX);
    Serial.println(")");
    return true;
  } else {
    Serial.print("MPU9250 ID 오류: 0x");
    Serial.println(whoami, HEX);
    return false;
  }
}

bool configureMPU9250() {
  // 소프트웨어 리셋
  writeByte(MPU9250_ADDRESS, 0x6B, 0x80);
  delay(100);
  
  // 클럭 소스 설정 (PLL with X-axis gyroscope)
  writeByte(MPU9250_ADDRESS, 0x6B, 0x01);
  delay(10);
  
  // 샘플률 설정 (1kHz)
  writeByte(MPU9250_ADDRESS, 0x19, 0x00); // SMPLRT_DIV = 0
  
  // 자이로 설정 (±2000dps, DLPF 41Hz)
  writeByte(MPU9250_ADDRESS, 0x1A, GYRO_DLPF_CFG); // CONFIG
  writeByte(MPU9250_ADDRESS, 0x1B, 0x18); // GYRO_CONFIG (2000dps)
  
  // 가속도계 설정 (±8g, DLPF 41Hz)
  writeByte(MPU9250_ADDRESS, 0x1C, 0x10); // ACCEL_CONFIG (8g)
  writeByte(MPU9250_ADDRESS, 0x1D, ACCEL_DLPF_CFG); // ACCEL_CONFIG2
  
  // I2C 마스터 활성화 (자기계 접근용)
  writeByte(MPU9250_ADDRESS, 0x6A, 0x20); // USER_CTRL
  writeByte(MPU9250_ADDRESS, 0x24, 0x0D); // I2C_MST_CTRL (400kHz)
  
  delay(10);
  
  Serial.println("MPU9250 설정 완료");
  return true;
}

bool initializeAK8963() {
  // AK8963 연결 테스트
  uint8_t whoami = readByteAK8963(0x00); // WIA
  if (whoami != 0x48) {
    Serial.print("AK8963 ID 오류: 0x");
    Serial.println(whoami, HEX);
    return false;
  }
  
  // 파워다운 모드
  writeByteAK8963(0x0A, 0x00);
  delay(10);
  
  // Fuse ROM 모드로 캘리브레이션 값 읽기
  writeByteAK8963(0x0A, 0x0F);
  delay(10);
  
  uint8_t asaX = readByteAK8963(0x10);
  uint8_t asaY = readByteAK8963(0x11);
  uint8_t asaZ = readByteAK8963(0x12);
  
  // 스케일 팩터 계산
  magScaleX = (asaX - 128) * 0.5 / 128 + 1;
  magScaleY = (asaY - 128) * 0.5 / 128 + 1;
  magScaleZ = (asaZ - 128) * 0.5 / 128 + 1;
  
  // 연속 측정 모드 (16-bit, 100Hz)
  writeByteAK8963(0x0A, 0x16);
  delay(10);
  
  Serial.println("AK8963 초기화 완료");
  return true;
}

void calibrateSensors() {
  if (!sensorInitialized) {
    Serial.println("ERROR: 센서가 초기화되지 않음");
    return;
  }
  
  Serial.println("센서 캘리브레이션 중... (드론을 평평한 곳에 두고 움직이지 마세요)");
  
  float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
  float accelSumX = 0, accelSumY = 0, accelSumZ = 0;
  float magSumX = 0, magSumY = 0, magSumZ = 0;
  
  int validSamples = 0;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    RawSensorData rawData;
    if (readRawSensorData(&rawData)) {
      gyroSumX += rawData.gyroX;
      gyroSumY += rawData.gyroY;
      gyroSumZ += rawData.gyroZ;
      
      accelSumX += rawData.accelX;
      accelSumY += rawData.accelY;
      // Z축 가속도는 중력 고려 (1g 빼기)
      accelSumZ += rawData.accelZ - 9.81;
      
      magSumX += rawData.magX;
      magSumY += rawData.magY;
      magSumZ += rawData.magZ;
      
      validSamples++;
    }
    
    // 진행률 표시
    if (i % 100 == 0) {
      Serial.print("캘리브레이션 진행률: ");
      Serial.print((i * 100) / CALIBRATION_SAMPLES);
      Serial.println("%");
    }
    
    delay(2);
  }
  
  if (validSamples > CALIBRATION_SAMPLES / 2) {
    // 오프셋 계산
    gyroOffsetX = gyroSumX / validSamples;
    gyroOffsetY = gyroSumY / validSamples;
    gyroOffsetZ = gyroSumZ / validSamples;
    
    accelOffsetX = accelSumX / validSamples;
    accelOffsetY = accelSumY / validSamples;
    accelOffsetZ = accelSumZ / validSamples;
    
    magOffsetX = magSumX / validSamples;
    magOffsetY = magSumY / validSamples;
    magOffsetZ = magSumZ / validSamples;
    
    sensorCalibrated = true;
    
    Serial.println("=== 캘리브레이션 완료 ===");
    Serial.print("Gyro 오프셋: X="); Serial.print(gyroOffsetX);
    Serial.print(", Y="); Serial.print(gyroOffsetY);
    Serial.print(", Z="); Serial.println(gyroOffsetZ);
    Serial.print("Accel 오프셋: X="); Serial.print(accelOffsetX);
    Serial.print(", Y="); Serial.print(accelOffsetY);
    Serial.print(", Z="); Serial.println(accelOffsetZ);
    
  } else {
    Serial.println("ERROR: 캘리브레이션 실패 - 센서 데이터 부족");
    sensorCalibrated = false;
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
  if (!sensorInitialized) {
    // 시뮬레이션 모드
    simulationMode = true;
    
    // 가상 센서 데이터 생성
    unsigned long currentTime = millis();
    float t = currentTime * 0.001; // 초 단위
    
    data->accelX = 0.0;
    data->accelY = 0.0;
    data->accelZ = 9.81; // 중력
    
    data->gyroX = sin(t * 0.5) * 2.0; // 약간의 노이즈
    data->gyroY = cos(t * 0.3) * 1.5;
    data->gyroZ = sin(t * 0.7) * 1.0;
    
    data->magX = 25.0;
    data->magY = 3.0;
    data->magZ = -40.0;
    
    // 자세 계산 (시뮬레이션)
    data->roll = sin(t * 0.2) * 5.0;
    data->pitch = cos(t * 0.15) * 3.0;
    data->yaw = t * 10.0; // 천천히 회전
    
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
  
  // 자세 융합 계산
  updateAttitude(data);
  
  data->sensorHealthy = true;
  data->timestamp = micros();
  
  return true;
}

bool readRawSensorData(RawSensorData* data) {
  // 가속도계와 자이로스코프 데이터 읽기 (14바이트)
  uint8_t rawData[14];
  if (!readBytes(MPU9250_ADDRESS, 0x3B, 14, rawData)) {
    return false;
  }
  
  // 16비트 데이터 조합
  int16_t accelX_raw = (rawData[0] << 8) | rawData[1];
  int16_t accelY_raw = (rawData[2] << 8) | rawData[3];
  int16_t accelZ_raw = (rawData[4] << 8) | rawData[5];
  // rawData[6], rawData[7]: 온도 (사용 안함)
  int16_t gyroX_raw = (rawData[8] << 8) | rawData[9];
  int16_t gyroY_raw = (rawData[10] << 8) | rawData[11];
  int16_t gyroZ_raw = (rawData[12] << 8) | rawData[13];
  
  // 가속도계 변환 (±8g → m/s²)
  data->accelX = (float)accelX_raw / 4096.0 * 9.81;
  data->accelY = (float)accelY_raw / 4096.0 * 9.81;
  data->accelZ = (float)accelZ_raw / 4096.0 * 9.81;
  
  // 자이로스코프 변환 (±2000dps → deg/s)
  data->gyroX = (float)gyroX_raw / 16.4;
  data->gyroY = (float)gyroY_raw / 16.4;
  data->gyroZ = (float)gyroZ_raw / 16.4;
  
  // 자기계 데이터 읽기
  readMagnetometerData(&data->magX, &data->magY, &data->magZ);
  
  return true;
}

bool readMagnetometerData(float* magX, float* magY, float* magZ) {
  // ST1 레지스터 확인 (데이터 준비 상태)
  uint8_t st1 = readByteAK8963(0x02);
  if (!(st1 & 0x01)) {
    return false; // 데이터 준비 안됨
  }
  
  // 자기계 데이터 읽기 (6바이트)
  uint8_t rawData[6];
  if (!readBytesAK8963(0x03, 6, rawData)) {
    return false;
  }
  
  // ST2 레지스터 확인 (오버플로우 체크)
  uint8_t st2 = readByteAK8963(0x09);
  if (st2 & 0x08) {
    return false; // 자기 센서 오버플로우
  }
  
  // 16비트 데이터 조합 (리틀 엔디안)
  int16_t magX_raw = (rawData[1] << 8) | rawData[0];
  int16_t magY_raw = (rawData[3] << 8) | rawData[2];
  int16_t magZ_raw = (rawData[5] << 8) | rawData[4];
  
  // µT 단위로 변환
  *magX = (float)magX_raw * 0.15;
  *magY = (float)magY_raw * 0.15;
  *magZ = (float)magZ_raw * 0.15;
  
  return true;
}

void updateAttitude(SensorData* data) {
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastFilterUpdate) / 1000000.0f;
  lastFilterUpdate = currentTime;
  
  if (deltaTime > 0.1f) { // 너무 긴 간격은 무시
    deltaTime = 0.001f;
  }
  
  // 가속도계로부터 Roll, Pitch 계산
  float accelRoll = atan2(data->accelY, sqrt(data->accelX * data->accelX + data->accelZ * data->accelZ)) * RAD_TO_DEG;
  float accelPitch = atan2(-data->accelX, sqrt(data->accelY * data->accelY + data->accelZ * data->accelZ)) * RAD_TO_DEG;
  
  // 자이로스코프 적분
  static float gyroRoll = 0, gyroPitch = 0, gyroYaw = 0;
  gyroRoll += data->gyroX * deltaTime;
  gyroPitch += data->gyroY * deltaTime;
  gyroYaw += data->gyroZ * deltaTime;
  
  // 상보 필터 적용
  data->roll = complementaryAlpha * gyroRoll + (1.0f - complementaryAlpha) * accelRoll;
  data->pitch = complementaryAlpha * gyroPitch + (1.0f - complementaryAlpha) * accelPitch;
  
  // Yaw는 자이로스코프만 사용 (자기계 융합은 추후 구현)
  data->yaw = gyroYaw;
  
  // 각도 범위 제한 (-180 ~ 180도)
  data->roll = normalizeAngle(data->roll);
  data->pitch = normalizeAngle(data->pitch);
  data->yaw = normalizeAngle(data->yaw);
  
  // 상보 필터용 자이로 각도 업데이트
  gyroRoll = data->roll;
  gyroPitch = data->pitch;
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

bool checkSensorHealth() {
  if (!sensorInitialized) {
    return false;
  }
  
  // MPU9250 연결 테스트
  uint8_t whoami = readByte(MPU9250_ADDRESS, 0x75);
  if (whoami != 0x71 && whoami != 0x73) {
    Serial.println("MPU9250 연결 끊김");
    return false;
  }
  
  // 센서 데이터 범위 체크
  RawSensorData testData;
  if (readRawSensorData(&testData)) {
    // 비현실적인 값 체크
    if (abs(testData.accelX) > 50.0 || abs(testData.accelY) > 50.0 || abs(testData.accelZ) > 50.0) {
      Serial.println("가속도계 값 이상");
      return false;
    }
    
    if (abs(testData.gyroX) > 3000.0 || abs(testData.gyroY) > 3000.0 || abs(testData.gyroZ) > 3000.0) {
      Serial.println("자이로스코프 값 이상");
      return false;
    }
  } else {
    Serial.println("센서 데이터 읽기 실패");
    return false;
  }
  
  return true;
}

float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  
  // ADC 값을 전압으로 변환 (3.3V 기준, 12비트 ADC)
  float voltage = (adcValue / 4095.0) * 3.3;
  
  // 분압 저항 고려
  voltage *= VOLTAGE_DIVIDER_RATIO;
  
  // 이동평균 필터 적용 (노이즈 제거)
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
// I2C 통신 함수들
// =================================

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data;
  readBytes(address, subAddress, 1, &data);
  return data;
}

bool readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  uint8_t result = Wire.endTransmission(false);
  
  if (result != 0) {
    return false;
  }
  
  uint8_t bytesReceived = Wire.requestFrom(address, count);
  if (bytesReceived != count) {
    return false;
  }
  
  for (int i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
  
  return true;
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readByteAK8963(uint8_t subAddress) {
  // MPU9250을 통한 AK8963 읽기
  writeByte(MPU9250_ADDRESS, 0x25, AK8963_ADDRESS | 0x80); // I2C_SLV0_ADDR
  writeByte(MPU9250_ADDRESS, 0x26, subAddress); // I2C_SLV0_REG
  writeByte(MPU9250_ADDRESS, 0x27, 0x81); // I2C_SLV0_CTRL (enable + 1 byte)
  
  delay(1);
  
  return readByte(MPU9250_ADDRESS, 0x49); // EXT_SENS_DATA_00
}

bool readBytesAK8963(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  // MPU9250을 통한 AK8963 다중 바이트 읽기
  writeByte(MPU9250_ADDRESS, 0x25, AK8963_ADDRESS | 0x80); // I2C_SLV0_ADDR
  writeByte(MPU9250_ADDRESS, 0x26, subAddress); // I2C_SLV0_REG
  writeByte(MPU9250_ADDRESS, 0x27, 0x80 | count); // I2C_SLV0_CTRL
  
  delay(2);
  
  return readBytes(MPU9250_ADDRESS, 0x49, count, dest); // EXT_SENS_DATA_00~
}

void writeByteAK8963(uint8_t subAddress, uint8_t data) {
  // MPU9250을 통한 AK8963 쓰기
  writeByte(MPU9250_ADDRESS, 0x25, AK8963_ADDRESS); // I2C_SLV0_ADDR
  writeByte(MPU9250_ADDRESS, 0x26, subAddress); // I2C_SLV0_REG
  writeByte(MPU9250_ADDRESS, 0x63, data); // I2C_SLV0_DO
  writeByte(MPU9250_ADDRESS, 0x27, 0x81); // I2C_SLV0_CTRL (enable + 1 byte)
  
  delay(2);
}

// sensors.cpp에 추가
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