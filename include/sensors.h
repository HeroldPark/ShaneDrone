/*
 * sensors.h - 센서 관련 함수 선언 및 구조체
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

// Raw 센서 데이터 구조체
struct RawSensorData {
  float accelX, accelY, accelZ;  // m/s²
  float gyroX, gyroY, gyroZ;     // deg/s
  float magX, magY, magZ;        // µT
};

// 센서 초기화 및 설정
bool initializeSensors();
bool testMPU9250Connection();
bool configureMPU9250();
bool initializeAK8963();

// 센서 캘리브레이션
void calibrateSensors();
void calibrateGyroscope();

// 센서 데이터 읽기
bool readSensorData(SensorData* data);
bool readRawSensorData(RawSensorData* data);
bool readMagnetometerData(float* magX, float* magY, float* magZ);

// 자세 융합 및 계산
void updateAttitude(SensorData* data);
float normalizeAngle(float angle);

// 센서 상태 모니터링
bool checkSensorHealth();

// 배터리 전압 측정
float readBatteryVoltage();

// I2C 통신 함수
uint8_t readByte(uint8_t address, uint8_t subAddress);
bool readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

// AK8963 (자기계) 통신 함수
uint8_t readByteAK8963(uint8_t subAddress);
bool readBytesAK8963(uint8_t subAddress, uint8_t count, uint8_t* dest);
void writeByteAK8963(uint8_t subAddress, uint8_t data);

#endif // SENSORS_H