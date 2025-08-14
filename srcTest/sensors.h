/*
 * sensors.h - MPU9250 센서 인터페이스 헤더
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "config.h"

// config.h가 제대로 포함되었는지 확인
#ifndef CONFIG_H
#error "config.h must be included before sensors.h"
#endif

// 원시 센서 데이터 구조체
struct RawSensorData
{
	float accelX, accelY, accelZ; // m/s²
	float gyroX, gyroY, gyroZ;	  // deg/s
	float magX, magY, magZ;		  // µT
};

// 함수 선언
bool initializeSensors();
bool testMPU9250Connection();
bool configureMPU9250();
bool initializeAK8963();

void calibrateSensors();
void calibrateGyroscope();

bool readSensorData(SensorData *data);
bool readRawSensorData(RawSensorData *data);
bool readMagnetometerData(float *magX, float *magY, float *magZ);

void updateAttitude(SensorData *data);
float normalizeAngle(float angle);

bool checkSensorHealth();
float readBatteryVoltage();

// I2C 통신 함수들
uint8_t readByte(uint8_t address, uint8_t subAddress);
bool readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

uint8_t readByteAK8963(uint8_t subAddress);
bool readBytesAK8963(uint8_t subAddress, uint8_t count, uint8_t *dest);
void writeByteAK8963(uint8_t subAddress, uint8_t data);

#endif // SENSORS_H