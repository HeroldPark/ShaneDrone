// ============== blackbox_logger.h ==============
#ifndef BLACKBOX_LOGGER_H
#define BLACKBOX_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "sensors.h"
#include "control.h"

// 로깅 설정
#define LOG_BUFFER_SIZE 512
#define LOG_FLUSH_INTERVAL 1000  // ms
#define LOG_FILE_PREFIX "flight_"
#define LOG_FILE_EXTENSION ".csv"
#define MAX_LOG_FILE_SIZE 10485760  // 10MB

// 로그 타입
enum LogType {
    LOG_ATTITUDE = 0x01,
    LOG_GYRO = 0x02,
    LOG_ACC = 0x04,
    LOG_MOTOR = 0x08,
    LOG_RC_INPUT = 0x10,
    LOG_BATTERY = 0x20,
    LOG_GPS = 0x40,
    LOG_ALL = 0xFF
};

// 로그 엔트리 구조체
struct LogEntry {
    unsigned long timestamp;
    
    // 자세
    float roll;
    float pitch;
    float yaw;
    
    // 센서
    float gyroX, gyroY, gyroZ;
    float accX, accY, accZ;
    
    // 모터
    uint16_t motorFL, motorFR, motorRL, motorRR;
    
    // RC 입력
    uint16_t throttle, rollInput, pitchInput, yawInput;
    
    // 시스템
    float batteryVoltage;
    bool armed;
    uint8_t flightMode;
};

class BlackboxLogger {
private:
    bool sdInitialized;
    bool logging;
    File logFile;
    String currentFileName;
    
    // 버퍼
    char buffer[LOG_BUFFER_SIZE];
    uint16_t bufferIndex;
    unsigned long lastFlushTime;
    
    // 로그 설정
    uint8_t logTypeMask;
    uint16_t logRate;  // Hz
    unsigned long lastLogTime;
    
    // 통계
    unsigned long logCount;
    unsigned long dropCount;
    unsigned long fileSize;
    
    // 파일 관리
    bool createNewLogFile();
    void closeLogFile();
    String generateFileName();
    bool writeHeader();
    
    // 버퍼 관리
    void flushBuffer();
    bool addToBuffer(const char* data);
    
public:
    BlackboxLogger();
    
    bool begin(uint8_t csPin = SS);
    void stop();
    
    // 로깅 제어
    bool startLogging();
    void stopLogging();
    bool isLogging() const { return logging; }
    
    // 로그 설정
    void setLogTypes(uint8_t types) { logTypeMask = types; }
    void setLogRate(uint16_t rate) { logRate = rate; }
    
    // 데이터 로깅
    void logData(const LogEntry& entry);
    void logData(const SensorData* sensors, const ControllerInput* input,
                 const MotorOutputs* motors, float battery, bool armed, uint8_t mode);
    
    // 수동 로그 메시지
    void logMessage(const String& message);
    void logEvent(const String& event);
    
    // 파일 관리
    bool listLogFiles();
    bool deleteLogFile(const String& filename);
    bool deleteAllLogs();
    unsigned long getFileSize() const { return fileSize; }
    
    // 통계
    unsigned long getLogCount() const { return logCount; }
    unsigned long getDropCount() const { return dropCount; }
    void resetStatistics();
    
    // SD 카드 정보
    bool isSDCardPresent();
    unsigned long getSDCardSize();
    unsigned long getSDCardFree();
};

#endif // BLACKBOX_LOGGER_H