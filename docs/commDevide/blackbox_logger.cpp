// ============== blackbox_logger.cpp ==============
#include "blackbox_logger.h"

BlackboxLogger::BlackboxLogger() :
    sdInitialized(false),
    logging(false),
    bufferIndex(0),
    lastFlushTime(0),
    logTypeMask(LOG_ALL),
    logRate(100),  // 기본 100Hz
    lastLogTime(0),
    logCount(0),
    dropCount(0),
    fileSize(0) {
}

bool BlackboxLogger::begin(uint8_t csPin) {
    Serial.println("[Blackbox] 초기화 시작");
    
    // SD 카드 초기화
    if (!SD.begin(csPin)) {
        Serial.println("[Blackbox] SD 카드 초기화 실패");
        return false;
    }
    
    sdInitialized = true;
    Serial.println("[Blackbox] SD 카드 초기화 성공");
    
    // SD 카드 정보 출력
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.print("[Blackbox] SD 카드 크기: ");
    Serial.print(cardSize);
    Serial.println(" MB");
    
    return true;
}

void BlackboxLogger::stop() {
    if (logging) {
        stopLogging();
    }
    
    SD.end();
    sdInitialized = false;
}

bool BlackboxLogger::startLogging() {
    if (!sdInitialized) {
        Serial.println("[Blackbox] SD 카드가 초기화되지 않음");
        return false;
    }
    
    if (logging) {
        Serial.println("[Blackbox] 이미 로깅 중");
        return true;
    }
    
    // 새 로그 파일 생성
    if (!createNewLogFile()) {
        return false;
    }
    
    // 헤더 작성
    if (!writeHeader()) {
        closeLogFile();
        return false;
    }
    
    logging = true;
    logCount = 0;
    dropCount = 0;
    
    Serial.print("[Blackbox] 로깅 시작: ");
    Serial.println(currentFileName);
    
    return true;
}

void BlackboxLogger::stopLogging() {
    if (!logging) return;
    
    // 버퍼 플러시
    flushBuffer();
    
    // 파일 닫기
    closeLogFile();
    
    logging = false;
    
    Serial.println("[Blackbox] 로깅 중지");
    Serial.print("[Blackbox] 로그 수: ");
    Serial.print(logCount);
    Serial.print(", 드롭: ");
    Serial.println(dropCount);
}

bool BlackboxLogger::createNewLogFile() {
    currentFileName = generateFileName();
    
    // 파일 열기
    logFile = SD.open(currentFileName, FILE_WRITE);
    if (!logFile) {
        Serial.print("[Blackbox] 파일 생성 실패: ");
        Serial.println(currentFileName);
        return false;
    }
    
    fileSize = 0;
    bufferIndex = 0;
    
    return true;
}

void BlackboxLogger::closeLogFile() {
    if (logFile) {
        logFile.close();
    }
}

String BlackboxLogger::generateFileName() {
    // 기존 파일 번호 찾기
    int fileNumber = 1;
    String fileName;
    
    do {
        fileName = String(LOG_FILE_PREFIX) + String(fileNumber) + String(LOG_FILE_EXTENSION);
        fileNumber++;
    } while (SD.exists(fileName) && fileNumber < 1000);
    
    return fileName;
}

bool BlackboxLogger::writeHeader() {
    String header = "Time,Roll,Pitch,Yaw,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,";
    header += "MotorFL,MotorFR,MotorRL,MotorRR,";
    header += "Throttle,RollIn,PitchIn,YawIn,";
    header += "Battery,Armed,Mode\n";
    
    return addToBuffer(header.c_str());
}

void BlackboxLogger::logData(const LogEntry& entry) {
    if (!logging) return;
    
    // 로그 레이트 체크
    unsigned long currentTime = millis();
    unsigned long logInterval = 1000 / logRate;
    
    if (currentTime - lastLogTime < logInterval) {
        return;
    }
    
    // CSV 형식으로 데이터 포맷
    char line[256];
    snprintf(line, sizeof(line),
        "%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%d,%u\n",
        entry.timestamp,
        entry.roll, entry.pitch, entry.yaw,
        entry.gyroX, entry.gyroY, entry.gyroZ,
        entry.accX, entry.accY, entry.accZ,
        entry.motorFL, entry.motorFR, entry.motorRL, entry.motorRR,
        entry.throttle, entry.rollInput, entry.pitchInput, entry.yawInput,
        entry.batteryVoltage,
        entry.armed ? 1 : 0,
        entry.flightMode
    );
    
    if (addToBuffer(line)) {
        logCount++;
        lastLogTime = currentTime;
    } else {
        dropCount++;
    }
    
    // 주기적 플러시
    if (currentTime - lastFlushTime > LOG_FLUSH_INTERVAL) {
        flushBuffer();
        lastFlushTime = currentTime;
    }
    
    // 파일 크기 체크
    if (fileSize > MAX_LOG_FILE_SIZE) {
        Serial.println("[Blackbox] 최대 파일 크기 도달, 새 파일 생성");
        stopLogging();
        startLogging();
    }
}

void BlackboxLogger::logData(const SensorData* sensors, const ControllerInput* input,
                             const MotorOutputs* motors, float battery, bool armed, uint8_t mode) {
    LogEntry entry;
    
    entry.timestamp = millis();
    
    // 센서 데이터
    entry.roll = sensors->roll;
    entry.pitch = sensors->pitch;
    entry.yaw = sensors->yaw;
    entry.gyroX = sensors->gyroX;
    entry.gyroY = sensors->gyroY;
    entry.gyroZ = sensors->gyroZ;
    entry.accX = sensors->accX;
    entry.accY = sensors->accY;
    entry.accZ = sensors->accZ;
    
    // 모터 출력
    entry.motorFL = motors->motor_fl;
    entry.motorFR = motors->motor_fr;
    entry.motorRL = motors->motor_rl;
    entry.motorRR = motors->motor_rr;
    
    // RC 입력
    entry.throttle = input->throttle;
    entry.rollInput = input->roll;
    entry.pitchInput = input->pitch;
    entry.yawInput = input->yaw;
    
    // 시스템 상태
    entry.batteryVoltage = battery;
    entry.armed = armed;
    entry.flightMode = mode;
    
    logData(entry);
}

void BlackboxLogger::logMessage(const String& message) {
    if (!logging) return;
    
    String logLine = String(millis()) + ",MSG," + message + "\n";
    addToBuffer(logLine.c_str());
}

void BlackboxLogger::logEvent(const String& event) {
    if (!logging) return;
    
    String logLine = String(millis()) + ",EVENT," + event + "\n";
    addToBuffer(logLine.c_str());
    
    // 이벤트는 즉시 플러시
    flushBuffer();
}

bool BlackboxLogger::addToBuffer(const char* data) {
    size_t dataLen = strlen(data);
    
    if (bufferIndex + dataLen >= LOG_BUFFER_SIZE) {
        // 버퍼가 가득 차면 플러시
        flushBuffer();
    }
    
    if (dataLen >= LOG_BUFFER_SIZE) {
        // 데이터가 버퍼보다 크면 직접 쓰기
        if (logFile) {
            size_t written = logFile.write(data, dataLen);
            fileSize += written;
            return written == dataLen;
        }
        return false;
    }
    
    // 버퍼에 추가
    memcpy(buffer + bufferIndex, data, dataLen);
    bufferIndex += dataLen;
    
    return true;
}

void BlackboxLogger::flushBuffer() {
    if (bufferIndex == 0 || !logFile) return;
    
    size_t written = logFile.write((uint8_t*)buffer, bufferIndex);
    if (written != bufferIndex) {
        Serial.println("[Blackbox] 버퍼 플러시 실패");
    }
    
    fileSize += written;
    bufferIndex = 0;
    
    // 파일 동기화
    logFile.flush();
}

bool BlackboxLogger::listLogFiles() {
    if (!sdInitialized) return false;
    
    File root = SD.open("/");
    if (!root) return false;
    
    Serial.println("[Blackbox] 로그 파일 목록:");
    
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            String fileName = file.name();
            if (fileName.startsWith(LOG_FILE_PREFIX)) {
                Serial.print("  ");
                Serial.print(fileName);
                Serial.print(" (");
                Serial.print(file.size());
                Serial.println(" bytes)");
            }
        }
        file = root.openNextFile();
    }
    
    root.close();
    return true;
}

bool BlackboxLogger::deleteLogFile(const String& filename) {
    if (!sdInitialized) return false;
    
    if (SD.remove(filename)) {
        Serial.print("[Blackbox] 파일 삭제됨: ");
        Serial.println(filename);
        return true;
    }
    
    return false;
}

bool BlackboxLogger::deleteAllLogs() {
    if (!sdInitialized) return false;
    
    File root = SD.open("/");
    if (!root) return false;
    
    int deleteCount = 0;
    File file = root.openNextFile();
    
    while (file) {
        String fileName = file.name();
        file.close();
        
        if (fileName.startsWith(LOG_FILE_PREFIX)) {
            if (SD.remove(fileName)) {
                deleteCount++;
            }
        }
        
        file = root.openNextFile();
    }
    
    root.close();
    
    Serial.print("[Blackbox] ");
    Serial.print(deleteCount);
    Serial.println(" 개 파일 삭제됨");
    
    return true;
}

void BlackboxLogger::resetStatistics() {
    logCount = 0;
    dropCount = 0;
}

bool BlackboxLogger::isSDCardPresent() {
    // SD 카드 존재 여부 체크
    return sdInitialized && SD.cardType() != CARD_NONE;
}

unsigned long BlackboxLogger::getSDCardSize() {
    if (!sdInitialized) return 0;
    return SD.cardSize() / 1024;  // KB 단위
}

unsigned long BlackboxLogger::getSDCardFree() {
    if (!sdInitialized) return 0;
    return SD.totalBytes() - SD.usedBytes();
}