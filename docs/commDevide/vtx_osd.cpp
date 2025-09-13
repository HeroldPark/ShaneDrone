// ============== vtx_osd.cpp ==============
#include "vtx_osd.h"

VTXOSD::VTXOSD() : 
    serial(nullptr),
    connected(false),
    lastSendTime(0) {
    
    config.power = 1;      // 기본 100mW
    config.frequency = 5800;
    config.channel = 1;
    config.band = 1;
}

bool VTXOSD::begin(HardwareSerial* serialPort, int rxPin, int txPin) {
    if (!serialPort) return false;
    
    serial = serialPort;
    serial->begin(115200, SERIAL_8N1, rxPin, txPin);
    
    Serial.println("[VTX] OSD 초기화 시작");
    
    delay(100);
    
    // VTX 연결 시도
    if (connect()) {
        Serial.println("[VTX] Walksnail Avatar HD V2 연결 성공");
        return true;
    }
    
    Serial.println("[VTX] VTX 연결 실패");
    return false;
}

bool VTXOSD::connect() {
    // AT 명령으로 연결 테스트
    if (!sendATCommand("AT")) {
        return false;
    }
    
    String response = readATResponse();
    if (response.indexOf("OK") < 0) {
        return false;
    }
    
    // VTX 초기 설정
    sendATCommand("AT+POWER=" + String(config.power));
    delay(10);
    sendATCommand("AT+FREQ=" + String(config.frequency));
    delay(10);
    
    connected = true;
    return true;
}

void VTXOSD::disconnect() {
    connected = false;
    if (serial) {
        serial->end();
    }
}

bool VTXOSD::sendATCommand(const String& command) {
    if (!serial) return false;
    
    serial->print(command);
    serial->print("\r\n");
    return true;
}

String VTXOSD::readATResponse(unsigned long timeout) {
    unsigned long start = millis();
    String response = "";
    
    while (millis() - start < timeout) {
        if (serial->available()) {
            char c = serial->read();
            response += c;
            if (response.endsWith("\r\n")) {
                break;
            }
        }
    }
    
    return response;
}

bool VTXOSD::setPower(uint8_t power) {
    if (power > 3) power = 3;
    
    config.power = power;
    
    if (connected) {
        return sendATCommand("AT+POWER=" + String(power));
    }
    
    return false;
}

bool VTXOSD::setFrequency(uint16_t freq) {
    config.frequency = freq;
    
    if (connected) {
        return sendATCommand("AT+FREQ=" + String(freq));
    }
    
    return false;
}

bool VTXOSD::setChannel(uint8_t band, uint8_t channel) {
    if (band < 1 || band > 5) return false;
    if (channel < 1 || channel > 8) return false;
    
    config.band = band;
    config.channel = channel;
    
    // 주파수 계산 (5.8GHz 대역)
    const uint16_t frequencies[5][8] = {
        // Band A
        {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725},
        // Band B
        {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866},
        // Band E
        {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945},
        // Band F
        {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880},
        // Band R
        {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}
    };
    
    uint16_t freq = frequencies[band - 1][channel - 1];
    return setFrequency(freq);
}

void VTXOSD::sendMSPPacket(uint8_t command, uint8_t* data, uint8_t dataSize) {
    if (!serial || !connected) return;
    
    uint8_t packet[255];
    uint8_t idx = 0;
    
    // 헤더
    packet[idx++] = '$';
    packet[idx++] = 'M';
    packet[idx++] = '<';
    packet[idx++] = dataSize;
    packet[idx++] = command;
    
    // 데이터
    for (uint8_t i = 0; i < dataSize; i++) {
        packet[idx++] = data[i];
    }
    
    // 체크섬
    uint8_t checksum = 0;
    for (uint8_t i = 3; i < idx; i++) {
        checksum ^= packet[i];
    }
    packet[idx++] = checksum;
    
    // 전송
    serial->write(packet, idx);
}

void VTXOSD::sendAttitude(float roll, float pitch, float yaw) {
    uint8_t data[6];
    
    // 각도를 0.1도 단위로 변환
    int16_t rollInt = (int16_t)(roll * 10);
    int16_t pitchInt = (int16_t)(pitch * 10);
    int16_t yawInt = (int16_t)(yaw);
    
    data[0] = rollInt & 0xFF;
    data[1] = (rollInt >> 8) & 0xFF;
    data[2] = pitchInt & 0xFF;
    data[3] = (pitchInt >> 8) & 0xFF;
    data[4] = yawInt & 0xFF;
    data[5] = (yawInt >> 8) & 0xFF;
    
    sendMSPPacket(MSP_ATTITUDE, data, 6);
}

void VTXOSD::sendBattery(float voltage, float current, uint16_t mah) {
    uint8_t data[7];
    
    // 전압 (0.1V 단위)
    uint8_t vbat = (uint8_t)(voltage * 10);
    
    // 전류 (0.01A 단위)
    uint16_t currentInt = (uint16_t)(current * 100);
    
    data[0] = vbat;
    data[1] = mah & 0xFF;
    data[2] = (mah >> 8) & 0xFF;
    data[3] = currentInt & 0xFF;
    data[4] = (currentInt >> 8) & 0xFF;
    data[5] = 0; // RSSI
    data[6] = 0; // Amperage
    
    sendMSPPacket(MSP_ANALOG, data, 7);
}

void VTXOSD::sendGPS(double lat, double lon, uint8_t sats, uint16_t altitude) {
    uint8_t data[16];
    
    // GPS 상태
    data[0] = (sats > 0) ? 1 : 0; // Fix
    data[1] = sats;                // 위성 수
    
    // 위도 (1e-7 도 단위)
    int32_t latInt = (int32_t)(lat * 10000000);
    data[2] = latInt & 0xFF;
    data[3] = (latInt >> 8) & 0xFF;
    data[4] = (latInt >> 16) & 0xFF;
    data[5] = (latInt >> 24) & 0xFF;
    
    // 경도 (1e-7 도 단위)
    int32_t lonInt = (int32_t)(lon * 10000000);
    data[6] = lonInt & 0xFF;
    data[7] = (lonInt >> 8) & 0xFF;
    data[8] = (lonInt >> 16) & 0xFF;
    data[9] = (lonInt >> 24) & 0xFF;
    
    // 고도 (미터)
    data[10] = altitude & 0xFF;
    data[11] = (altitude >> 8) & 0xFF;
    
    // 속도 (cm/s)
    data[12] = 0;
    data[13] = 0;
    
    // 지면 코스
    data[14] = 0;
    data[15] = 0;
    
    sendMSPPacket(MSP_RAW_GPS, data, 16);
}

void VTXOSD::sendStatus(bool armed, uint8_t flightMode) {
    uint8_t data[11];
    
    // 상태 플래그
    uint16_t flags = 0;
    if (armed) flags |= 0x01;
    
    data[0] = 0; // 사이클 타임
    data[1] = 0;
    data[2] = 0; // I2C 에러
    data[3] = 0;
    data[4] = flags & 0xFF; // 센서 플래그
    data[5] = (flags >> 8) & 0xFF;
    data[6] = flightMode; // 비행 모드
    data[7] = 0; // 프로파일
    data[8] = 0; // 시스템 로드
    data[9] = 0;
    data[10] = 0; // 자이로/사이클 타임
    
    sendMSPPacket(MSP_STATUS, data, 11);
}

void VTXOSD::sendRawIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    uint8_t data[18];
    
    // 가속도 (512 = 1g)
    int16_t axInt = (int16_t)(ax * 512);
    int16_t ayInt = (int16_t)(ay * 512);
    int16_t azInt = (int16_t)(az * 512);
    
    // 자이로 (도/초)
    int16_t gxInt = (int16_t)(gx);
    int16_t gyInt = (int16_t)(gy);
    int16_t gzInt = (int16_t)(gz);
    
    // 가속도
    data[0] = axInt & 0xFF;
    data[1] = (axInt >> 8) & 0xFF;
    data[2] = ayInt & 0xFF;
    data[3] = (ayInt >> 8) & 0xFF;
    data[4] = azInt & 0xFF;
    data[5] = (azInt >> 8) & 0xFF;
    
    // 자이로
    data[6] = gxInt & 0xFF;
    data[7] = (gxInt >> 8) & 0xFF;
    data[8] = gyInt & 0xFF;
    data[9] = (gyInt >> 8) & 0xFF;
    data[10] = gzInt & 0xFF;
    data[11] = (gzInt >> 8) & 0xFF;
    
    // 자력계 (미사용)
    for (int i = 12; i < 18; i++) {
        data[i] = 0;
    }
    
    sendMSPPacket(MSP_RAW_IMU, data, 18);
}

void VTXOSD::sendAltitude(float altitude, float vario) {
    uint8_t data[6];
    
    // 고도 (cm)
    int32_t altInt = (int32_t)(altitude * 100);
    
    // 수직 속도 (cm/s)
    int16_t varioInt = (int16_t)(vario * 100);
    
    data[0] = altInt & 0xFF;
    data[1] = (altInt >> 8) & 0xFF;
    data[2] = (altInt >> 16) & 0xFF;
    data[3] = (altInt >> 24) & 0xFF;
    data[4] = varioInt & 0xFF;
    data[5] = (varioInt >> 8) & 0xFF;
    
    sendMSPPacket(MSP_ALTITUDE, data, 6);
}

void VTXOSD::updateTelemetry(const SensorData* sensors, float battery, bool armed, uint8_t mode) {
    if (!connected) return;
    
    // 전송 주기 제한 (50Hz)
    unsigned long currentTime = millis();
    if (currentTime - lastSendTime < 20) return;
    
    // 자세 데이터
    sendAttitude(sensors->roll, sensors->pitch, sensors->yaw);
    
    // 배터리 데이터
    sendBattery(battery);
    
    // IMU 데이터
    sendRawIMU(sensors->gyroX, sensors->gyroY, sensors->gyroZ,
               sensors->accX, sensors->accY, sensors->accZ);
    
    // 상태 정보
    sendStatus(armed, mode);
    
    // GPS (더미 데이터)
    sendGPS();
    
    lastSendTime = currentTime;
}