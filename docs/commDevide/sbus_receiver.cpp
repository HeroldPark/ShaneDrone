// ============== sbus_receiver.cpp ==============
#include "sbus_receiver.h"

SBUSReceiver::SBUSReceiver() : 
    serial(nullptr), 
    packetIndex(0), 
    lastByteTime(0) {
    
    // 채널 초기화 (중앙값)
    for (int i = 0; i < SBUS_CHANNEL_COUNT; i++) {
        data.channels[i] = 1500;
    }
    data.failsafe = false;
    data.lostFrame = false;
    data.connected = false;
    data.lastReceived = 0;
}

bool SBUSReceiver::begin(HardwareSerial* serialPort, int rxPin) {
    if (!serialPort) return false;
    
    serial = serialPort;
    serial->begin(SBUS_BAUDRATE, SERIAL_8E2, rxPin, -1, SBUS_INVERT);
    
    Serial.println("[SBUS] 수신기 초기화 완료");
    return true;
}

bool SBUSReceiver::read() {
    if (!serial) return false;
    
    unsigned long currentTime = millis();
    bool packetReceived = false;
    
    // 패킷 타임아웃 체크
    if (currentTime - lastByteTime > SBUS_PACKET_TIMEOUT_MS && packetIndex > 0) {
        packetIndex = 0;
    }
    
    // SBUS 데이터 읽기
    while (serial->available()) {
        uint8_t inByte = serial->read();
        lastByteTime = currentTime;
        
        // 헤더 체크
        if (packetIndex == 0 && inByte != SBUS_HEADER) {
            continue;
        }
        
        packet[packetIndex++] = inByte;
        
        // 패킷 완성
        if (packetIndex >= SBUS_PACKET_SIZE) {
            packetIndex = 0;
            
            if (packet[24] == SBUS_FOOTER) {
                parsePacket();
                data.lastReceived = currentTime;
                data.connected = true;
                packetReceived = true;
            }
        }
    }
    
    // 연결 타임아웃 체크
    if (currentTime - data.lastReceived > SBUS_TIMEOUT_MS) {
        data.connected = false;
        data.failsafe = true;
    }
    
    return packetReceived;
}

void SBUSReceiver::parsePacket() {
    // 11비트 채널 데이터 파싱
    data.channels[0]  = ((packet[1] | packet[2] << 8) & 0x07FF);
    data.channels[1]  = ((packet[2] >> 3 | packet[3] << 5) & 0x07FF);
    data.channels[2]  = ((packet[3] >> 6 | packet[4] << 2 | packet[5] << 10) & 0x07FF);
    data.channels[3]  = ((packet[5] >> 1 | packet[6] << 7) & 0x07FF);
    data.channels[4]  = ((packet[6] >> 4 | packet[7] << 4) & 0x07FF);
    data.channels[5]  = ((packet[7] >> 7 | packet[8] << 1 | packet[9] << 9) & 0x07FF);
    data.channels[6]  = ((packet[9] >> 2 | packet[10] << 6) & 0x07FF);
    data.channels[7]  = ((packet[10] >> 5 | packet[11] << 3) & 0x07FF);
    data.channels[8]  = ((packet[12] | packet[13] << 8) & 0x07FF);
    data.channels[9]  = ((packet[13] >> 3 | packet[14] << 5) & 0x07FF);
    data.channels[10] = ((packet[14] >> 6 | packet[15] << 2 | packet[16] << 10) & 0x07FF);
    data.channels[11] = ((packet[16] >> 1 | packet[17] << 7) & 0x07FF);
    data.channels[12] = ((packet[17] >> 4 | packet[18] << 4) & 0x07FF);
    data.channels[13] = ((packet[18] >> 7 | packet[19] << 1 | packet[20] << 9) & 0x07FF);
    data.channels[14] = ((packet[20] >> 2 | packet[21] << 6) & 0x07FF);
    data.channels[15] = ((packet[21] >> 5 | packet[22] << 3) & 0x07FF);
    
    // SBUS 값을 마이크로초로 변환
    for (int i = 0; i < 16; i++) {
        data.channels[i] = sbusToMicroseconds(data.channels[i]);
    }
    
    // 디지털 채널 (채널 17, 18)
    data.channels[16] = (packet[23] & 0x01) ? 2000 : 1000;
    data.channels[17] = (packet[23] & 0x02) ? 2000 : 1000;
    
    // 상태 플래그
    data.failsafe = (packet[23] & 0x08) != 0;
    data.lostFrame = (packet[23] & 0x04) != 0;
}

uint16_t SBUSReceiver::sbusToMicroseconds(uint16_t sbusValue) {
    // 172~1811 -> 1000~2000 μs
    uint16_t value = map(sbusValue, 172, 1811, 1000, 2000);
    return constrain(value, 1000, 2000);
}

bool SBUSReceiver::isConnected() const {
    return data.connected && !data.failsafe;
}

uint16_t SBUSReceiver::getChannel(uint8_t channel) const {
    if (channel < 1 || channel > SBUS_CHANNEL_COUNT) return 1500;
    return data.channels[channel - 1];
}

void SBUSReceiver::debugPrint() const {
    Serial.println("=== SBUS 디버그 정보 ===");
    Serial.print("연결: ");
    Serial.print(data.connected ? "예" : "아니오");
    Serial.print(" | Failsafe: ");
    Serial.print(data.failsafe ? "예" : "아니오");
    Serial.print(" | Lost Frame: ");
    Serial.println(data.lostFrame ? "예" : "아니오");
    
    Serial.print("채널: ");
    for (int i = 0; i < 8; i++) {
        Serial.print("CH");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(data.channels[i]);
        Serial.print(" ");
    }
    Serial.println();
    
    Serial.print("마지막 수신: ");
    Serial.print(millis() - data.lastReceived);
    Serial.println("ms 전");
}