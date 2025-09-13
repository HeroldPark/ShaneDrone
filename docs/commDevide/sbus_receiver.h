// ============== sbus_receiver.h ==============
#ifndef SBUS_RECEIVER_H
#define SBUS_RECEIVER_H

#include <Arduino.h>
#include "config.h"

// SBUS 상수 정의
#define SBUS_CHANNEL_COUNT 18
#define SBUS_PACKET_SIZE 25
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00
#define SBUS_BAUDRATE 100000
#define SBUS_INVERT true
#define SBUS_TIMEOUT_MS 100
#define SBUS_PACKET_TIMEOUT_MS 3

// SBUS 데이터 구조체
struct SBUSData {
    uint16_t channels[SBUS_CHANNEL_COUNT];
    bool failsafe;
    bool lostFrame;
    bool connected;
    unsigned long lastReceived;
};

class SBUSReceiver {
private:
    HardwareSerial* serial;
    uint8_t packet[SBUS_PACKET_SIZE];
    uint8_t packetIndex;
    unsigned long lastByteTime;
    SBUSData data;
    
    void parsePacket();
    uint16_t sbusToMicroseconds(uint16_t sbusValue);
    
public:
    SBUSReceiver();
    bool begin(HardwareSerial* serialPort, int rxPin);
    bool read();
    const SBUSData& getData() const { return data; }
    bool isConnected() const;
    bool isFailsafe() const { return data.failsafe; }
    uint16_t getChannel(uint8_t channel) const;
    void debugPrint() const;
};

#endif // SBUS_RECEIVER_H