// ============== vtx_osd.h ==============
#ifndef VTX_OSD_H
#define VTX_OSD_H

#include <Arduino.h>
#include "config.h"
#include "sensors.h"

// MSP 프로토콜 정의
#define MSP_HEADER_SIZE 5
#define MSP_CHECKSUM_SIZE 1
#define MSP_PROTOCOL_V1 '$'
#define MSP_DIRECTION_IN '<'
#define MSP_DIRECTION_OUT '>'

// MSP 명령 코드
enum MSPCommand {
    MSP_STATUS = 101,
    MSP_RAW_IMU = 102,
    MSP_ATTITUDE = 108,
    MSP_ANALOG = 110,
    MSP_RC = 105,
    MSP_RAW_GPS = 106,
    MSP_COMP_GPS = 107,
    MSP_ALTITUDE = 109,
    MSP_BATTERY = 130
};

// VTX 설정
struct VTXConfig {
    uint8_t power;      // 0-3 (25mW, 100mW, 200mW, 400mW)
    uint16_t frequency; // MHz
    uint8_t channel;    // 1-8
    uint8_t band;       // 1-5 (A, B, E, F, R)
};

class VTXOSD {
private:
    HardwareSerial* serial;
    bool connected;
    unsigned long lastSendTime;
    VTXConfig config;
    
    // MSP 패킷 생성
    void sendMSPPacket(uint8_t command, uint8_t* data, uint8_t dataSize);
    uint8_t calculateChecksum(uint8_t* data, uint8_t size);
    
    // AT 명령 전송
    bool sendATCommand(const String& command);
    String readATResponse(unsigned long timeout = 100);
    
public:
    VTXOSD();
    
    bool begin(HardwareSerial* serialPort, int rxPin, int txPin);
    bool connect();
    void disconnect();
    bool isConnected() const { return connected; }
    
    // VTX 설정
    bool setPower(uint8_t power);
    bool setFrequency(uint16_t freq);
    bool setChannel(uint8_t band, uint8_t channel);
    
    // OSD 데이터 전송
    void sendAttitude(float roll, float pitch, float yaw);
    void sendBattery(float voltage, float current = 0, uint16_t mah = 0);
    void sendGPS(double lat = 0, double lon = 0, uint8_t sats = 0, uint16_t altitude = 0);
    void sendStatus(bool armed, uint8_t flightMode);
    void sendRawIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void sendAltitude(float altitude, float vario = 0);
    
    // 전체 텔레메트리 업데이트
    void updateTelemetry(const SensorData* sensors, float battery, bool armed, uint8_t mode);
};

#endif // VTX_OSD_H