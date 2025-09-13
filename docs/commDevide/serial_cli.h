// ============== serial_cli.h ==============
#ifndef SERIAL_CLI_H
#define SERIAL_CLI_H

#include <Arduino.h>
#include "config.h"

// CLI 설정
#define CLI_BUFFER_SIZE 128
#define CLI_MAX_ARGS 10

// 명령 콜백 타입
typedef void (*CliCommandCallback)(int argc, char** argv);

// CLI 명령 구조체
struct CliCommand {
    const char* name;
    const char* description;
    CliCommandCallback callback;
};

class SerialCLI {
private:
    Stream* stream;
    char buffer[CLI_BUFFER_SIZE];
    uint8_t bufferIndex;
    bool echoEnabled;
    
    // 명령 목록
    static const int MAX_COMMANDS = 20;
    CliCommand commands[MAX_COMMANDS];
    uint8_t commandCount;
    
    // 내장 명령 핸들러
    static SerialCLI* instance;
    static void helpCommand(int argc, char** argv);
    static void statusCommand(int argc, char** argv);
    static void calibrateCommand(int argc, char** argv);
    static void pidCommand(int argc, char** argv);
    static void motorCommand(int argc, char** argv);
    static void saveCommand(int argc, char** argv);
    static void loadCommand(int argc, char** argv);
    static void rebootCommand(int argc, char** argv);
    
    // 파싱 함수
    void processLine();
    int parseCommand(char* line, char** argv);
    void executeCommand(int argc, char** argv);
    
public:
    SerialCLI();
    
    bool begin(Stream* serialStream, unsigned long baudrate = 115200);
    void loop();
    
    // 명령 등록
    bool registerCommand(const char* name, const char* description, CliCommandCallback callback);
    void printPrompt();
    void println(const String& message);
    void print(const String& message);
    
    // 에코 설정
    void setEcho(bool enable) { echoEnabled = enable; }
    
    // 상태 콜백 (외부에서 설정)
    void (*getStatusCallback)();
    void (*calibrateCallback)();
    void (*setPIDCallback)(const char* axis, float kp, float ki, float kd);
    void (*motorTestCallback)(uint8_t motor, uint16_t value);
    void (*saveConfigCallback)();
    void (*loadConfigCallback)();
};

#endif // SERIAL_CLI_H