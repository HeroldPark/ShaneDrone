// ============== serial_cli.cpp ==============
#include "serial_cli.h"

// Static 멤버 초기화
SerialCLI* SerialCLI::instance = nullptr;

SerialCLI::SerialCLI() : 
    stream(nullptr),
    bufferIndex(0),
    echoEnabled(true),
    commandCount(0),
    getStatusCallback(nullptr),
    calibrateCallback(nullptr),
    setPIDCallback(nullptr),
    motorTestCallback(nullptr),
    saveConfigCallback(nullptr),
    loadConfigCallback(nullptr) {
    
    instance = this;
    memset(buffer, 0, CLI_BUFFER_SIZE);
}

bool SerialCLI::begin(Stream* serialStream, unsigned long baudrate) {
    if (!serialStream) return false;
    
    stream = serialStream;
    
    // Serial이면 baudrate 설정
    if (Serial* ser = dynamic_cast<Serial*>(stream)) {
        ser->begin(baudrate);
    }
    
    // 기본 명령 등록
    registerCommand("help", "사용 가능한 명령 표시", helpCommand);
    registerCommand("status", "시스템 상태 표시", statusCommand);
    registerCommand("calibrate", "센서 캘리브레이션", calibrateCommand);
    registerCommand("pid", "PID 값 설정 (pid <roll/pitch/yaw> <kp> <ki> <kd>)", pidCommand);
    registerCommand("motor", "모터 테스트 (motor <1-4> <1000-2000>)", motorCommand);
    registerCommand("save", "설정 저장", saveCommand);
    registerCommand("load", "설정 불러오기", loadCommand);
    registerCommand("reboot", "시스템 재시작", rebootCommand);
    
    println("");
    println("========================================");
    println("   Shane 드론 CLI v1.0");
    println("   'help'를 입력하여 명령 목록 확인");
    println("========================================");
    printPrompt();
    
    return true;
}

void SerialCLI::loop() {
    if (!stream) return;
    
    while (stream->available()) {
        char c = stream->read();
        
        // 에코
        if (echoEnabled) {
            stream->write(c);
        }
        
        // 엔터 처리
        if (c == '\r' || c == '\n') {
            if (bufferIndex > 0) {
                buffer[bufferIndex] = '\0';
                if (echoEnabled) {
                    stream->println();
                }
                processLine();
                bufferIndex = 0;
                memset(buffer, 0, CLI_BUFFER_SIZE);
                printPrompt();
            }
        }
        // 백스페이스 처리
        else if (c == '\b' || c == 127) {
            if (bufferIndex > 0) {
                bufferIndex--;
                buffer[bufferIndex] = '\0';
                if (echoEnabled) {
                    stream->print("\b \b");
                }
            }
        }
        // 일반 문자
        else if (c >= 32 && c < 127) {
            if (bufferIndex < CLI_BUFFER_SIZE - 1) {
                buffer[bufferIndex++] = c;
            }
        }
    }
}

void SerialCLI::processLine() {
    char* argv[CLI_MAX_ARGS];
    int argc = parseCommand(buffer, argv);
    
    if (argc > 0) {
        executeCommand(argc, argv);
    }
}

int SerialCLI::parseCommand(char* line, char** argv) {
    int argc = 0;
    char* token = strtok(line, " ");
    
    while (token != nullptr && argc < CLI_MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(nullptr, " ");
    }
    
    return argc;
}

void SerialCLI::executeCommand(int argc, char** argv) {
    // 명령 검색
    for (uint8_t i = 0; i < commandCount; i++) {
        if (strcmp(argv[0], commands[i].name) == 0) {
            if (commands[i].callback) {
                commands[i].callback(argc, argv);
            }
            return;
        }
    }
    
    // 명령을 찾을 수 없음
    print("알 수 없는 명령: ");
    println(argv[0]);
    println("'help'를 입력하여 사용 가능한 명령 확인");
}

bool SerialCLI::registerCommand(const char* name, const char* description, CliCommandCallback callback) {
    if (commandCount >= MAX_COMMANDS) return false;
    
    commands[commandCount].name = name;
    commands[commandCount].description = description;
    commands[commandCount].callback = callback;
    commandCount++;
    
    return true;
}

void SerialCLI::printPrompt() {
    stream->print("drone> ");
}

void SerialCLI::println(const String& message) {
    if (stream) {
        stream->println(message);
    }
}

void SerialCLI::print(const String& message) {
    if (stream) {
        stream->print(message);
    }
}

// Static 명령 핸들러들
void SerialCLI::helpCommand(int argc, char** argv) {
    if (!instance) return;
    
    instance->println("");
    instance->println("=== 사용 가능한 명령 ===");
    
    for (uint8_t i = 0; i < instance->commandCount; i++) {
        instance->print("  ");
        instance->print(instance->commands[i].name);
        instance->print(" - ");
        instance->println(instance->commands[i].description);
    }
    
    instance->println("");
}

void SerialCLI::statusCommand(int argc, char** argv) {
    if (!instance) return;
    
    instance->println("");
    instance->println("=== 시스템 상태 ===");
    
    if (instance->getStatusCallback) {
        instance->getStatusCallback();
    } else {
        instance->println("상태 콜백이 설정되지 않음");
    }
    
    instance->println("");
}

void SerialCLI::calibrateCommand(int argc, char** argv) {
    if (!instance) return;
    
    instance->println("");
    instance->println("센서 캘리브레이션 시작...");
    instance->println("드론을 평평한 곳에 놓고 움직이지 마세요.");
    
    if (instance->calibrateCallback) {
        instance->calibrateCallback();
        instance->println("캘리브레이션 완료!");
    } else {
        instance->println("캘리브레이션 콜백이 설정되지 않음");
    }
    
    instance->println("");
}

void SerialCLI::pidCommand(int argc, char** argv) {
    if (!instance) return;
    
    if (argc != 5) {
        instance->println("사용법: pid <roll/pitch/yaw> <kp> <ki> <kd>");
        instance->println("예: pid roll 1.2 0.8 0.15");
        return;
    }
    
    const char* axis = argv[1];
    float kp = atof(argv[2]);
    float ki = atof(argv[3]);
    float kd = atof(argv[4]);
    
    if (strcmp(axis, "roll") != 0 && 
        strcmp(axis, "pitch") != 0 && 
        strcmp(axis, "yaw") != 0) {
        instance->println("유효하지 않은 축: roll, pitch, yaw 중 선택");
        return;
    }
    
    if (instance->setPIDCallback) {
        instance->setPIDCallback(axis, kp, ki, kd);
        instance->print(axis);
        instance->print(" PID 설정: P=");
        instance->print(String(kp, 3));
        instance->print(" I=");
        instance->print(String(ki, 3));
        instance->print(" D=");
        instance->println(String(kd, 3));
    } else {
        instance->println("PID 콜백이 설정되지 않음");
    }
}

void SerialCLI::motorCommand(int argc, char** argv) {
    if (!instance) return;
    
    if (argc == 1) {
        instance->println("사용법: motor <1-4> <1000-2000>");
        instance->println("  motor 1 1200 - 모터 1을 1200으로 설정");
        instance->println("  motor all 1000 - 모든 모터를 1000으로 설정");
        instance->println("  motor stop - 모든 모터 정지");
        return;
    }
    
    if (strcmp(argv[1], "stop") == 0) {
        instance->println("모든 모터 정지");
        if (instance->motorTestCallback) {
            for (uint8_t i = 1; i <= 4; i++) {
                instance->motorTestCallback(i, 1000);
            }
        }
        return;
    }
    
    if (argc != 3) {
        instance->println("잘못된 인수 개수");
        return;
    }
    
    if (strcmp(argv[1], "all") == 0) {
        uint16_t value = atoi(argv[2]);
        if (value < 1000 || value > 2000) {
            instance->println("값은 1000-2000 사이여야 합니다");
            return;
        }
        
        instance->print("모든 모터를 ");
        instance->print(String(value));
        instance->println("으로 설정");
        
        if (instance->motorTestCallback) {
            for (uint8_t i = 1; i <= 4; i++) {
                instance->motorTestCallback(i, value);
            }
        }
    } else {
        uint8_t motor = atoi(argv[1]);
        uint16_t value = atoi(argv[2]);
        
        if (motor < 1 || motor > 4) {
            instance->println("모터 번호는 1-4 사이여야 합니다");
            return;
        }
        
        if (value < 1000 || value > 2000) {
            instance->println("값은 1000-2000 사이여야 합니다");
            return;
        }
        
        instance->print("모터 ");
        instance->print(String(motor));
        instance->print("를 ");
        instance->print(String(value));
        instance->println("으로 설정");
        
        if (instance->motorTestCallback) {
            instance->motorTestCallback(motor, value);
        }
    }
}

void SerialCLI::saveCommand(int argc, char** argv) {
    if (!instance) return;
    
    instance->println("설정 저장 중...");
    
    if (instance->saveConfigCallback) {
        instance->saveConfigCallback();
        instance->println("설정이 저장되었습니다");
    } else {
        instance->println("저장 콜백이 설정되지 않음");
    }
}

void SerialCLI::loadCommand(int argc, char** argv) {
    if (!instance) return;
    
    instance->println("설정 불러오는 중...");
    
    if (instance->loadConfigCallback) {
        instance->loadConfigCallback();
        instance->println("설정을 불러왔습니다");
    } else {
        instance->println("불러오기 콜백이 설정되지 않음");
    }
}

void SerialCLI::rebootCommand(int argc, char** argv) {
    if (!instance) return;
    
    instance->println("시스템을 재시작합니다...");
    delay(100);
    ESP.restart();
}