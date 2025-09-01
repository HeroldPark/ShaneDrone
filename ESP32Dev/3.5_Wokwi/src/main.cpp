/*
 * ESP32 드론 컨트롤러 v2.2
 * GPIO 핀 수정 및 모터 테스트 기능 포함
 */

#include <Arduino.h>

// ESP32에서 안전하게 사용 가능한 GPIO 핀
#define LED_BUILTIN 2      // 내장 LED (안전)
#define STATUS_LED_PIN 2   // 상태 LED (내장 LED 공유)

// 외부 LED 핀 정의
const int LED_PIN = 13;

// L293D 모터 드라이버 핀 설정
// 각 모터는 2개의 제어 핀 필요 (방향 + PWM)
// 모터 1 (FL)
#define MOTOR_FL_PIN1 25    // IN1 (방향)
#define MOTOR_FL_PIN2 26    // IN2 (방향)
#define MOTOR_FL_ENA 14     // ENA (PWM 속도 제어)

// 모터 2 (FR) 
#define MOTOR_FR_PIN1 27    // IN3 (방향)
#define MOTOR_FR_PIN2 33    // IN4 (방향)
#define MOTOR_FR_ENB 32     // ENB (PWM 속도 제어)

// 추가 모터를 위한 예비 (필요시 두 번째 L293D 사용)
#define MOTOR_RL_PIN1 15    // 후면 좌측 (옵션)
#define MOTOR_RL_PIN2 4     // 후면 좌측 (옵션)
#define MOTOR_RR_PIN1 16    // 후면 우측 (옵션)
#define MOTOR_RR_PIN2 17    // 후면 우측 (옵션)

// PWM 설정
#define PWM_FREQUENCY 1000  // L293D용 낮은 주파수 (1kHz)
#define PWM_RESOLUTION 8    // PWM 해상도 (8-bit = 0-255)
// PWM 채널 할당
#define PWM_CHANNEL_FL 0    // FL 모터 속도
#define PWM_CHANNEL_FR 1    // FR 모터 속도
#define PWM_CHANNEL_RL 2    // RL 모터 속도 (옵션)
#define PWM_CHANNEL_RR 3    // RR 모터 속도 (옵션)

// 드론 설정
#define FRAME_SIZE 65
#define MOTOR_KV 14000
#define BATTERY_CELLS 1

// 디버그 모드
#define DEBUG_MODE true

// 모터 테스트 모드
enum MotorTestMode {
  TEST_OFF = 0,
  TEST_INDIVIDUAL = 1,
  TEST_ALL = 2,
  TEST_SEQUENCE = 3,
  TEST_MANUAL = 4
};

// 구조체 정의
struct SensorData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float roll, pitch, yaw;
  bool sensorHealthy;
  unsigned long timestamp;
};

struct DroneState {
  bool armed;
  bool calibrated;
  uint8_t flightMode;
  float batteryVoltage;
  uint16_t flightTime;
};

struct MotorState {
  int speedFL;
  int speedFR;
  int speedRL;
  int speedRR;
  MotorTestMode testMode;
  int currentTestMotor;
  unsigned long testStartTime;
};

// 전역 변수
SensorData sensorData;
DroneState droneState;
MotorState motorState;

// 타이밍 변수
unsigned long previousLedMillis = 0;
unsigned long previousStatusMillis = 0;
unsigned long previousSensorMillis = 0;
unsigned long previousMotorTestMillis = 0;
unsigned long loopStartTime = 0;
bool ledState = false;
int loopCounter = 0;

// 함수 선언
void initializeSystem();
void initializeData();
void initializePWM();
void updateLED();
void updateSensors();
void printStatus();
void checkSystem();
void handleSerialCommands();
void printHelp();

// 모터 제어 함수
void setMotorSpeed(int motor, int speed);
void setAllMotors(int speed);
void stopAllMotors();
void testIndividualMotor(int motorNum);
void testAllMotors();
void runMotorTestSequence();
void printMotorStatus();

void setup() {
  // 시리얼 통신 초기화 (가장 먼저)
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);  // Serial 연결 대기 (최대 3초)
  }
  
  delay(500);  // 안정화 대기
  
  Serial.println("\n\n========================================");
  Serial.println("    ESP32 드론 컨트롤러 v2.2");
  Serial.println("    모터 테스트 기능 포함");
  Serial.println("========================================\n");
  
  // 시스템 초기화
  initializeSystem();
  
  // 도움말 출력
  printHelp();
  
  Serial.println("\n>>> 메인 루프 시작 <<<\n");
  loopStartTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // 시리얼 명령 처리
  handleSerialCommands();
  
  // LED 업데이트 (500ms)
  if (currentMillis - previousLedMillis >= 500) {
    previousLedMillis = currentMillis;
    updateLED();
  }
  
  // 센서 업데이트 (100ms)
  if (currentMillis - previousSensorMillis >= 100) {
    previousSensorMillis = currentMillis;
    updateSensors();
  }
  
  // 상태 출력 (1초)
  if (currentMillis - previousStatusMillis >= 1000) {
    previousStatusMillis = currentMillis;
    printStatus();
  }
  
  // 모터 테스트 실행 (테스트 모드일 때)
  if (motorState.testMode != TEST_OFF) {
    if (currentMillis - previousMotorTestMillis >= 100) {
      previousMotorTestMillis = currentMillis;
      
      switch(motorState.testMode) {
        case TEST_SEQUENCE:
          runMotorTestSequence();
          break;
        case TEST_ALL:
          testAllMotors();
          break;
        case TEST_INDIVIDUAL:
          // 개별 모터 테스트는 명령으로 제어
          break;
        default:
          break;
      }
    }
  }
  
  // 시스템 체크
  checkSystem();
  
  // CPU 부하 감소
  delay(1);
  loopCounter++;
}

void initializeSystem() {
  Serial.println("[INIT] 시스템 초기화 시작");
  
  // GPIO 핀 안전 체크
  Serial.println("[GPIO] 핀 설정 시작");
  
  // LED 핀 설정
  Serial.print("  - LED 핀 (GPIO");
  Serial.print(LED_BUILTIN);
  Serial.print(") 설정...");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(" OK");

  // 외부 LED 핀 설정
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // PWM 초기화
  Serial.println("[PWM] PWM 채널 초기화");
  initializePWM();
  
  // 데이터 구조체 초기화
  Serial.print("[DATA] 구조체 초기화...");
  initializeData();
  Serial.println(" OK");
  
  // LED 테스트
  Serial.print("[TEST] LED 테스트");
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    Serial.print(".");
  }
  Serial.println(" OK");
  
  // 시스템 정보 출력
  Serial.println("\n[INFO] 시스템 정보:");
  Serial.print("  - Chip Model: ");
  Serial.println(ESP.getChipModel());
  Serial.print("  - CPU Freq: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(" MHz");
  Serial.print("  - Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("  - Flash Size: ");
  Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
  Serial.println(" MB");
  
  Serial.println("\n[INIT] 초기화 완료!");
}

void initializePWM() {
  Serial.println("[L293D] 모터 드라이버 초기화");
  
  // 방향 제어 핀 설정
  pinMode(MOTOR_FL_PIN1, OUTPUT);
  pinMode(MOTOR_FL_PIN2, OUTPUT);
  pinMode(MOTOR_FR_PIN1, OUTPUT);
  pinMode(MOTOR_FR_PIN2, OUTPUT);
  
  // Enable 핀 (PWM) 설정
  pinMode(MOTOR_FL_ENA, OUTPUT);
  pinMode(MOTOR_FR_ENB, OUTPUT);
  
  // PWM 채널 설정 (Enable 핀용)
  ledcSetup(PWM_CHANNEL_FL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_FR, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // PWM 채널을 Enable 핀에 연결
  ledcAttachPin(MOTOR_FL_ENA, PWM_CHANNEL_FL);
  ledcAttachPin(MOTOR_FR_ENB, PWM_CHANNEL_FR);
  
  Serial.println("  - 모터1 (FL): IN1=" + String(MOTOR_FL_PIN1) + 
                 ", IN2=" + String(MOTOR_FL_PIN2) + 
                 ", ENA=" + String(MOTOR_FL_ENA));
  Serial.println("  - 모터2 (FR): IN3=" + String(MOTOR_FR_PIN1) + 
                 ", IN4=" + String(MOTOR_FR_PIN2) + 
                 ", ENB=" + String(MOTOR_FR_ENB));
  
  // 모든 모터 정지 (브레이크 모드)
  stopAllMotors();
  Serial.println("  - 초기 상태: 모든 모터 브레이크");
}

void initializeData() {
  // SensorData 초기화
  memset(&sensorData, 0, sizeof(SensorData));
  sensorData.accelZ = 9.81;
  sensorData.sensorHealthy = true;
  
  // DroneState 초기화
  memset(&droneState, 0, sizeof(DroneState));
  droneState.calibrated = true;
  droneState.batteryVoltage = 4.1;
  
  // MotorState 초기화
  memset(&motorState, 0, sizeof(MotorState));
  motorState.testMode = TEST_OFF;
}

// L293D 모터 제어 함수
void setMotorL293D(int motor, int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  
  if (motor == 1) { // FL 모터
    motorState.speedFL = speed;
    if (speed == 0) {
      // 브레이크 모드 (두 핀 모두 LOW)
      digitalWrite(MOTOR_FL_PIN1, LOW);
      digitalWrite(MOTOR_FL_PIN2, LOW);
      ledcWrite(PWM_CHANNEL_FL, 0);
      Serial.println("[MOTOR FL] 브레이크 (정지)");
    } else {
      // 방향 설정
      digitalWrite(MOTOR_FL_PIN1, forward ? HIGH : LOW);
      digitalWrite(MOTOR_FL_PIN2, forward ? LOW : HIGH);
      // 속도 설정
      ledcWrite(PWM_CHANNEL_FL, speed);
      Serial.print("[MOTOR FL] ");
      Serial.print(forward ? "정방향" : "역방향");
      Serial.print(" 속도: ");
      Serial.println(speed);
    }
  } 
  else if (motor == 2) { // FR 모터
    motorState.speedFR = speed;
    if (speed == 0) {
      // 브레이크 모드
      digitalWrite(MOTOR_FR_PIN1, LOW);
      digitalWrite(MOTOR_FR_PIN2, LOW);
      ledcWrite(PWM_CHANNEL_FR, 0);
      Serial.println("[MOTOR FR] 브레이크 (정지)");
    } else {
      // 방향 설정
      digitalWrite(MOTOR_FR_PIN1, forward ? HIGH : LOW);
      digitalWrite(MOTOR_FR_PIN2, forward ? LOW : HIGH);
      // 속도 설정
      ledcWrite(PWM_CHANNEL_FR, speed);
      Serial.print("[MOTOR FR] ");
      Serial.print(forward ? "정방향" : "역방향");
      Serial.print(" 속도: ");
      Serial.println(speed);
    }
  }
}

void setMotorSpeed(int motor, int speed) {
  // L293D용 수정: 기본 정방향
  setMotorL293D(motor, speed, true);
}

void stopAllMotors() {
  // L293D 브레이크 모드 - 모든 핀을 LOW로 설정
  // 모터 1 (FL) 완전 정지
  digitalWrite(MOTOR_FL_PIN1, LOW);
  digitalWrite(MOTOR_FL_PIN2, LOW);
  ledcWrite(PWM_CHANNEL_FL, 0);
  
  // 모터 2 (FR) 완전 정지
  digitalWrite(MOTOR_FR_PIN1, LOW);
  digitalWrite(MOTOR_FR_PIN2, LOW);
  ledcWrite(PWM_CHANNEL_FR, 0);
  
  // 상태 초기화
  motorState.speedFL = 0;
  motorState.speedFR = 0;
  motorState.speedRL = 0;
  motorState.speedRR = 0;
  motorState.testMode = TEST_OFF;
  
  Serial.println("[L293D] 모든 모터 브레이크 - 완전 정지!");
  delay(100); // 정지 확실히 하기 위한 짧은 지연
}

void setAllMotors(int speed) {
  speed = constrain(speed, 0, 255);
  
  // 두 모터 모두 정방향으로 설정
  setMotorL293D(1, speed, true);  // FL
  setMotorL293D(2, speed, true);  // FR
  
  // 4개 모터 사용 시 (두 번째 L293D 연결 시)
  // setMotorL293D(3, speed, true);  // RL
  // setMotorL293D(4, speed, true);  // RR
}

void testIndividualMotor(int motorNum) {
  stopAllMotors();
  delay(100);
  
  String motorName;
  switch(motorNum) {
    case 1: motorName = "FL (전면 좌측)"; break;
    case 2: motorName = "FR (전면 우측)"; break;
    case 3: motorName = "RL (후면 좌측)"; break;
    case 4: motorName = "RR (후면 우측)"; break;
    default: return;
  }
  
  Serial.println("\n[TEST] " + motorName + " 모터 테스트");
  
  // 점진적 가속
  for(int speed = 0; speed <= 100; speed += 10) {
    setMotorSpeed(motorNum, speed);
    Serial.print("  속도: " + String(speed) + " ");
    Serial.println("(" + String((speed * 100) / 255) + "%)");
    delay(200);
  }
  
  // 점진적 감속
  for(int speed = 100; speed >= 0; speed -= 10) {
    setMotorSpeed(motorNum, speed);
    delay(200);
  }
  
  Serial.println("  테스트 완료!");
}

void testAllMotors() {
  static int testSpeed = 0;
  static bool increasing = true;
  
  if (increasing) {
    testSpeed += 5;
    if (testSpeed >= 100) {
      testSpeed = 100;
      increasing = false;
    }
  } else {
    testSpeed -= 5;
    if (testSpeed <= 0) {
      testSpeed = 0;
      increasing = true;
      motorState.testMode = TEST_OFF;
      Serial.println("[TEST] 전체 모터 테스트 완료");
    }
  }
  
  setAllMotors(testSpeed);
}

void runMotorTestSequence() {
  static int sequenceStep = 0;
  static unsigned long stepStartTime = 0;
  unsigned long currentTime = millis();
  
  if (stepStartTime == 0) {
    stepStartTime = currentTime;
  }
  
  if (currentTime - stepStartTime >= 2000) { // 2초마다 다음 단계
    stepStartTime = currentTime;
    
    stopAllMotors();
    delay(100);
    
    switch(sequenceStep) {
      case 0:
        Serial.println("\n[SEQ] 1. FL 모터 테스트");
        setMotorSpeed(1, 80);
        break;
      case 1:
        Serial.println("[SEQ] 2. FR 모터 테스트");
        setMotorSpeed(2, 80);
        break;
      case 2:
        Serial.println("[SEQ] 3. RL 모터 테스트");
        setMotorSpeed(3, 80);
        break;
      case 3:
        Serial.println("[SEQ] 4. RR 모터 테스트");
        setMotorSpeed(4, 80);
        break;
      case 4:
        Serial.println("[SEQ] 5. 대각선 테스트 (FL+RR)");
        setMotorSpeed(1, 80);
        setMotorSpeed(4, 80);
        break;
      case 5:
        Serial.println("[SEQ] 6. 대각선 테스트 (FR+RL)");
        setMotorSpeed(2, 80);
        setMotorSpeed(3, 80);
        break;
      case 6:
        Serial.println("[SEQ] 7. 전체 모터 저속");
        setAllMotors(50);
        break;
      case 7:
        Serial.println("[SEQ] 8. 전체 모터 중속");
        setAllMotors(100);
        break;
      case 8:
        Serial.println("[SEQ] 시퀀스 완료!");
        stopAllMotors();
        motorState.testMode = TEST_OFF;
        sequenceStep = -1;
        stepStartTime = 0;
        break;
    }
    sequenceStep++;
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() == 0) return;
    
    char command = cmd.charAt(0);
    
    switch(command) {
      // 모터 개별 테스트
      case '1':
        motorState.testMode = TEST_INDIVIDUAL;
        testIndividualMotor(1);
        break;
      case '2':
        motorState.testMode = TEST_INDIVIDUAL;
        testIndividualMotor(2);
        break;
      case '3':
        motorState.testMode = TEST_INDIVIDUAL;
        testIndividualMotor(3);
        break;
      case '4':
        motorState.testMode = TEST_INDIVIDUAL;
        testIndividualMotor(4);
        break;
      
      // 모터 제어
      case 'a': // 전체 모터 테스트
        motorState.testMode = TEST_ALL;
        Serial.println("[TEST] 전체 모터 테스트 시작");
        break;
      
      case 's': // 시퀀스 테스트
        motorState.testMode = TEST_SEQUENCE;
        Serial.println("[TEST] 시퀀스 테스트 시작");
        break;
      
      case 'r': // 역방향 테스트
        Serial.println("[TEST] 역방향 테스트");
        setMotorL293D(1, 100, false);  // FL 역방향
        setMotorL293D(2, 100, false);  // FR 역방향
        break;
      
      case 'f': // 정방향 테스트
        Serial.println("[TEST] 정방향 테스트");
        setMotorL293D(1, 100, true);   // FL 정방향
        setMotorL293D(2, 100, true);   // FR 정방향
        break;
      
      case 'q': // 정지
        stopAllMotors();
        break;
      
      // 수동 속도 설정
      case '+': // 속도 증가
        {
          int newSpeed = motorState.speedFL + 10;
          setAllMotors(newSpeed);
          Serial.println("[MOTOR] 속도 증가: " + String(newSpeed));
        }
        break;
      
      case '-': // 속도 감소
        {
          int newSpeed = motorState.speedFL - 10;
          setAllMotors(newSpeed);
          Serial.println("[MOTOR] 속도 감소: " + String(newSpeed));
        }
        break;
      
      case 'm': // 모터 상태
        printMotorStatus();
        break;
      
      case 'h': // 도움말
        printHelp();
        break;
      
      case 'p': // 시스템 상태
        printStatus();
        break;
      
      // 숫자 입력 시 PWM 값 직접 설정
      default:
        if (isdigit(command)) {
          int value = cmd.toInt();
          if (value >= 0 && value <= 255) {
            setAllMotors(value);
            Serial.println("[MOTOR] PWM 설정: " + String(value));
          }
        }
        break;
    }
  }
}

void printMotorStatus() {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║        모터 상태 정보              ║");
  Serial.println("╠════════════════════════════════════╣");
  Serial.print("║ FL (GPIO"); Serial.print(MOTOR_FL_PIN1); 
  Serial.print("): "); Serial.print(motorState.speedFL);
  Serial.print(" ("); Serial.print((motorState.speedFL * 100) / 255); Serial.println("%)     ║");
  
  Serial.print("║ FR (GPIO"); Serial.print(MOTOR_FR_PIN2); 
  Serial.print("): "); Serial.print(motorState.speedFR);
  Serial.print(" ("); Serial.print((motorState.speedFR * 100) / 255); Serial.println("%)     ║");
  
  Serial.print("║ RL (GPIO"); Serial.print(MOTOR_RL_PIN1); 
  Serial.print("): "); Serial.print(motorState.speedRL);
  Serial.print(" ("); Serial.print((motorState.speedRL * 100) / 255); Serial.println("%)     ║");
  
  Serial.print("║ RR (GPIO"); Serial.print(MOTOR_RR_PIN2); 
  Serial.print("): "); Serial.print(motorState.speedRR);
  Serial.print(" ("); Serial.print((motorState.speedRR * 100) / 255); Serial.println("%)     ║");
  
  Serial.print("║ 테스트 모드: ");
  switch(motorState.testMode) {
    case TEST_OFF: Serial.println("OFF         ║"); break;
    case TEST_INDIVIDUAL: Serial.println("개별        ║"); break;
    case TEST_ALL: Serial.println("전체        ║"); break;
    case TEST_SEQUENCE: Serial.println("시퀀스      ║"); break;
    case TEST_MANUAL: Serial.println("수동        ║"); break;
  }
  Serial.println("╚════════════════════════════════════╝");
}

void printHelp() {
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║          명령어 도움말                ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.println("║ === 모터 개별 테스트 ===              ║");
  Serial.println("║ 1 : 모터1 (FL) 테스트                 ║");
  Serial.println("║ 2 : 모터2 (FR) 테스트                 ║");
  Serial.println("║                                       ║");
  Serial.println("║ === 모터 그룹 테스트 ===              ║");
  Serial.println("║ a : 전체 모터 동시 테스트             ║");
  Serial.println("║ s : 시퀀스 테스트 (순차적)            ║");
  Serial.println("║                                       ║");
  Serial.println("║ === L293D 모터 제어 ===               ║");
  Serial.println("║ q : 모든 모터 정지 (브레이크)         ║");
  Serial.println("║ f : 정방향 회전                       ║");
  Serial.println("║ r : 역방향 회전                       ║");
  Serial.println("║ + : 전체 속도 10 증가                 ║");
  Serial.println("║ - : 전체 속도 10 감소                 ║");
  Serial.println("║ 0-255 : PWM 값 직접 설정              ║");
  Serial.println("║                                       ║");
  Serial.println("║ === 정보 표시 ===                     ║");
  Serial.println("║ m : 모터 상태 표시                    ║");
  Serial.println("║ p : 시스템 상태 표시                  ║");
  Serial.println("║ h : 이 도움말 표시                    ║");
  Serial.println("╚═══════════════════════════════════════╝");
}

void updateLED() {
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);
  digitalWrite(LED_PIN, ledState);
  
  // 모터 테스트 중일 때는 LED 빠르게 깜빡임
  if (motorState.testMode != TEST_OFF) {
    digitalWrite(LED_BUILTIN, !ledState);
  }
}

void updateSensors() {
  float time = millis() / 1000.0;
  
  // 간단한 센서 시뮬레이션
  sensorData.roll = sin(time * 0.5) * 10.0;
  sensorData.pitch = cos(time * 0.3) * 8.0;
  sensorData.yaw += 0.1;
  
  if (sensorData.yaw > 180.0) sensorData.yaw -= 360.0;
  
  sensorData.timestamp = micros();
}

void printStatus() {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║      드론 상태 모니터              ║");
  Serial.println("╠════════════════════════════════════╣");
  
  // 자세 정보
  Serial.print("║ 자세: R=");
  Serial.print(sensorData.roll, 1);
  Serial.print("° P=");
  Serial.print(sensorData.pitch, 1);
  Serial.print("° Y=");
  Serial.print(sensorData.yaw, 1);
  Serial.println("°        ║");
  
  // 시스템 상태
  Serial.print("║ 배터리: ");
  Serial.print(droneState.batteryVoltage);
  Serial.println(" V                 ║");
  
  Serial.print("║ 비행시간: ");
  Serial.print(droneState.flightTime++);
  Serial.println(" 초               ║");
  
  Serial.print("║ 메모리: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes        ║");
  
  Serial.print("║ 루프: ");
  Serial.print(loopCounter);
  Serial.print(" (");
  Serial.print((millis() - loopStartTime) / 1000);
  Serial.println("초)           ║");
  
  // 모터 상태 간단 표시
  if (motorState.testMode != TEST_OFF) {
    Serial.println("║ 모터: 테스트 중 ⚡                ║");
  } else {
    Serial.println("║ 모터: 대기 중 ⭕                  ║");
  }
  
  Serial.println("╚════════════════════════════════════╝");
}

void checkSystem() {
  // Watchdog 리셋
  yield();
  
  // 스택 오버플로우 체크
  if (ESP.getFreeHeap() < 10000) {
    Serial.println("[WARNING] Low memory!");
  }
  
  // 온도 체크 (ESP32 내부 온도 센서)
  #ifdef CONFIG_IDF_TARGET_ESP32
  float temp = temperatureRead();
  if (temp > 70.0) {
    Serial.print("[WARNING] High temperature: ");
    Serial.println(temp);
    delay(1000);
  }
  #endif
}

/*
 * === L293D 모터 드라이버 연결 가이드 ===
 * 
 * L293D IC 핀 연결:
 * --------------------------------
 * | L293D 핀  | ESP32 연결      |
 * --------------------------------
 * | Enable1   | GPIO 14 (PWM)   |
 * | Input1    | GPIO 25         |
 * | Input2    | GPIO 26         |
 * | Output1   | 모터1 (+)       |
 * | Output2   | 모터1 (-)       |
 * | GND       | GND (공통)      |
 * | VCC2      | 모터 전원 (6-12V)|
 * | Input3    | GPIO 27         |
 * | Input4    | GPIO 33         |
 * | Output3   | 모터2 (+)       |
 * | Output4   | 모터2 (-)       |
 * | Enable2   | GPIO 32 (PWM)   |
 * | VCC1      | 5V (로직 전원)  |
 * --------------------------------
 * 
 * 모터 제어 로직:
 * - IN1=HIGH, IN2=LOW: 정방향
 * - IN1=LOW, IN2=HIGH: 역방향
 * - IN1=LOW, IN2=LOW: 브레이크 (정지)
 * - IN1=HIGH, IN2=HIGH: 프리런 (관성)
 * - Enable=PWM: 속도 제어
 */