# DIY Pavo Pico Drone - Arduino Nano ESP32

완전 커스텀 펌웨어를 사용한 DIY 마이크로 드론 프로젝트입니다.

## 🚁 하드웨어 사양

### 주요 구성품
- **프레임**: Pavo Pico Brushless Whoop (65mm)
- **메인보드**: Arduino Nano ESP32
- **센서**: MPU9250 (9DOF IMU)
- **ESC**: 4-in-1 ESC 20×20mm
- **모터**: BETAFPV 1102 14000KV (4개) - $20-25
- **배터리**: BETAFPV LAVA 550mAh 1S - $16-20
- **프로펠러**: GF 45mm 2블레이드
- **VTX**: Walksnail Avatar HD V2 20×20mm
- **ND 필터**: Walksnail Avatar 전용

### 핀 연결도

```
Arduino Nano ESP32 핀 배치:
┌─────────────────────┐
│  [USB]              │
│                     │
│  21(SDA)  22(SCL)   │ ← MPU9250 I2C
│  5   6   7   8      │ ← 모터 PWM (FL,FR,RL,RR)
│  4                  │ ← SBUS 수신기
│  17  16             │ ← VTX (TX,RX)
│  A0                 │ ← 배터리 전압
│  2                  │ ← 상태 LED
└─────────────────────┘
```

## 🛠️ 설치 및 설정

### 1. Arduino IDE 설정

```bash
# ESP32 보드 패키지 설치
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

### 2. 필요한 라이브러리

```cpp
#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFi.h>
```

### 3. 하드웨어 연결

#### MPU9250 센서 연결
```
MPU9250   Arduino Nano ESP32
VCC    →  3.3V
GND    →  GND
SDA    →  Pin 21
SCL    →  Pin 22
```

#### 4-in-1 ESC 연결
```
ESC     Arduino Pin
M1(FL)  →  Pin 5
M2(FR)  →  Pin 6  
M3(RL)  →  Pin 7
M4(RR)  →  Pin 8
```

#### SBUS 수신기 연결
```
RX Pin 4 (SBUS 신호 반전 필요)
```

#### 배터리 전압 측정
```
Battery(+) → 분압저항 → Pin A0
분압비: 3:1 (11.1V → 3.3V)
```

## 🔧 펌웨어 기능

### 센서 시스템
- **MPU9250**: 9축 IMU (가속도, 자이로, 자기계)
- **상보 필터**: 자세 융합 알고리즘
- **자동 캘리브레이션**: 시작시 센서 영점 조정
- **센서 헬스 체크**: 실시간 센서 상태 모니터링

### 제어 시스템
- **PID 제어**: 독립적인 Roll/Pitch/Yaw PID
- **비행 모드**:
  - STABILIZE: 자세 안정화
  - ACRO: 각속도 제어
  - ALTITUDE_HOLD: 고도 유지 (추후 구현)
- **모터 믹싱**: X 형태 쿼드콥터 배치
- **Anti-windup**: PID 적분항 포화 방지

### 통신 시스템
- **SBUS**: 16채널 수신기 프로토콜
- **WiFi 텔레메트리**: 실시간 데이터 전송
- **VTX 연동**: Walksnail Avatar HD V2 OSD
- **CLI**: 시리얼 명령줄 인터페이스

### 안전 기능
- **ARM/DISARM**: 안전한 시동/정지 시퀀스
- **Failsafe**: 통신 두절시 자동 착륙
- **배터리 모니터링**: 저전압 경고 및 보호
- **센서 오류 감지**: 자동 DISARM

## 🎛️ 초기 설정

### 1. 센서 캘리브레이션

```cpp
// 드론을 평평한 곳에 놓고 5초간 대기
// 자동으로 자이로/가속도 오프셋 계산
```

### 2. PID 튜닝

BETAFPV 1102 14000KV 모터용 기본값:

```cpp
// Roll/Pitch PID
P = 1.2
I = 0.8  
D = 0.15

// Yaw PID
P = 1.5
I = 0.5
D = 0.05
```

### 3. 조종기 설정

```
채널 매핑:
CH1: Throttle
CH2: Roll
CH3: Pitch  
CH4: Yaw
CH5: ARM/DISARM 스위치
CH6: 비행모드 스위치
```

## 📡 무선 기능

### WiFi 텔레메트리

```
SSID: PavoPico_Drone
Password: drone123
IP: 192.168.4.1:8888
```

실시간 데이터:
- 자세 정보 (Roll/Pitch/Yaw)
- 자이로 데이터
- 조종기 입력
- 배터리 전압
- 비행 모드

### PID 원격 튜닝

WiFi를 통한 실시간 PID 조정:

```
PID_ROLL,1.2,0.8,0.15
PID_PITCH,1.2,0.8,0.15  
PID_YAW,1.5,0.5,0.05
```

## 🔍 디버깅

### 시리얼 모니터 명령

```
help       - 명령어 도움말
status     - 시스템 상태
calibrate  - 센서 캘리브레이션
pid_roll <p> <i> <d>  - Roll PID 설정
reset_pid  - PID 적분항 리셋
sbus_debug - SBUS 채널 상태
```

### LED 상태 표시

```
깜빡임: DISARM 상태
켜짐: ARM 상태  
꺼짐: 시스템 오류
```

## ⚠️ 안전 주의사항

### ARM 조건
- 스로틀 최소값
- 모든 스틱 중앙 위치
- 배터리 전압 정상
- 센서 정상 동작
- ARM 스위치 ON

### 비상 상황 대응
- **통신 두절**: 10초 후 자동 착륙
- **저전압**: 경고 후 자동 DISARM
- **센서 오류**: 즉시 DISARM
- **수동 DISARM**: ARM 스위치 OFF

## 🚀 비행 전 체크리스트

1. ✅ 하드웨어 연결 확인
2. ✅ 센서 캘리브레이션 완료
3. ✅ 조종기 바인딩 확인
4. ✅ 배터리 전압 체크 (>3.5V)
5. ✅ 프로펠러 장착 및 고정
6. ✅ 안전 지역 확보
7. ✅ ARM 조건 만족 확인

## 📈 성능 사양

- **제어 주파수**: 1000Hz
- **센서 업데이트**: 1000Hz  
- **SBUS 수신**: 500Hz
- **텔레메트리**: 50Hz
- **최대 기울기**: ±30°
- **최대 회전속도**: 200°/s

## 🔧 고급 기능

### 확장 가능한 기능
- GPS 모듈 추가 (위치 유지)
- 바로미터 추가 (고도 유지)
- 광류 센서 (실내 위치 유지)
- FPV 카메라 제어
- 블랙박스 로깅

### 커스터마이징
- PID 게인 실시간 조정
- 비행 모드 추가
- 스마트 기능 구현
- 자동 착륙 시퀀스

## 📋 트러블슈팅

### 자주 발생하는 문제

**센서 초기화 실패**
```
- I2C 연결 확인
- 3.3V 전원 확인  
- MPU9250 주소 확인 (0x68)
```

**ARM이 안됨**
```
- 조종기 바인딩 확인
- 스틱 중앙 위치 확인
- 배터리 전압 확인
- 센서 캘리브레이션 확인
```

**진동이 심함**
```
- 프로펠러 밸런싱
- 모터 마운트 확인
- PID D 게인 조정
- 프레임 강성 확인
```

## 🎯 개발 로드맵

### v1.1 (예정)
- [ ] GPS 모듈 지원
- [ ] 바로미터 고도 센서
- [ ] 자동 이착륙 모드
- [ ] 블랙박스 로깅

### v1.2 (예정)  
- [ ] 광류 센서 지원
- [ ] 실내 위치 유지
- [ ] 웨이포인트 비행
- [ ] 스마트폰 앱 연동

## 📜 라이센스

MIT License - 자유롭게 사용, 수정, 배포 가능

## 🤝 기여하기

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## 📞 지원 및 커뮤니티

### 개발자 연락처
- **GitHub**: [Your GitHub Profile]
- **Email**: [your-email@example.com]
- **Discord**: DIY 드론 커뮤니티

### 유용한 링크
- [Arduino ESP32 공식 문서](https://docs.espressif.com/projects/arduino-esp32/)
- [MPU9250 데이터시트](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [BETAFPV 1102 모터 스펙](https://betafpv.com/)
- [Walksnail Avatar HD V2 매뉴얼](https://www.walksnail.com/)

---

**⚠️ 주의: 이 프로젝트는 실험적 성격의 DIY 드론입니다. 비행 전 반드시 지역 법규를 확인하고, 안전한 환경에서만 테스트하시기 바랍니다.**

**🎯 Happy Flying! 안전한 비행 되세요!** 🚁