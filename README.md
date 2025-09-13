# Pavo Pico Drone

## 2025-08-14
	- Arduino IDE => PlatformIO project로 변경
	- 더 다양한 기능 지원

## 2025-08-24
	- ESP32Dev 추가 - test code
	- PavoPicoDrone 전체적으로 수정해야함.

## 2025-08-25
	- Compile 성공
	- VSCode + PlatformIO + Arduino Nano ESP32 S3
	- DIY Pavo Pico Drone - Arduino Nano ESP32 + MPU9250
	- BETAFPV 1102 14000KV 모터 + Walksnail Avatar HD V2

## 2025-08-27
	- PavoPicoDrone => ShaneDrone으로 변경
	- 저장소 변경
## git remote set-url origin git@github.com:사용자명/저장소명.git

	- 3.5_Wokwi 오류 수정 : 사용 함수 미설정

## 2025-09-01
	- Telemetry를 Web 서버로 확대

### 5. 웹 대시보드 테스트
	이제 센서 오류와 관계없이 시스템이 부팅되므로:
	- 시리얼 모니터에서 WiFi AP 생성 확인
	- 스마트폰/PC에서 "Shane_Drone" WiFi에 연결
	- 브라우저에서 http://192.168.4.1 접속
	- 시뮬레이션 데이터로 대시보드 작동 확인

### 6. 2025-09-13
	- Arduino nano esp32 compile, upload, monitor
	- http://192.168.4.1 웹 서버 접속 완료