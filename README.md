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

###	Chrome vs Edge 문제
	- Chrome에서 chrome://net-internals/#dns 접속하여 DNS 캐시 클리어
	- 또는 Chrome에서 http://192.168.4.1 대신 http://192.168.4.1/ (마지막 슬래시 추가) 시도

### 7. 2025-09-14
	- communication.cpp에서 web part 분리 : web.cpp, web.h
	- web => async(Mobile), legacy(PC)로 분리
	- platformio.ini에서 사용하는 타겟에 따라 선택적 compile
	- serial CLI는 차단 : communicationLoop.handleSerialCLI() - 사용하기 불편하고 의미 없다.

	=> web_async_v0.1.cpp + drone_3d_viewer.html
	=> /src/web_async.cpp, /data/index.html, /data/css/style.css, /data/js/drone-controller.js
	=> 3D Drone Model을 적용 한  후 웹적속인 안되는 상황 발생.
	=> spiffs를 사용하여 웹 접속하는 방식으로 변경했다.
	=> build, upload, Serial Monitor 정상 수행
	=> Serial Monitor 정상 출력 상태이다.
	=> http://192.168.4.1 접속 안된다?
