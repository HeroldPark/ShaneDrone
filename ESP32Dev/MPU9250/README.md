# 부트로더(다운로드) 모드에서 바로 업로드 (CDC 우회)

1. 보드만 단독 연결(센서/점퍼 전부 해제).
2. B1–GND 잠깐 쇼트 → RST 버튼 (LED 보라색 고정이면 다운로드 모드).
3. 쇼트 해제 후 포트 확인:

```
pio device list

COM9
----
Hardware ID: USB VID:PID=303A:1001 SER=74:4D:BD:A1:B5:E4 LOCATION=1-1:x.0
Description: USB 직렬 장치(COM9)
```
(이전에 erase가 된 포트가 COM9이었습니다. 이번에도 비슷하게 잡힐 가능성 큼. 이 상태에서는 COM10 보이지 않는다. 이때 COM10은 보이지 않는다.)

```
pio run -t erase

정상 수행된다.
```

4. 바로 그 부트로더 포트로 업로드:
   1) COM9로 업로드(부트로더/다운로드 모드)할 때의 LED 상태
    - 전원 LED(초록 ‘ON’): 항상 켜짐.
    - RGB 상태 LED(보드 상단): 고정된 보라/자홍색(보드 리비전마다 약간 다를 수 있음).
↳   - 이게 보이면 **펌웨어 다운로드 모드(USB-Serial/JTAG)**로 진입된 상태예요.
    - 노란 ‘L’(D13) LED: 상태는 상관없음(꺼져 있거나 깜빡여도 무방).

   2) 그리고 LED보다 더 확실한 지표는 이 두 가지예요:
    - pio device list 실행 시, 새 포트(이전에 보였던 COM9)가 잡히고
    - 업로드/erase 로그에 USB mode: USB-Serial/JTAG 같은 문구가 뜨는 것.

   3) 올바른 순서(재확인)
    - B1 ↔ GND 잠깐 쇼트 → RST 누르기 → RGB가 보라 고정되면 다운로드 모드 진입.
    - 쇼트 제거(그대로 보라 유지)
    - pio device list로 COM9 확인.
    - 그 다음 바로 upload 시작

```
pio run -t upload -v --upload-port COM9
```
    - platformio.ini에 upload_port=COM10이 있어도, 위 CLI 인자가 우선합니다.
    - 가능하면 platformio.ini에서 upload_port=COM10 주석 처리
    - 속도는 안정적으로: upload_speed = 115200

5. 업로드 완료 후 RST 눌러 정상 부팅 확인. 그 다음에야 CDC(COM10 등 : Communications Device Class : USB를 통한 시리얼 통신)로 시리얼이 살아납니다.
    - USB 포트 뺐다. 다시 꼿으면 된다.
    - platformio 아래 시리얼 포트는 Auto 로 셋팅하면 됨
```
pio device list

COM10
-----
Hardware ID: USB VID:PID=2341:0070 SER=744DBDA1B5E4 LOCATION=1-1:x.1
Description: USB 직렬 장치(COM10)
```

6. 이 상태에서 시리얼 모니터링
```
pio device monitor

--- Terminal on COM10 | 115200 8-N-1
--- Available filters and text transformations: colorize, debug, default, direct, esp32_exception_decoder, hexlify, log2file, nocontrol, printable, send_on_enter, 
time
--- More details at https://bit.ly/pio-monitor-filters
--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H
❤️ 보드 정상 동작 중... (LED ON)
💤 LED OF
```