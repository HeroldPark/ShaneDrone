/*
 * I2C Scanner - Arduino Nano ESP32 독립 테스트
 *
 * 업로드: pio run -e i2c-scanner-jtag -t upload
 * 모니터: pio device monitor -p COM4 -b 115200
 *
 * 테스트 항목:
 *   1. Wire.begin() 기본 핀 (GPIO11=SDA, GPIO12=SCL)
 *   2. 전체 I2C 주소 스캔 (0x01 ~ 0x7F)
 *   3. MPU9250 WHO_AM_I 레지스터 확인
 *   4. 다중 클럭 속도 테스트 (100kHz / 400kHz)
 */

#include <Arduino.h>
#include <Wire.h>

// ─── 설정 ────────────────────────────────────────────
#define SCAN_INTERVAL_MS  3000   // 스캔 반복 간격
#define MPU9250_ADDR_LOW  0x68   // AD0=GND
#define MPU9250_ADDR_HIGH 0x69   // AD0=VCC
#define WHO_AM_I_REG      0x75
// ─────────────────────────────────────────────────────

static uint32_t scanCount = 0;

// ── 단일 주소 응답 확인 ──────────────────────────────
static bool probeAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// ── 레지스터 1바이트 읽기 ────────────────────────────
static bool readReg(uint8_t addr, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

// ── 장치명 반환 ──────────────────────────────────────
static const char* deviceName(uint8_t addr) {
  switch (addr) {
    case 0x68: return "MPU9250/MPU6050 (AD0=GND)";
    case 0x69: return "MPU9250/MPU6050 (AD0=VCC)";
    case 0x0C: return "AK8963 (자기계)";
    case 0x1E: return "HMC5883L";
    case 0x76:
    case 0x77: return "BMP280/BME280";
    case 0x3C:
    case 0x3D: return "OLED SSD1306";
    default:   return "알 수 없는 장치";
  }
}

// ── MPU9250 상세 진단 ────────────────────────────────
static void diagnoseMPU9250(uint8_t addr) {
  Serial.printf("\n  [MPU9250 상세 진단] 주소 0x%02X\n", addr);

  uint8_t whoami = 0;
  if (readReg(addr, WHO_AM_I_REG, whoami)) {
    Serial.printf("    WHO_AM_I (0x75) = 0x%02X  ", whoami);
    if (whoami == 0x71)      Serial.println("→ MPU9250 확인! ✓");
    else if (whoami == 0x73) Serial.println("→ MPU9255 확인! ✓");
    else if (whoami == 0x68) Serial.println("→ MPU6050 (MPU9250 아님)");
    else                     Serial.println("→ 알 수 없는 ID ✗");
  } else {
    Serial.println("    WHO_AM_I 읽기 실패 ✗");
  }

  // PWR_MGMT_1 (슬립 모드 확인)
  uint8_t pwr = 0;
  if (readReg(addr, 0x6B, pwr)) {
    Serial.printf("    PWR_MGMT_1 (0x6B) = 0x%02X  ", pwr);
    if (pwr & 0x40) Serial.println("→ 슬립 모드 (SLEEP=1) ⚠️");
    else            Serial.println("→ 정상 동작 ✓");
  }
}

// ── 클럭별 스캔 ──────────────────────────────────────
static int scanBus(uint32_t clock) {
  Wire.setClock(clock);
  delay(10);

  Serial.printf("\n  클럭 %lukHz 스캔:\n", clock / 1000);
  Serial.println("  ┌────────┬──────────────────────────────────┐");
  Serial.println("  │ 주소   │ 장치                             │");
  Serial.println("  ├────────┼──────────────────────────────────┤");

  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (probeAddress(addr)) {
      Serial.printf("  │ 0x%02X   │ %-32s │\n", addr, deviceName(addr));
      found++;
    }
  }

  if (found == 0) {
    Serial.println("  │  ---   │ 장치 없음                        │");
  }
  Serial.println("  └────────┴──────────────────────────────────┘");
  Serial.printf("  → 발견: %d개\n", found);
  return found;
}

// ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);  // USB CDC 안정화

  Serial.println("\n");
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║   I2C Scanner — Arduino Nano ESP32      ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println("  SDA : D11 (GPIO11)");
  Serial.println("  SCL : D12 (GPIO12)");
  Serial.println("  Wire.begin() — 인자 없음 (기본 핀 사용)");
}

void loop() {
  scanCount++;
  Serial.println();
  Serial.printf("━━━━━━━  스캔 #%lu  ━━━━━━━\n", scanCount);

  // Wire 초기화 (매 스캔마다 재초기화로 확실히)
  Wire.end();
  delay(10);
  bool ok = Wire.begin();
  Serial.printf("Wire.begin() : %s\n", ok ? "성공 ✓" : "실패 ✗");
  if (!ok) {
    Serial.println("I2C 하드웨어 초기화 실패 — 배선 확인 필요");
    delay(SCAN_INTERVAL_MS);
    return;
  }

  // 100kHz 스캔
  int found100 = scanBus(100000);

  // 400kHz 스캔
  int found400 = scanBus(400000);

  // MPU9250 상세 진단 (발견된 경우)
  Wire.setClock(100000);
  if (probeAddress(MPU9250_ADDR_LOW))  diagnoseMPU9250(MPU9250_ADDR_LOW);
  if (probeAddress(MPU9250_ADDR_HIGH)) diagnoseMPU9250(MPU9250_ADDR_HIGH);

  // 요약
  Serial.println();
  if (found100 == 0 && found400 == 0) {
    Serial.println("⚠️  I2C 장치 없음 — 하드웨어 체크리스트:");
    Serial.println("   [1] VCC → 3.3V,  GND → GND");
    Serial.println("   [2] SDA → D11,   SCL → D12");
    Serial.println("   [3] NCS → 3.3V   (SPI 모드 방지, 필수!)");
    Serial.println("   [4] AD0 → GND    (주소 0x68)");
    Serial.println("   [5] 풀업 저항 4.7kΩ SDA/SCL ↔ 3.3V");
  } else {
    Serial.println("✅ I2C 통신 정상");
  }

  delay(SCAN_INTERVAL_MS);
}
