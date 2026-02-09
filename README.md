#include <WiFi.h>
#include "esp_wifi.h"

/* ===================== 설정 ===================== */

// 초기 채널 (로터리 없는 개발 단계에서 사용)
#define DEFAULT_SNIFF_CHANNEL      6

// 1초 요약 출력
#define REPORT_INTERVAL_MS         1000

// === Payload 유효 범위 (네가 준 값 반영) ===
#define CMD_MIN                    0x01
#define CMD_MAX                    0x06
#define DEVTYPE_MIN                0x01
#define DEVTYPE_MAX                0x0A

// GROUP/CHANNEL/ADDR는 일단 넉넉히 (네 말대로 max 기준 OK)
#define MAX_GROUP                  0xFF
#define MAX_CHANNEL                0xFF
#define MAX_DEVICE_ADDR            0xFF

// === CRC 검증: 예전 코드 확인 전까지는 OFF 권장 ===
#define CRC_CHECK_ENABLED          0   // 나중에 1로 올리고 CRC 방식 확정하면 됨

// CRC 후보 (선택용). 지금은 CRC_CHECK_ENABLED=0이라 사용 안 됨.
// 나중에 방식 확인되면 하나만 남기고 쓰면 됨.
enum CRC_MODE_T {
  CRC_CCITT_0x1021 = 0,
  CRC_IBM_0xA001   = 1,  // Modbus/IBM
};
#define CRC_MODE  CRC_CCITT_0x1021

/* ========== 로터리(채널 변경) 입력 핀 예시 ========== */
// 4비트 로터리(0~15)라고 가정한 예시. 네 실제 로터리에 맞게 수정.
#define ROT_EN                 0   // 1이면 로터리 입력 사용
#define ROT_PIN0              32
#define ROT_PIN1              33
#define ROT_PIN2              25
#define ROT_PIN3              26

/* ===================== 통계 ===================== */
// 콜백(WiFi task) ↔ loop() 간 동기화 필요
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t cnt_total_action = 0;
uint32_t cnt_len_fail     = 0;
uint32_t cnt_len_mismatch = 0;
uint32_t cnt_field_fail   = 0;
uint32_t cnt_crc_fail     = 0;
uint32_t cnt_valid        = 0;

int32_t  rssi_sum = 0;
int32_t  rssi_cnt = 0;

uint8_t  g_channel = DEFAULT_SNIFF_CHANNEL;

/* ===================== CRC (옵션) ===================== */

static inline uint16_t crc16_ccitt_1021(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
  }
  return crc;
}

static inline uint16_t crc16_ibm_a001(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
  }
  return crc;
}

static inline uint16_t calc_crc16(const uint8_t *data, uint16_t len)
{
#if CRC_MODE == CRC_CCITT_0x1021
  return crc16_ccitt_1021(data, len);
#else
  return crc16_ibm_a001(data, len);
#endif
}

/* ===================== 로터리 채널 읽기 ===================== */

static inline uint8_t read_rotary_channel_0_15()
{
#if ROT_EN == 0
  return g_channel;
#else
  uint8_t b0 = digitalRead(ROT_PIN0) ? 1 : 0;
  uint8_t b1 = digitalRead(ROT_PIN1) ? 1 : 0;
  uint8_t b2 = digitalRead(ROT_PIN2) ? 1 : 0;
  uint8_t b3 = digitalRead(ROT_PIN3) ? 1 : 0;
  uint8_t v = (b3 << 3) | (b2 << 2) | (b1 << 1) | (b0 << 0);

  // 프로젝트에서 실제 RF 채널 매핑이 다르면 여기서 변환
  // 예: 0~15 → 1~13 매핑 등
  // 지금은 "그대로 채널 번호"로 사용한다고 가정
  return v;
#endif
}

static void apply_channel_if_changed()
{
  uint8_t new_ch = read_rotary_channel_0_15();
  if (new_ch == 0 || new_ch > 13) {
    // 일반적인 2.4GHz Wi-Fi 채널 범위(1~13) 밖이면 무시
    // (현장/국가 설정에 따라 14는 예외. 보통 안 씀)
    return;
  }

  if (new_ch != g_channel) {
    g_channel = new_ch;
    esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);
    // 채널 전환 로그는 1회만
    Serial.printf("[CH] set to %u\n", g_channel);
  }
}

/* ===================== Payload 검사 ===================== */
/*
  payload 포맷(옛날 AP→Peer):
  [0] GROUP
  [1] CHANNEL
  [2] COMMAND
  [3] DEVICE_TYPE
  [4] DEVICE_ADDRESS
  [5] LENGTH (data bytes)
  [6..] DATA
  [end-2..end-1] CRC16 (LSB, MSB)  // 네가 준 설명 기준
*/

static inline bool is_valid_payload(const uint8_t *payload, uint16_t payload_len, int rssi)
{
  // 최소 길이: 6(header) + 2(CRC) = 8
  if (payload_len < 8) return false;

  uint8_t group   = payload[0];
  uint8_t channel = payload[1];
  uint8_t cmd     = payload[2];
  uint8_t devtype = payload[3];
  uint8_t addr    = payload[4];
  uint8_t dlen    = payload[5];

  // LENGTH 일관성: payload_len == 6 + dlen + 2
  uint16_t expected = (uint16_t)(6 + dlen + 2);
  if (payload_len != expected) return false;

  // 범위 체크 (너가 준 범위 반영)
  if (group   > MAX_GROUP)        return false;
  if (channel > MAX_CHANNEL)      return false;
  if (addr    > MAX_DEVICE_ADDR)  return false;

  if (cmd < CMD_MIN || cmd > CMD_MAX) return false;
  if (devtype < DEVTYPE_MIN || devtype > DEVTYPE_MAX) return false;

#if CRC_CHECK_ENABLED
  uint16_t rx_crc = (uint16_t)payload[payload_len - 2] |
                    ((uint16_t)payload[payload_len - 1] << 8); // LSB,MSB 가정
  uint16_t calc = calc_crc16(payload, payload_len - 2);
  if (rx_crc != calc) return false;
#endif

  // 여기까지 오면 “의미있는 패킷”으로 취급
  (void)rssi;
  return true;
}

/* ===================== Sniffer Callback ===================== */

void sniffer_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
  if (type != WIFI_PKT_MGMT) return;

  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
  uint8_t *frame = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;
  int rssi = pkt->rx_ctrl.rssi;

  // Action frame 간이 체크 (입문/실무 절충)
  if (frame[0] != 0xD0) return;

  // 기본 헤더 24바이트 가정
  if (len <= 24) return;

  uint8_t *payload = frame + 24;
  uint16_t payload_len = (uint16_t)(len - 24);

  bool ok = is_valid_payload(payload, payload_len, rssi);

  portENTER_CRITICAL_ISR(&g_mux);
  cnt_total_action++;

  if (!ok) {
    // 실패 이유 분리(대략)
    if (payload_len < 8) cnt_len_fail++;
    else {
      // LENGTH mismatch / field / crc를 더 세분화하고 싶으면 여기서 분기 가능
      // 현재는 "유효성 실패"를 상황별로 좀 더 나눠 집계
      uint8_t dlen = payload[5];
      uint16_t expected = (uint16_t)(6 + dlen + 2);
      if (payload_len != expected) cnt_len_mismatch++;
      else {
#if CRC_CHECK_ENABLED
        // CRC fail / field fail 분리
        uint8_t cmd = payload[2];
        uint8_t devtype = payload[3];
        if (cmd < CMD_MIN || cmd > CMD_MAX || devtype < DEVTYPE_MIN || devtype > DEVTYPE_MAX) cnt_field_fail++;
        else cnt_crc_fail++;
#else
        // CRC OFF일 때는 필드 실패만 집계(나머지는 len mismatch로 대부분 걸러짐)
        uint8_t cmd = payload[2];
        uint8_t devtype = payload[3];
        if (cmd < CMD_MIN || cmd > CMD_MAX || devtype < DEVTYPE_MIN || devtype > DEVTYPE_MAX) cnt_field_fail++;
        else cnt_field_fail++; // CRC를 안 보니 “정상인지 확정 불가” 영역 → field_fail로 흡수
#endif
      }
    }
  } else {
    cnt_valid++;
    rssi_sum += rssi;
    rssi_cnt++;
  }
  portEXIT_CRITICAL_ISR(&g_mux);
}

/* ===================== Setup / Loop ===================== */

void setup()
{
  Serial.begin(115200);
  delay(300);

#if ROT_EN
  pinMode(ROT_PIN0, INPUT_PULLUP);
  pinMode(ROT_PIN1, INPUT_PULLUP);
  pinMode(ROT_PIN2, INPUT_PULLUP);
  pinMode(ROT_PIN3, INPUT_PULLUP);
#endif

  WiFi.mode(WIFI_MODE_NULL);

  esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous_rx_cb(sniffer_cb);
  esp_wifi_set_promiscuous(true);

  Serial.println("=== ESP-NOW RF Sniffer Jig v1 ===");
  Serial.printf("CH=%u, CRC_CHECK=%s\n", g_channel, CRC_CHECK_ENABLED ? "ON" : "OFF");
}

void loop()
{
  static uint32_t last_report = 0;
  uint32_t now = millis();

  // 로터리로 채널 변경(현장 구간 이동 대비)
  apply_channel_if_changed();

  if (now - last_report >= REPORT_INTERVAL_MS) {
    last_report = now;

    uint32_t a, v, lf, lm, ff, cf;
    int32_t rs, rc;

    portENTER_CRITICAL(&g_mux);
    a  = cnt_total_action;
    v  = cnt_valid;
    lf = cnt_len_fail;
    lm = cnt_len_mismatch;
    ff = cnt_field_fail;
    cf = cnt_crc_fail;
    rs = rssi_sum;
    rc = rssi_cnt;

    // reset
    cnt_total_action = 0;
    cnt_valid        = 0;
    cnt_len_fail     = 0;
    cnt_len_mismatch = 0;
    cnt_field_fail   = 0;
    cnt_crc_fail     = 0;
    rssi_sum         = 0;
    rssi_cnt         = 0;
    portEXIT_CRITICAL(&g_mux);

    int avg_rssi = (rc > 0) ? (rs / rc) : 0;

    // 1ms 주기면 정상일 때 valid가 900~1100/s 근처가 나올 것(상황/필터/거리 따라 변동)
    Serial.printf("[1s][CH%u] action=%lu valid=%lu lenF=%lu lenM=%lu fieldF=%lu crcF=%lu avgRSSI=%d\n",
                  g_channel, a, v, lf, lm, ff, cf, avg_rssi);
  }

  delay(5);
}
