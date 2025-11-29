#include <Arduino.h>
#include <M5StampC3LED.h>
#include "NfcPort400.h"

const int RX_PIN = 18; // RC-S660/S の TXD
const int TX_PIN = 19; // RC-S660/S の RXD

M5StampC3LED led = M5StampC3LED();
NfcPort400 *nfc;

// タイムアウト設定(ms)
const uint32_t ACK_TIMEOUT = 200;    // ACK待ち時間を短縮
const uint32_t RESP_TIMEOUT = 300;   // レスポンス待ち時間を短縮

// ポーリング開始時刻
unsigned long pollingStartTime = 0;
const unsigned long POLLING_TIMEOUT = 10000;  // 10秒でリブート

void printHex(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  // ESP32: UART1 を RX=18, TX=19 に割り当て
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  nfc = new NfcPort400(Serial1);

  Serial.println("=== NFC Port-400 FeliCa demo ===");

  // 一応ファームウェアバージョンを確認
  NfcPort400FirmwareVersion ver;
  if (nfc->getFirmwareVersion(ver)) {
    Serial.print("FW Version overall: 0x");
    Serial.println(ver.overallFw, HEX);
  } else {
    Serial.println("GetFirmwareVersion failed.");
  }

  // Transparent Session 開始
  if (!nfc->startTransparentSession()) {
    Serial.println("startTransparentSession failed.");
    return;
  }

  // FeliCa 通信に切り替え
  if (!nfc->switchToFelica()) {
    Serial.println("switchToFelica failed.");
    return;
  }

  // RF ON
  if (!nfc->rfOn()) {
    Serial.println("rfOn failed.");
    return;
  }

  led.show(0, 0, 0);
  Serial.println("Ready. Please touch a FeliCa card...");

  // ポーリング開始時刻を記録
  pollingStartTime = millis();
}

void loop() {
  // ポーリング開始から10秒経過したらリブート
  if (millis() - pollingStartTime >= POLLING_TIMEOUT) {
    Serial.println("Polling timeout (10s). Rebooting ESP32...");
    delay(100);  // シリアル出力完了を待つ
    ESP.restart();  // ESP32をリブート
  }

  uint8_t idm[8];
  uint8_t pmm[8];

  // タイムアウトを短縮してポーリング
  bool ok = nfc->felicaPolling(0xFFFF, 0x00, 0x00, idm, pmm, ACK_TIMEOUT, RESP_TIMEOUT);

  if (ok) {
    led.show(0, 255, 0);

    // カード検出時は毎回表示
    Serial.print("FeliCa card detected! IDm: ");
    printHex(idm, 8);
    Serial.print("  PMm: ");
    printHex(pmm, 8);
    Serial.println();

    Serial.println("Rebooting ESP32...");
    delay(1000);  // シリアル出力完了を待つ
    led.show(0, 0, 0);
    ESP.restart();  // ESP32をリブート
  } else {
    delay(100);  // ポーリング間隔
  }
}

