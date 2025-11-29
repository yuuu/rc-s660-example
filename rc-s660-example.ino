#include <Arduino.h>
#include "NfcPort400.h"

const int RX_PIN = 18; // RC-S660/S の TXD
const int TX_PIN = 19; // RC-S660/S の RXD

NfcPort400 *nfc;

// 前回検出したカードの IDm
uint8_t lastIdm[8];
bool lastIdmValid = false;

// 連続失敗回数
int failureStreak = 0;

void printHex(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
}

void reinitNfcPort400() {
  Serial.println("Re-init NFC Port-400...");

  // Transparent Session を一旦閉じてから再度開き直す
  // 失敗しても続行（セッションが既に閉じているケースもあるので）
  nfc->endTransparentSession();

  delay(10);

  if (!nfc->startTransparentSession()) {
    Serial.println("  startTransparentSession failed.");
    return;
  }

  if (!nfc->switchToFelica()) {
    Serial.println("  switchToFelica failed.");
    return;
  }

  if (!nfc->rfOn()) {
    Serial.println("  rfOn failed.");
    return;
  }

  Serial.println("  Re-init done.");
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

  Serial.println("Ready. Please touch a FeliCa card...");
}

void loop() {
  uint8_t idm[8];
  uint8_t pmm[8];

  bool ok = nfc->felicaPolling(0xFFFF, 0x00, 0x00, idm, pmm);

  if (ok) {
    // ポーリング成功 → 失敗カウンタをリセット
    failureStreak = 0;

    // すでに有効な IDm があり、今回と同じなら何も表示しない
    bool isSameAsLast = lastIdmValid && (memcmp(idm, lastIdm, 8) == 0);

    if (!isSameAsLast) {
      Serial.print("FeliCa card detected! IDm: ");
      printHex(idm, 8);
      Serial.print("  PMm: ");
      printHex(pmm, 8);
      Serial.println();

      memcpy(lastIdm, idm, 8);
      lastIdmValid = true;
    }

    delay(200);  // 読み取り間隔
  } else {
    // 今回ポーリング失敗
    failureStreak++;

    // 直前までカードがいたなら「離脱」とみなす
    if (lastIdmValid) {
      Serial.println("FeliCa card removed.");
      lastIdmValid = false;
    }

    // 一定回数連続で失敗したら、Port-400 を再初期化
    if (failureStreak >= 5) {  // 約1秒分 (5回×200ms)
      reinitNfcPort400();
      failureStreak = 0;
    }

    delay(200);
  }
}

