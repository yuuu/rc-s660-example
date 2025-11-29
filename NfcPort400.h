#pragma once
#include <Arduino.h>

struct NfcPort400FirmwareVersion {
  uint32_t overallFw;     // 全体ファームウェアバージョン (32bit)
  uint16_t mcuFw;         // MCUファームウェアバージョン
  uint16_t samFw;         // SAMファームウェアバージョン
  uint16_t rffeFw;        // RFFEファームウェアバージョン
  uint16_t rffeEeprom;    // RFFE EEPROMバージョン
  uint16_t bootloader;    // ブートローダバージョン
  uint16_t fwUpdateState; // ファームウェア更新状態
  uint16_t bootState;     // 起動状態 (0:FW起動, 1:BL起動)
  uint8_t  sw1;           // APDUステータスSW1
  uint8_t  sw2;           // APDUステータスSW2
};

class NfcPort400 {
public:
  // RC-S660/S がつながっている UART (ESP32なら Serial1 など)
  explicit NfcPort400(Stream &serial);

  // --- 既存: Get Firmware Version ---
  bool getFirmwareVersion(NfcPort400FirmwareVersion &out,
                          uint32_t ackTimeoutMs   = 500,
                          uint32_t respTimeoutMs  = 1000);

  // --- 追加: Transparent Session / FeliCa 関連 ---

  // Transparent Session を開始 (Manage Session / Start Transparent Session)
  bool startTransparentSession(uint32_t timeoutMs = 1000);

  // Transparent Session を終了 (Manage Session / End Transparent Session)
  bool endTransparentSession(uint32_t timeoutMs = 1000);

  // FeliCa 通信用に設定 (Switch Protocol: StandardType=0x03, Layer=0x00)
  bool switchToFelica(uint32_t timeoutMs = 1000);

  // RF ON (Manage Session / Turn On RF Field)
  bool rfOn(uint32_t timeoutMs = 1000);

  // FeliCa Polling を投げて IDm (必須), PMm(任意) を取得
  // systemCode: 0xFFFF で「任意のシステム」
  // requestCode: 0x00 (デフォルト: 追加情報なし)
  // timeSlot: 0x00 (1スロット)
  // idmOut: 8バイトのバッファ
  // pmmOut: 8バイトのバッファ、不要なら nullptr
  bool felicaPolling(uint16_t systemCode,
                     uint8_t requestCode,
                     uint8_t timeSlot,
                     uint8_t *idmOut,   // 8 bytes
                     uint8_t *pmmOut,   // 8 bytes or nullptr
                     uint32_t ackTimeoutMs  = 500,
                     uint32_t respTimeoutMs = 1000);

private:
  Stream &m_serial;

  bool readBytesWithTimeout(uint8_t *buf, size_t len, uint32_t timeoutMs);
  bool readHostPacket(uint8_t *pdBuf, uint16_t &pdLen, bool &isAck, uint32_t timeoutMs);
  static uint8_t calcLCS(uint16_t len);
  static uint8_t calcDCS(const uint8_t *data, int size);

  // PC_to_RDR_Escape で APDU を送信し、APDUレスポンス(abData)を受け取る共通処理
  bool sendEscapeApdu(const uint8_t *apdu, uint8_t apduLen,
                      uint8_t *apduResp, uint16_t &apduRespLen,
                      uint32_t ackTimeoutMs, uint32_t respTimeoutMs);

  // Transparent Exchange の Data field (BER-TLV) から Tag=0x97 (ICC Response) を探す
  bool findTlvTag97(const uint8_t *data, uint16_t len,
                    const uint8_t *&val, uint8_t &valLen);
};
