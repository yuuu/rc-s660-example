#include "NfcPort400.h"

// ----------------- ユーティリティ -----------------

uint8_t NfcPort400::calcLCS(uint16_t len) {
  uint8_t lenH = (len >> 8) & 0xFF;
  uint8_t lenL = len & 0xFF;
  return (uint8_t)(0x100 - ((lenH + lenL) & 0xFF));
}

uint8_t NfcPort400::calcDCS(const uint8_t *data, int size) {
  uint16_t sum = 0;
  for (int i = 0; i < size; i++) sum += data[i];
  return (uint8_t)(0x100 - (sum & 0xFF));
}

NfcPort400::NfcPort400(Stream &serial)
  : m_serial(serial) {
}

bool NfcPort400::readBytesWithTimeout(uint8_t *buf, size_t len, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t i = 0;
  while (i < len) {
    if (m_serial.available()) {
      buf[i++] = (uint8_t)m_serial.read();
    } else {
      if (millis() - start > timeoutMs) {
        return false;
      }
      // ★ここが重要：他タスクに譲って WDT をなだめる
      delay(1);   // または yield();
    }
  }
  return true;
}


// ホストパケット (00 00 FF LENH LENL LCS PD... DCS 00) を1つ読む
bool NfcPort400::readHostPacket(uint8_t *pdBuf, uint16_t &pdLen, bool &isAck, uint32_t timeoutMs) {
  uint8_t b;
  uint32_t start = millis();

  enum State { WAIT_PRE, WAIT_SC1, WAIT_SC2 };
  State st = WAIT_PRE;

  // preamble(00) + start code(00 FF) を探す
  while (true) {
    if (m_serial.available()) {
      b = (uint8_t)m_serial.read();
      switch (st) {
        case WAIT_PRE:
          if (b == 0x00) st = WAIT_SC1;
          break;
        case WAIT_SC1:
          if (b == 0x00) {
            st = WAIT_SC2;
          } else {
            st = WAIT_PRE;
          }
          break;
        case WAIT_SC2:
          if (b == 0xFF) {
            goto FOUND_START;
          } else {
            st = WAIT_PRE;
          }
          break;
      }
    } else {
      if (millis() - start > timeoutMs) {
        return false;
      }
      // WDT対策: 他タスクに譲る
      delay(1);
    }
  }

FOUND_START:
  // LEN(2) + LCS(1)
  uint8_t header[3];
  if (!readBytesWithTimeout(header, 3, timeoutMs)) return false;
  uint8_t lenH = header[0];
  uint8_t lenL = header[1];
  uint8_t lcs  = header[2];
  uint16_t len = ((uint16_t)lenH << 8) | lenL;

  // ACKフレーム: LEN=0, LCS=0xFF
  if (len == 0 && lcs == 0xFF) {
    uint8_t post;
    if (!readBytesWithTimeout(&post, 1, timeoutMs)) return false;
    isAck = true;
    pdLen = 0;
    return true;
  }

  // LCS チェック
  if (lcs != calcLCS(len)) {
    return false;
  }

  // PD + DCS + POST
  if (!readBytesWithTimeout(pdBuf, len, timeoutMs)) return false;
  uint8_t dcs, post;
  if (!readBytesWithTimeout(&dcs, 1, timeoutMs)) return false;
  if (!readBytesWithTimeout(&post, 1, timeoutMs)) return false;

  if (dcs != calcDCS(pdBuf, len)) {
    return false;
  }

  isAck = false;
  pdLen = len;
  return true;
}

// PC_to_RDR_Escape で APDU を送信し、APDUレスポンス(abData)を受け取る
bool NfcPort400::sendEscapeApdu(const uint8_t *apdu, uint8_t apduLen,
                                uint8_t *apduResp, uint16_t &apduRespLen,
                                uint32_t ackTimeoutMs, uint32_t respTimeoutMs) {
  // CCID PC_to_RDR_Escape
  // bMessageType(1) + dwLength(4) + bSlot(1) + bSeq(1) + RFU(3) + APDU
  uint8_t seqNum = 0x01; // シンプルに固定シーケンス番号
  uint16_t pdLen = 1 + 4 + 1 + 1 + 3 + apduLen;
  uint8_t pd[128];
  int p = 0;

  pd[p++] = 0x6B;                 // bMessageType (PC_to_RDR_Escape)
  pd[p++] = apduLen & 0xFF;       // dwLength (LE)
  pd[p++] = (apduLen >> 8) & 0xFF;
  pd[p++] = 0x00;
  pd[p++] = 0x00;
  pd[p++] = 0x00;                 // bSlot
  pd[p++] = seqNum;               // bSeq
  pd[p++] = 0x00; pd[p++] = 0x00; pd[p++] = 0x00; // RFU

  for (uint8_t i = 0; i < apduLen; i++) {
    pd[p++] = apdu[i];
  }

  // ホストパケット
  uint8_t pre  = 0x00;
  uint8_t sc[2] = {0x00, 0xFF};
  uint8_t lenH = (pdLen >> 8) & 0xFF;
  uint8_t lenL = pdLen & 0xFF;
  uint8_t lcs  = calcLCS(pdLen);
  uint8_t dcs  = calcDCS(pd, pdLen);
  uint8_t post = 0x00;

  // 送信
  m_serial.write(pre);
  m_serial.write(sc, 2);
  m_serial.write(lenH);
  m_serial.write(lenL);
  m_serial.write(lcs);
  m_serial.write(pd, pdLen);
  m_serial.write(dcs);
  m_serial.write(post);

  // ACK 受信
  uint8_t buf[256];
  uint16_t hostPdLen = 0;
  bool isAck = false;

  if (!readHostPacket(buf, hostPdLen, isAck, ackTimeoutMs)) {
    return false;
  }
  if (!isAck) {
    return false;
  }

  // レスポンスパケット受信
  if (!readHostPacket(buf, hostPdLen, isAck, respTimeoutMs)) {
    return false;
  }
  if (isAck) {
    return false;
  }

  // CCID (RDR_to_PC_*) 解析
  if (hostPdLen < 10) {
    return false;
  }

  uint8_t bMessageType = buf[0];
  if (bMessageType != 0x83 && bMessageType != 0x80 && bMessageType != 0x81) {
    // 主に RDR_to_PC_Escape (0x83) を想定
    return false;
  }

  uint32_t dwLen = (uint32_t)buf[1] |
                   ((uint32_t)buf[2] << 8) |
                   ((uint32_t)buf[3] << 16) |
                   ((uint32_t)buf[4] << 24);

  uint8_t bStatus = buf[7];
  uint8_t bError  = buf[8];

  // CCIDレベルエラー
  if ((bStatus & 0xC0) != 0x00 || bError != 0x00) {
    return false;
  }

  if (dwLen + 10 > hostPdLen) {
    return false;
  }

  // abData (= APDUレスポンス)
  const uint8_t *abData = &buf[10];
  apduRespLen = (uint16_t)dwLen;
  if (apduRespLen > 240) return false;

  for (uint16_t i = 0; i < apduRespLen; i++) {
    apduResp[i] = abData[i];
  }

  return true;
}

// BER-TLV の中をなめて Tag=0x97(ICC Response) を探す
bool NfcPort400::findTlvTag97(const uint8_t *data, uint16_t len,
                              const uint8_t *&val, uint8_t &valLen) {
  uint16_t i = 0;
  while (i + 2 <= len) {
    uint8_t tag1 = data[i++];

    // 2バイトタグ(5F, FFなど)はスキップ
    bool twoByteTag = (tag1 == 0x5F || tag1 == 0xFF);
    uint8_t tag2 = 0;
    if (twoByteTag) {
      if (i >= len) return false;
      tag2 = data[i++];
    }

    if (i >= len) return false;
    uint8_t L = data[i++];

    if (!twoByteTag && tag1 == 0x97) {
      if (i + L > len) return false;
      val = &data[i];
      valLen = L;
      return true;
    }

    // 次のTLVへ
    i += L;
  }
  return false;
}

// ----------------- Get Firmware Version -----------------

bool NfcPort400::getFirmwareVersion(NfcPort400FirmwareVersion &out,
                                    uint32_t ackTimeoutMs,
                                    uint32_t respTimeoutMs) {
  // APDU: FF 56 00 00 00 (Le=0x00)
  uint8_t apdu[] = {0xFF, 0x56, 0x00, 0x00, 0x00};
  uint8_t resp[64];
  uint16_t respLen = 0;

  if (!sendEscapeApdu(apdu, sizeof(apdu), resp, respLen, ackTimeoutMs, respTimeoutMs)) {
    return false;
  }
  if (respLen < 20) return false;

  const uint8_t *d = resp;
  uint16_t dataLen = respLen - 2;
  auto rd16 = [](const uint8_t *p) -> uint16_t {
    return ((uint16_t)p[0] << 8) | p[1];
  };

  out.overallFw =
    ((uint32_t)d[0] << 24) |
    ((uint32_t)d[1] << 16) |
    ((uint32_t)d[2] <<  8) |
    ((uint32_t)d[3]);
  out.mcuFw       = rd16(&d[4]);
  out.samFw       = rd16(&d[6]);
  out.rffeFw      = rd16(&d[8]);
  out.rffeEeprom  = rd16(&d[10]);
  out.bootloader  = rd16(&d[12]);
  out.fwUpdateState = rd16(&d[14]);
  out.bootState     = rd16(&d[16]);
  out.sw1           = resp[respLen - 2];
  out.sw2           = resp[respLen - 1];

  return true;
}

// ----------------- Transparent Session / FeliCa 関連 -----------------

bool NfcPort400::startTransparentSession(uint32_t timeoutMs) {
  // APDU: FF C2 00 00 Lc Data Le
  // Data: 81 00 (Start Transparent Session)
  uint8_t apdu[] = {
    0xFF, 0xC2, 0x00, 0x00,
    0x02,       // Lc
    0x81, 0x00, // Tag=81h, Length=0
    0x00        // Le=0x00
  };
  uint8_t resp[32];
  uint16_t respLen = 0;
  if (!sendEscapeApdu(apdu, sizeof(apdu), resp, respLen, timeoutMs, timeoutMs)) return false;
  if (respLen < 2) return false;
  // SW1SW2 のチェック (90 00 or 62 82 など許容するなら条件ゆるくしてもOK)
  return true;
}

bool NfcPort400::endTransparentSession(uint32_t timeoutMs) {
  // Data: 82 00 (End Transparent Session)
  uint8_t apdu[] = {
    0xFF, 0xC2, 0x00, 0x00,
    0x02,
    0x82, 0x00,
    0x00
  };
  uint8_t resp[32];
  uint16_t respLen = 0;
  if (!sendEscapeApdu(apdu, sizeof(apdu), resp, respLen, timeoutMs, timeoutMs)) return false;
  if (respLen < 2) return false;
  return true;
}

bool NfcPort400::switchToFelica(uint32_t timeoutMs) {
  // Switch Protocol APDU
  // CLA INS P1 P2 Lc Data Le
  // FF C2 00 02 Lc (8F 02 03 00) 00
  //   8F: Switch Protocol Data Object, Length=2
  //     03: StandardType=FeliCa
  //     00: Layer=FeliCa通信用に設定 (Pollingは自前でやる)
  uint8_t apdu[] = {
    0xFF, 0xC2, 0x00, 0x02,
    0x04,
    0x8F, 0x02, 0x03, 0x00,
    0x00
  };
  uint8_t resp[64];
  uint16_t respLen = 0;
  if (!sendEscapeApdu(apdu, sizeof(apdu), resp, respLen, timeoutMs, timeoutMs)) return false;
  if (respLen < 2) return false;
  return true;
}

bool NfcPort400::rfOn(uint32_t timeoutMs) {
  // Manage Session: Turn On RF Field
  // Data: 84 00
  uint8_t apdu[] = {
    0xFF, 0xC2, 0x00, 0x00,
    0x02,
    0x84, 0x00,
    0x00
  };
  uint8_t resp[32];
  uint16_t respLen = 0;
  if (!sendEscapeApdu(apdu, sizeof(apdu), resp, respLen, timeoutMs, timeoutMs)) return false;
  if (respLen < 2) return false;
  return true;
}

bool NfcPort400::felicaPolling(uint16_t systemCode,
                               uint8_t requestCode,
                               uint8_t timeSlot,
                               uint8_t *idmOut,
                               uint8_t *pmmOut,
                               uint32_t ackTimeoutMs,
                               uint32_t respTimeoutMs) {
  if (!idmOut) return false;

  // --- 1) FeliCa Polling コマンド ---
  // [LEN][CMD=0x00][SystemCode(2)][RequestCode][TimeSlot]
  uint8_t poll[6];
  poll[0] = 6;                 // 全長
  poll[1] = 0x00;              // Polling command code
  poll[2] = (uint8_t)(systemCode >> 8);
  poll[3] = (uint8_t)(systemCode & 0xFF);
  poll[4] = requestCode;
  poll[5] = timeSlot;

  // --- 2) Transparent Exchange 用 Data Object ---
  // シンプルに 95(Transceive) のみ使用
  uint8_t dob[32];
  uint8_t idx = 0;

  dob[idx++] = 0x95;               // Tag = Transceive
  dob[idx++] = sizeof(poll);       // Length
  for (uint8_t i = 0; i < sizeof(poll); i++) {
    dob[idx++] = poll[i];
  }

  uint8_t Lc = idx;

  // --- 3) Transparent Exchange APDU ---
  // CLA INS P1 P2 Lc Data Le
  // FF C2 00 01 Lc [95...] 00
  uint8_t apdu[4 + 1 + 32 + 1];
  uint8_t apdx = 0;
  apdu[apdx++] = 0xFF;
  apdu[apdx++] = 0xC2;
  apdu[apdx++] = 0x00;
  apdu[apdx++] = 0x01;   // P2=01: Transparent Exchange
  apdu[apdx++] = Lc;     // Lc
  for (uint8_t i = 0; i < Lc; i++) apdu[apdx++] = dob[i];
  apdu[apdx++] = 0x00;   // Le

  uint8_t resp[128];
  uint16_t respLen = 0;
  if (!sendEscapeApdu(apdu, apdx, resp, respLen, ackTimeoutMs, respTimeoutMs)) {
    return false;
  }
  if (respLen < 2) {
    return false;
  }

  // 最後 2バイトが SW1 SW2
  uint8_t sw1 = resp[respLen - 2];
  uint8_t sw2 = resp[respLen - 1];

  // 正常終了 (90 00) 以外は「カードなし or エラー」とみなして false
  if (sw1 != 0x90 || sw2 != 0x00) {
    return false;
  }

  // TLV 部分
  uint16_t dataLen = respLen - 2;
  const uint8_t *data = resp;

  // TLV の中から Tag=0x97 (ICC Response) を探す
  const uint8_t *icc = nullptr;
  uint8_t iccLen = 0;
  if (!findTlvTag97(data, dataLen, icc, iccLen)) {
    return false;
  }

  // FeliCa Polling 応答:
  // [LEN][RESP=0x01][IDm(8)][PMm(8)]...
  if (iccLen < 18) return false;
  if (icc[1] != 0x01) return false; // Response code=Polling response

  // IDm 抽出
  for (int i = 0; i < 8; i++) {
    idmOut[i] = icc[2 + i];
  }

  // PMm 抽出
  if (pmmOut) {
    for (int i = 0; i < 8; i++) {
      pmmOut[i] = icc[10 + i];
    }
  }

  return true;
}


