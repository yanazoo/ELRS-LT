# Gate ESP32 の変更（`src/gate_node/`）

目的: 生 promiscuous ESP-NOW フレーム（送信元 MAC キー）から RSSI を読むのを
やめ、代わりに EP スニファーから `GateEP1Packet_t`（UID キー）を受信し、
既存の EMA 状態機械に流す。Web Node への UART プロトコルと `processRSSI()` は
変更しない。

## promiscuous.*（任意で espnow_rx.* にリネーム可）

promiscuous スニファー設定を、通常の ESP-NOW 受信コールバックに置き換える:

```cpp
// src/gate_ep1/config.h の GateEP1Packet_t と完全一致させること
typedef struct __attribute__((packed)) {
    uint8_t  pilot_uid[6];
    int8_t   rssi;
    uint8_t  lq;
    uint32_t ts;
} GateEP1Packet_t;

void onEspNowRecv(const uint8_t *srcMac, const uint8_t *data, int len) {
    if (len != sizeof(GateEP1Packet_t)) return;
    const GateEP1Packet_t *pkt = (const GateEP1Packet_t*)data;

    int pilotIdx = findPilotByUID(pkt->pilot_uid);   // pilots.* の変更
    if (pilotIdx < 0) return;                         // 未知 -> scan報告

    processRSSI(pilotIdx, pkt->rssi);                 // 以降は変更なし
}
```

初期化: スニファーと同じ WiFi チャンネルで `esp_now_init()` +
`esp_now_register_recv_cb(onEspNowRecv)`。promiscuous モード /
無線ヘッダから RSSI を取る経路は完全に削除する。

## pilots.*（MAC -> UID）

- パイロットごとのキー項目を MAC から `uint8_t uid[6]` に変更
  （同じ6バイト・同じワイヤ書式なので JSON/UART 層はほぼ変わらない）。
- MAC ルックアップを `findPilotByUID(const uint8_t uid[6])` に置き換える。
- scan 報告: 未知 UID は、従来の未知 MAC と同じく Config タブの scan リストに出す。

## 変更しない部分

- `uart_gate.*` の JSON メッセージ（`lap`, `rssi`, `scan` 等）。`uid` フィールドは
  既に `AA:BB:CC:DD:EE:FF` テキストを運んでおり、その意味が「バインド UID」になる。
- `sd_gate.*` の CSV 列（`UID` 列ラベルがそのまま合う）。
- `main.cpp` の EMA + Enter/Exit + クールダウン状態機械。

## Web UI（`data/`）

- Config タブ: MAC 入力欄を "UID" にラベル変更し、6バイトhex を検証。
- UI でパイロットの "MAC" と書かれている箇所は "UID" と読み替える。ロースター
  JSON のフィールド名は churn を抑えるため `mac` のままでもよいし、`uid` に
  リネームしてもよい（その場合 `nvs_store`, `json_api`, `gate_comm` も合わせて更新）。

## テストチェックリスト

- [ ] UID を持たせたスニファー1台が `GateEP1Packet_t` を送り、Gate ESP32 が
      対応する `rssi` JSON を Web Node にログ出力する。
- [ ] ドローン接近で Calib タブの波形が動く。
- [ ] Enter/Exit 閾値で RSSI ピーク時に `lap` がトリガーされる。
- [ ] スニファー4台 / UID 4個が4つの別スロットに解決される。
