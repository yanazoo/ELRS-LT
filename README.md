# ELRS Lap Timer

🌐 **日本語** | [English](README.en.md)

**HappyModel EP1/EP2 TCXO** に焼いたカスタムスニファーファームで、
ELRS テレメトリ uplink の RSSI からドローンの通過を検出するラップタイマー。

> ESP-NOW-Lap-Timer（専用ビーコン方式）を改良したもの。
> ドローン側は標準 ELRS ファームをそのまま使い、ゲート側の EP1/EP2 TCXO がスニファーとして機能する。

## 特徴

- **ドローン改造不要** — 標準 ELRS ファームのまま。XIAO ビーコン不要
- **EP1/EP2 TCXO をゲート受信機に流用** — ESP8285 + SX1280 + TCXO。カスタムファームで FHSS 追従 + RSSI 計測
- 最大 4 パイロット同時計測、最大 20 名ロースター管理
- RSSI ピーク検出 + RotorHazard 準拠の状態機械でゲート通過を判定
- EMA フィルタ（α=0.3）によるスムーズな RSSI 処理
- GitHub Dark テーマ Web UI（日本語 TTS・Canvas 波形グラフ・SD ファイルブラウザ）
- SD カードへのレース CSV 自動記録・名簿バックアップ/復元
- HSモード / 計測モード 切替対応

---

## ベースプロジェクト

[yanazoo/ESP-NOW-Lap-Timer](https://github.com/yanazoo/ESP-NOW-Lap-Timer) の改良版。

| | ESP-NOW-Lap-Timer | ELRS Lap Timer（本リポジトリ） |
|---|---|---|
| ドローン側 | XIAO ESP32-C3 ビーコン搭載 | 標準 ELRS ファームのまま（改造不要） |
| ゲート受信 | ESP32 Promiscuous モード | EP1/EP2 TCXO カスタムスニファーファーム |
| 識別キー | ESP-NOW 送信元 MAC | 6 バイト ELRS バインド UID |
| ゲートとのやりとり | ESP-NOW | ESP-NOW（同じ） |

---

## ハードウェア構成

```
[ドローン] EP1/EP2 TCXO (標準 ELRS ファーム, パイロット TX にバインド) → 通常の RC RX

[ゲート]   EP1/EP2 TCXO x4 (カスタムスニファーファーム)
               FHSS 同期 + RSSI 計測 → ESP-NOW
[ゲート]   ESP32-WROVER (TTGO T8) → EMA 状態機械 + ラップ検出
               UART JSON
[ピット]   XIAO ESP32-S3 → WiFi AP + Web UI
               WiFi SSID: ESP-NOW-LT  PASS: esp-now-lt  IP: 20.0.0.1
```

### ピン配線（Gate Node ↔ Web Node）

| ESP32-WROVER-E (Gate) | 方向 | XIAO ESP32-S3 (Web) |
|-----------------------|------|----------------------|
| GPIO26 (TX1)          | →   | GPIO3 / D2 (RX1)    |
| GPIO25 (RX1)          | ←   | GPIO2 / D1 (TX1)    |
| GND                   | —   | GND                  |

---

## ビルド・書き込み

```bash
# Gate EP1 Sniffer (EP1/EP2 TCXO = ESP8285) — パイロット数分用意
pio run -e gate_ep1 -t upload

# Gate Node (ESP32-WROVER-E / LilyGo TTGO T8 V1.8)
pio run -e gate_node -t upload

# Web Node (XIAO ESP32-S3)
pio run -e web_node -t upload
pio run -e web_node -t uploadfs   # JS/HTML 変更後に必要
```

EP1/EP2 書き込み配線（USB-シリアル変換, 3.3V）:

```
EP pad    変換アダプタ
TX    →   RX
RX    ←   TX
GND   —   GND
VCC   —   3V3
```

> ブート方法: RX パッド（GPIO3）を LOW にしたまま電源投入 → ESP8285 が UART ブートローダーに入る

---

## Web UI

**接続:** WiFi SSID `ESP-NOW-LT` (PASS: `esp-now-lt`) → ブラウザで `http://20.0.0.1`

---

## ソースコード構成

```
src/
├─ gate_ep1/    EP1/EP2 TCXO スニファーファーム (ESP8285)
├─ gate_node/   ゲートノード (TTGO T8) — ESP-NOW 受信 + EMA ラップ検出
└─ web_node/    Web ノード (XIAO S3) — WiFi AP + レース管理 UI
data/           Web UI (LittleFS)
boards/         カスタムボード定義
```

詳細は [HANDOFF.md](HANDOFF.md) と [ARCHITECTURE.md](ARCHITECTURE.md) を参照。

---

## 関連リポジトリ

- ベース: [yanazoo/ESP-NOW-Lap-Timer](https://github.com/yanazoo/ESP-NOW-Lap-Timer)
- ExpressLRS（FHSS / SX1280 ドライバ参考）: [ExpressLRS/ExpressLRS](https://github.com/ExpressLRS/ExpressLRS)
- PhobosLT（4ch SPI 構成参考）: [yanazoo/PhobosLT_4ch](https://github.com/yanazoo/PhobosLT_4ch)
