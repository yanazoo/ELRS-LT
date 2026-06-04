# ELRS Lap Timer

🌐 **日本語** | [English](README.en.md)

**HappyModel EP1/EP2 TCXO** に焼いたカスタムスニファーファームで、
ドローンが送り返す **ELRS テレメトリ（TRSS）uplink** の RSSI からドローンの通過を検出するラップタイマー。

> ESP-NOW-Lap-Timer（専用ビーコン方式）を改良したもの。
> ドローン側は標準 ELRS ファームをそのまま使い、ゲート側の EP1/EP2 TCXO がスニファーとして機能する。
> キャリブレーショングラフ・ラップ検出ともに **ドローンのテレメトリ信号だけ** を追い、据え置きの送信機（TX）の RSSI には反応しない。

## 特徴

- **ドローン改造不要** — 標準 ELRS ファームのまま。XIAO ビーコン不要
- **EP1/EP2 TCXO をゲート受信機に流用** — ESP8285 + SX1280 + TCXO。カスタムファームで FHSS 追従 + RSSI 計測
- **ドローンのテレメトリ（TRSS）だけを抽出** — OTA パケット種別 `0b11`（PACKET_TYPE_TLM）のみを計測対象にし、据え置きの送信機（TX）の信号レベルや移動には反応しない。キャリブレーショングラフ・ラップ検出ともにドローンの信号だけを追う
- **広いテレメトリ比率に対応** — HOLD + 比率連動サイレンスにより **1:64 まで安定、1:128（500Hz の "Std"）も機能**。ELRS 3.6.3 で検証
- 最大 4 パイロット同時計測、最大 20 名ロースター管理
- RSSI ピーク検出 + RotorHazard 準拠の状態機械でゲート通過を判定
- EMA フィルタ（α=0.25）によるスムーズな RSSI 処理
- GitHub Dark テーマ Web UI（日本語 TTS・Canvas 波形グラフ・SD ファイルブラウザ）
- SD カードへのレース CSV 自動記録・名簿バックアップ/復元
- HSモード / 計測モード 切替対応

---

## バージョン

| ブランチ | テレメトリ比率の安定範囲 | 主な違い |
|---|---|---|
| `1.0.0` | 〜1:4 で安定 | 報告窓ごとに2サンプル要求。1:8 以上は取りこぼしが増える |
| `1.0.1`（= 現 `main`） | **1:64 で安定 / 1:128 は機能** | HOLD + 比率連動サイレンス（min=1）、EMA α=0.25 |

> `main` は常に最新（現在 1.0.1）。各リリースはタグ代わりのブランチ（`1.0.0` / `1.0.1`）として保存しています。

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
  パイロット①  パイロット②  パイロット③  パイロット④
  [TX 送信機]  [TX 送信機]  [TX 送信機]  [TX 送信機]
       ↕ ELRS RC リンク（downlink + uplink, FHSS 2.4GHz LoRa）
  [EP1/EP2 TCXO 標準 ELRS ファーム, ドローン搭載]

  ── ゲート通過時の uplink RSSI をスニファーが検出 ──

  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐
  │EP1 スニファ│ │EP1 スニファ│ │EP1 スニファ│ │EP1 スニファ│
  │ #1 (Pilot1)│ │ #2 (Pilot2)│ │ #3 (Pilot3)│ │ #4 (Pilot4)│
  │ ESP8285    │ │ ESP8285    │ │ ESP8285    │ │ ESP8285    │
  │ +SX1280    │ │ +SX1280    │ │ +SX1280    │ │ +SX1280    │
  └─────┬──────┘ └─────┬──────┘ └─────┬──────┘ └─────┬──────┘
        └───────────────┴───────────────┴───────────────┘
                          ESP-NOW（2.4GHz）
                                ↓
                  ┌─────────────────────────┐
                  │  Gate Node (TTGO T8)    │
                  │  ESP32-WROVER-E         │
                  │  EMA + ラップ検出       │
                  └────────────┬────────────┘
                               │ UART 115200 bps
                  ┌────────────▼────────────┐
                  │  Web Node (XIAO S3)     │
                  │  WiFi AP: ESP-NOW-LT    │
                  │  IP: 20.0.0.1           │
                  └────────────┬────────────┘
                               │ WiFi
                  ┌────────────▼────────────┐
                  │  スマホ / PC ブラウザ    │
                  │  http://20.0.0.1        │
                  └─────────────────────────┘
```

### 結線図 — Gate Node ↔ Web Node（UART）

```
 TTGO T8 (Gate Node)           XIAO ESP32-S3 (Web Node)
 ESP32-WROVER-E                
 ┌─────────────────┐           ┌─────────────────┐
 │  GPIO26 (UART TX) ─────────→ D2 / GPIO3 (RX) │
 │  GPIO25 (UART RX) ←───────── D1 / GPIO2 (TX) │
 │  GND              ─────────── GND             │
 └─────────────────┘           └─────────────────┘
```

| ESP32-WROVER-E (Gate) | 方向 | XIAO ESP32-S3 (Web) |
|-----------------------|------|----------------------|
| GPIO26 (TX1)          | →   | GPIO3 / D2 (RX1)    |
| GPIO25 (RX1)          | ←   | GPIO2 / D1 (TX1)    |
| GND                   | —   | GND                  |

### 結線図 — Gate Node ↔ SD カード（SPI）

```
 TTGO T8 / WROVER-E (Gate Node)
 ┌──────────────────────────────────────┐
 │  GPIO13 (MOSI) ─────→ SD Card MOSI  │
 │  GPIO2  (MISO) ←───── SD Card MISO  │
 │  GPIO14 (SCK)  ─────→ SD Card CLK   │
 │  GPIO15 (CS)   ─────→ SD Card CS    │
 │  3V3           ─────→ SD Card VCC   │
 │  GND           ─────→ SD Card GND   │
 └──────────────────────────────────────┘
```

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

EP1/EP2 書き込み配線（USB-シリアル変換, **必ず 3.3V**）:

```
 USB-シリアル変換              EP1/EP2 TCXO
 （CP2102 / CH340 等）         外部パッド
 ┌─────────────┐               ┌───────────┐
 │  3V3 ────────────────────→  VCC        │
 │  GND ────────────────────── GND        │
 │  TX  ────────────────────→  RX (GPIO3) │ ← ブートローダー兼用
 │  RX  ←──────────────────── TX (GPIO1) │
 └─────────────┘               └───────────┘
```

> **書き込みモードの入れ方**
> 1. RX パッド（GPIO3）を GND に短絡したまま VCC を接続
> 2. LED が点灯し続けたらブートローダーモード（UART 待ち受け中）
> 3. `pio run -e gate_ep1 -t upload` を実行
> 4. 完了後、GND 短絡を外してリセット

---

## TRSS（ドローンテレメトリ）の扱い

キャリブレーショングラフとラップ検出は、ドローンが送信機へ送り返す **テレメトリ uplink（TRSS）だけ** を見ています。

- ELRS 3.6.3 では OTA パケット種別の下位2ビットで種別が決まり、**ドローンのテレメトリは `0b11`（PACKET_TYPE_TLM）**。TX 側は RC=`0b00` / MSP=`0b01` / SYNC=`0b10` のみ。種別 `0b11` だけを計測対象にすることで、据え置きの TX の信号レベル・移動に左右されずドローンだけを追えます。
- テレメトリは比率（Ratio）が高いほど疎になります（間隔 = 分母 × 2ms：1:16=32ms … **1:128=256ms**）。1サンプルでも受信すれば値を保持（HOLD）し、比率に応じたサイレンスタイムアウト（1:64→約384ms、1:128→600ms）を過ぎたら floor に落とします。これにより疎なサンプルでもグラフが floor にチラつきません。
- SYNC パケットからはテレメトリ比率と UID 照合のみを読み取り、**FHSS ホップグリッドの再アンカーは行いません**（再アンカーはロックを壊すため）。
- **推奨比率**: ラップ計測の確実性重視なら **1:64**（高比率を取りつつ安定）。最高の安定が必要なら **1:2**。

---

## Web UI

**接続:** WiFi SSID `ESP-NOW-LT` (PASS: `esp-now-lt`) → ブラウザで `http://20.0.0.1`

### タブ構成

- **Race** — 3秒カウントダウン + レースタイマー（開始 / 停止 / クリア）、4列パイロットグリッド、ラップ表。停止 = 一時停止、再度開始でカウントダウン無しで再開。HSモード / 計測モードに対応
- **Config** — 機体スキャン（未登録のみ表示）、自動チャンネル割当、ロースター（最大20名）、グローバル設定（読み上げ・読み上げ速度・ラップモード・クールダウン）
- **Calib** — パイロットごとの RSSI 波形グラフ（Canvas）、Enter/Exit 閾値スライダー（800ms デバウンス後に自動保存）。**グラフはドローンの TRSS に追従**
- **SD** — SD カード内のレース CSV の一覧・ダウンロード（UTF-8 BOM 付き）・削除

### 主な調整パラメータ（Gate Node）

| パラメータ | デフォルト | 説明 |
|---|---|---|
| Enter 閾値 | -55 dBm | 通過開始の RSSI しきい値 |
| Exit 閾値 | -62 dBm | 通過終了の RSSI しきい値 |
| EMA_ALPHA | 0.25 | RSSI 平滑化係数（小さいほど滑らか/遅い） |
| クールダウン | 3000 ms | 最小ラップ間隔 |
| RSSI 報告間隔 | 50 ms | スニファー → ゲートの報告周期（20Hz） |

> Enter/Exit 閾値は **Calib タブからパイロットごとにリアルタイム調整**でき、即座に Gate Node へ反映されます。デフォルトは TX のバックグラウンド（-65〜-75dBm 程度）より十分高く設定し、登録直後に常時通過状態へ入らないようにしています。

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
