# HANDOFF: ESP-NOW-LT + ELRS EP1/EP2 (Gate Sniffer Edition)

> Claude Code（Windows / PlatformIO）向けオンボーディング文書。
> まずこれを読み、次に `docs/ARCHITECTURE.md`、それから下の Step 1 から着手。

## 概要（TL;DR）

既存の **ESP-NOW-Lap-Timer** リポジトリを拡張し、専用の XIAO ESP32-C3 ビーコン
（Aircraft Node）の代わりに、ドローン搭載の **HappyModel EP1/EP2 TCXO** が出す
**ELRS テレメトリ uplink** をラップ検出に使う。

ゲートノードは、ドローン通過時に EP の 2.4GHz 信号の RSSI ピークを検出する。
重要なのは、ゲート側の受信機も **EP1/EP2 TCXO** に「スニファー」用カスタム
ファームを焼いたもの（ESP8285 + SX1280）である点。これにより、別途 SX1280
ブレイクアウト基板も、420000baud の UART x4 問題も不要になる。

## 変更しない部分（書き換え禁止）

- `src/web_node/` 全体（UI / レース制御 / NVS / SD）
- `src/gate_node/` の UART プロトコル（`uart_gate.*`）、SD（`sd_gate.*`）
- `src/gate_node/` の EMA 状態機械（`main.cpp` の Enter/Exit 閾値）
- Gate -> Web の UART JSON プロトコル
- `data/` の Web UI（Config タブの MAC -> UID ラベル変更を除く）

## 新規 / 変更する部分

| コンポーネント | 状態 | 場所 |
|----------------|------|------|
| Gate EP スニファーファーム | 新規 | `src/gate_ep1/` |
| Gate Node の ESP-NOW 受信 | 変更（MACキー -> UIDキー） | `src/gate_node/promiscuous.*` |
| パイロット識別 | 変更（MAC -> 6バイトUID） | `src/gate_node/pilots.*`, web roster |
| Aircraft Node（XIAO C3 ビーコン） | 削除 | 今後は焼かない |
| platformio env `gate_ep1` | 新規 | `platformio.ini` |

## ハードウェアの役割

```
[ドローン] EP1/EP2 TCXO (標準ELRSファーム, パイロットTXにバインド) -> 通常のRC RX
[ゲート]   EP1/EP2 TCXO x4 (カスタムスニファーファーム) -> FHSS同期 + RSSI -> ESP-NOW
[ゲート]   ESP32-WROVER (TTGO T8) -> 既存gate_nodeロジック, ESP-NOW受信
[ピット]   XIAO ESP32-S3 -> 既存web_node, WiFi AP + UI
```

## なぜテレメトリ uplink を使うのか

ドローンの EP は、パイロットの TX へテレメトリパケットを送り返す。このパケットは
移動するドローンから発信されるため、同期したゲート受信機は通過の瞬間に RSSI の
ピークを観測できる。詳しい理屈は `docs/ARCHITECTURE.md` を参照。

## パイロットごとの設定

各パイロットの TX はバインド UID（6バイト）を持つ。ゲートスニファーは FHSS ホップ
シーケンスを計算するために各 UID を知る必要がある。UID は Web UI の Config タブで
一度だけ入力し（旧 MAC 欄を置き換え）、ゲートスニファーへ配布する。

## セキュリティ / リポジトリ運用

- 実際のバインド UID、バインドフレーズ、WiFi パスワード、各種キーはコミットしない。
- `src/gate_ep1/secrets.h` は gitignore 対象にする。テンプレートとして
  `secrets.example.h` をコミット済み。push 前に実 UID/フレーズが staged に
  含まれていないか必ず確認すること。
- web_node の既存 AP パスワードは既知のデフォルト値。秘匿情報ではないが、
  変更する場合はその旨を明示する。

## ビルド / 書き込みクイックスタート

```
# Gate sniffer (EP1/EP2 TCXO = ESP8285) - パイロットスロットごとに4台焼く
pio run -e gate_ep1 -t upload

# Gate Node (ESP32-WROVER / TTGO T8)
pio run -e gate_node -t upload

# Web Node (XIAO ESP32-S3)
pio run -e web_node -t upload
pio run -e web_node -t uploadfs   # JS/HTML 変更後に必要
```

EP 書き込み配線（USB-シリアル変換, 3.3V）:

```
EP pad    変換アダプタ
TX    ->  RX
RX    ->  TX
GND   ->  GND
VCC   ->  3V3
```

> ブート方法: 別途 GPIO0 テストポイントは不要。RX パッド（GPIO3）を LOW に
> 引いたまま電源投入すると、ESP8285 が UART ブートローダーに入る。
> TX オフなのに LED が点灯しっぱなしなら、ブートローダーモードに入っている合図。

## 開発ステップ（この順で進める）

1. **書き込み確認** - `gate_ep1` env がビルド・書き込みできることを確認。
   EP 1台に空スケッチを焼き、USB シリアル出力を確認する。
2. **SX1280 SPI 疎通** - 固定周波数でチップステータス + 生 RSSI を読む。
   `src/gate_ep1/sx1280_sniffer.cpp` の TODO 参照。
3. **FHSS シーケンス** - ELRS から `generateFHSSsequence` を移植し、既知 UID で
   単体テスト。`src/gate_ep1/fhss.cpp` の TODO 参照。
4. **ロックオン / 同期** - シーケンスをスキャンし、最初のパケットを検出して
   スロットオフセットを計算、以降ホップに追従する。
5. **ESP-NOW 送信** - `GateEP1Packet` を Gate Node の MAC へ送る。
6. **Gate Node 統合** - `promiscuous.*` を UID キーに変更し、既存の
   `processRSSI()` を呼ぶ。`docs/gate_esp32_changes.md` 参照。
7. **4台同時テスト** - スニファー4台・パイロット4機でラップ検出を通しで検証。
8. **Web UI** - Config タブの MAC -> UID 表記変更。ロースターの往復を確認。

## 参考リポジトリ

- ExpressLRS（FHSS, SX1280ドライバ）: https://github.com/ExpressLRS/ExpressLRS
- 本プロジェクトのベース: https://github.com/yanazoo/ESP-NOW-Lap-Timer
- PhobosLT（4ch SPI構成の参考）: https://github.com/yanazoo/PhobosLT_4ch

## 未解決事項

解決済み:
- GPIO0 テストポイント -> 不要。RX パッド（GPIO3）を LOW にして電源投入すると
  ESP8285 の UART ブートローダーに入る。5V/GND/RX/TX で esptool 書き込み。
- SX1280 SPI ピン -> ELRS の generic ESP8285 2.4GHz RX レイアウトから確定
  （EP1/EP2 はこのリファレンスピン配置を共有）。値は config.h に記載:
  NSS=15 SCK=14 MOSI=13 MISO=12 BUSY=5 DIO1=4 RST=2 RX=3 TX=1 LED=16。
  EP1 と EP2 は同一 PCB（アンテナのみ違い）なのでどちらでも可。ゲート用途では
  EP1 の U.FL が指向性アンテナを付けられて有利。

未解決:
- 使用するテレメトリ Ratio（パケット周期 / 同期ウィンドウに影響）。
- スニファー以外の EP 向けに CRSF-UART フォールバック経路を残すか。
- バッチで焼く前に、現物でピン配置を裏取りすること。
