# ARCHITECTURE: EP スニファーによるラップ検出

## 信号の流れ

```
パイロットTX --[LoRa downlink, FHSS]--> ドローンのEP (標準ELRS RX)
                                            |
                                            | テレメトリ uplink (同じFHSSシーケンス)
                                            v
                                ゲートEP スニファー x4 (カスタムファーム)
                                            |  スロットごとのRSSI
                                            |  ESP-NOW
                                            v
                                ゲートESP32 (TTGO T8, 既存ロジック)
                                            |  UART JSON (変更なし)
                                            v
                                Web Node (XIAO S3, 変更なし)
                                            |  WiFi AP
                                            v
                                       スマホ / PC UI
```

## なぜ downlink ではなく uplink なのか

- downlink（TX -> RX）はパイロットの据え置き送信機から発信される。その RSSI を
  ゲートで測ってもドローンの位置を追えない。
- uplink（RX -> TX のテレメトリ）はドローンの EP から発信される。ドローンが
  ゲートに近づくと、ゲートスニファーが測る uplink の RSSI がピークを迎える。
  そのピークが通過の瞬間。

## なぜ EP をゲート受信機に使うのか

EP1/EP2 TCXO = ESP8285（ESP8266相当MCU）+ SX1280（2.4GHz LoRa）+ TCXO。

- ESP8285 は Arduino + ESP-NOW に対応するため、各スニファーが Gate ESP32 と
  無線で通信できる。これで 420000baud の CRSF UART 4本が不要になる。
- SX1280 は EP の内部 SPI で ESP8285 に接続されており、カスタムファームから
  直接制御できる（周波数設定・RXモード・RSSI読み取り）。
- TCXO により発振が安定し、FHSS のタイミングドリフトが小さく同期しやすい。

## FHSS 追従

ELRS はホップシーケンスを 6バイトのバインド UID から決定論的に導出する:

```
seed = UID[2] | UID[3]<<8 | UID[4]<<16 | UID[5]<<24
hopSequence = shuffle(0..N-1, rng(seed))   // N ~= 80 ch (2.4GHz)
```

UID が分かれば、ゲートスニファーは全ホップを事前計算し、次スロットの SX1280
周波数を先回りでセットできる。難しいのは「時間的同期」、つまり今が何番目の
スロットかを知ること。これは下記のロックオンフェーズで解決する。

## ロックオンフェーズ（スニファーごと）

```
1. SCAN:    hopSequence を短い RX ドウェルで素早く順に切り替え、
            あるチャンネル index k でパケット（または強RSSI）を検出するまで回す。
2. ESTIMATE: 検出時刻 t0 + チャンネル k から現在のスロット index と、
            ELRS フレームクロックに対するタイマーオフセットを算出する。
3. FOLLOW:  スロットごとにホップ index を進める。連続ミスが続いたら SCAN に戻す。
```

TCXO のおかげで FOLLOW フェーズは粘り強い。ドローンが遠い（連続ミスが多い）
ときだけ再 SCAN すればよい。ラップ検出は信号が強いゲート近傍でのみ重要なので、
遠方での一時的なミスは許容できる。

## RSSI -> ラップ（既存プロジェクトを再利用）

スニファーは生 RSSI を Gate ESP32 に報告する。Gate ESP32 はそれを既存の
パイプラインへそのまま流す（変更なし）:

```
生RSSI -> EMAフィルタ (alpha = 0.3) -> Enter/Exit 状態機械 -> sendLap()
```

状態機械（RotorHazard 方式）は `gate_node/main.cpp` に実装済み:

```
CLEAR --(ema > EnterAt)--> CROSSING
CROSSING: ピークとピーク時刻を追跡
CROSSING --(ema < ExitAt かつ クールダウン経過)--> sendLap(peakTime) -> CLEAR
```

デフォルト閾値: EnterAt -80dBm, ExitAt -90dBm, クールダウン 3000ms。
これらは Calib タブからパイロットごとに調整可能で、変更不要。

## タイミング予算（250Hz の例）

```
ELRS スロット @ 250Hz     約 4.0 ms
SX1280 周波数切替         約 1.0 ms
RX ドウェル + RSSI読み    残り約 3 ms に収まる   -> OK
```

高パケットレート（500Hz, 約2ms）では余裕が縮むので、Step 4 で実測検証する。
テレメトリ Ratio が uplink の頻度を下げるため、ゲートでの実効サンプルレート =
パケットレート / テレメトリRatio。使用中の Ratio を確認すること。

## 識別モデルの変更

旧: パイロットを Aircraft Node のハードウェア MAC で識別。
新: パイロットを 6バイトの ELRS バインド UID で識別。

- ロースターは MAC の代わりに UID を保存（テキスト書式 `AA:BB:CC:DD:EE:FF` は同じ）。
- ゲートスニファーは追従すべき UID を持たされる。
- Gate ESP32 は受信 ESP-NOW パケットを送信元 MAC ではなく UID でキーにする。

## 障害モードと対策

| モード | 対策 |
|--------|------|
| ドローン遠方で同期ロスト | 許容。ミス連続で再SCAN |
| 2.4GHz 混雑（WiFi等） | TCXO + FHSS の狭帯域ドウェル、閾値調整 |
| テレメトリOFF（TXでRaceモード） | テレメトリ有効を必須化、または downlink追従にフォールバック |
| UI での UID 打ち間違い | 6バイトhex書式を検証、オンライン/オフライン状態を表示 |
