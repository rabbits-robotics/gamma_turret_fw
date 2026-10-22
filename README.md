# gamma_turret_fw

gamma_turret 基板（M5Stack Stamp-P4 / ESP32-P4）のファームウェア。
上位コントローラ（Jetson Orin Nano）からの UART 指令を受け、CAN 2 系統で首・車体・脚のモータを、PWM で射出 ESC を、半二重 UART で装填サーボを制御する。
通信・モータ・制御器の実装は共通ライブラリ [rabcl](https://github.com/rabbits-robotics/rabcl) に寄せて育て、beta までの資産を使い回す。

> **ステータス凡例**
> - ✅ 確定・実装済み
> - 🔧 実装中
> - 💭 構想段階（変更の可能性あり）

## システム構成

```
[ Jetson Orin Nano ]
  │ UART（3.3V TTL）
  ▼
[ gamma_turret（Stamp-P4 / ESP32-P4） ]  ← BNO055 IMU（UART）
  ├── CAN1 → DM3507 × 2（首 Roll / Pitch）、MHF5005（首 Yaw）
  ├── CAN2 → DM2325 × 4（オムニ 4 輪）、DM4310 × 4（脚）
  ├── PWM × 2 → ESC → 射出ローラー × 2
  └── UART（半二重） → ST3215 サーボ（装填）
```

CAN2 はスリップリング経由で車体側の受動分配基板へ届く。

## I/O

| 対象 | ペリフェラル | 内容 | 状態 |
|---|---|---|---|
| 上位コントローラ | UART（3.3V TTL） | Jetson からの指令受信、フィードバック送信 | ✅ |
| CAN1 | TWAI、MCP2562FD | 砲塔・ジンバル系（首 3 軸） | ✅ |
| CAN2 | TWAI、MCP2562FD | 車体系（車輪 4 輪、脚 4 本） | ✅ |
| 射出 ESC × 2 | LEDC（PWM）、50Hz | 待機 1000µs / 射出 1200µs | ✅ |
| 装填サーボ | UART（半二重、1kΩ 合流） | ST3215 | ✅ |
| IMU | UART | BNO055（基板直載せ） | ✅ |
| HMI | GPIO | LED × 6（電源 / デバッグ / アクティビティ）、BOOT / RESET タクト SW | ✅ |

### 受信: Jetson からの指令

<!-- TODO: プロトコル（rabcl の 28B/112B を踏襲するか、新規定義か）、ボーレート。別途相談 -->

### アクチュエータ制御

| アクチュエータ | 通信 | 内容 | 状態 |
|---|---|---|---|
| 首 Yaw 軸 | CAN1（LK モータ） | MHF5005 | 💭 |
| 首 Pitch 軸 | CAN1（DAMIAO） | DM3507 | 💭 |
| 首 Roll 軸 | CAN1（DAMIAO） | DM3507 | 💭 |
| 車体（CAN_CHASSIS 1〜4） | CAN2（DAMIAO） | DM2325 × 4、オムニ 4 輪（速度制御、`rabcl::OmniDrive`） | 💭 |
| 脚（CAN_LEG 1〜4） | CAN2（DAMIAO） | DM4310 × 4、各車輪の持ち上げ機構。アクティブダンパ・車高調整 | 💭 |
| 射出ローラー × 2 | PWM → ESC | 50Hz サーボパルス。待機 1000µs / 射出 1200µs（beta 実測値を踏襲） | 💭 |
| 装填 | 半二重 UART | ST3215 サーボ | 💭 |

射出ローラーの ESC は beta と同じく起動時にキャリブレーションを行う（2000µs → 1000µs、各 3 秒待機）。
射出時のパルス幅 1200µs は beta の実機調整値（2025-03 時点）。gamma でも実機で再調整する。

脚（DM4310）は各車輪を独立に持ち上げる機構を駆動し、以下の 2 機能を担う。

| 機能 | 内容 | 状態 |
|---|---|---|
| アクティブダンパ | 路面からの衝撃を脚側で吸収 | 💭 |
| 車高調整 | 4 脚の持ち上げ量で車高を変える | 💭 |
| 制御モード | DM4310 の MIT モードを想定。IMU やモータ FB 値を使ったトルク制御も候補（要相談） | 💭 |

<!-- TODO: IMU との連携、Jetson からの指令形式 -->

### CAN ID 一覧

DAMIAO のモータは CAN_ID を 16 未満にする（説明書の推奨）。フィードバックの先頭バイトが「CAN_ID の下位 8 bit | 状態コード << 4」なので、CAN_ID が 16 以上だと状態コード（有効 / 無効 / 過電圧などの異常）と重なって読めなくなる（2026-10-22 に DM4310 で確認: CAN_ID 0x47 だと D[0] = 0x57）。beta の ID（0x42〜0x46）は引き継がず、流用するモータも DMTool で書き直す。定数は `src/board.hpp`。

| バス | 部位 | モータ | モード | CAN_ID（指令） | 指令フレーム ID | MST_ID（FB） | 状態 |
|---|---|---|---|---|---|---|---|
| CAN1 | 首 Yaw | MHF5005（LK） | 位置 | `0x141` | `0x141` | `0x141` | beta と同じ |
| CAN1 | 首 Pitch | DM3507 | MIT | `0x01` | `0x01` | `0x11` | beta から変更 |
| CAN1 | 首 Roll | DM3507 | MIT | `0x02` | `0x02` | `0x12` | 新規 |
| CAN2 | 前右輪 | DM2325 | 速度 | `0x03` | `0x203` | `0x13` | beta から変更 |
| CAN2 | 前左輪 | DM2325 | 速度 | `0x04` | `0x204` | `0x14` | beta から変更 |
| CAN2 | 後右輪 | DM2325 | 速度 | `0x05` | `0x205` | `0x15` | beta から変更 |
| CAN2 | 後左輪 | DM2325 | 速度 | `0x06` | `0x206` | `0x16` | beta から変更 |
| CAN2 | 前右脚 | DM4310 | MIT | `0x07` | `0x07` | `0x17` | 新規 |
| CAN2 | 前左脚 | DM4310 | MIT | `0x08` | `0x08` | `0x18` | 新規 |
| CAN2 | 後右脚 | DM4310 | MIT | `0x09` | `0x09` | `0x19` | 新規 |
| CAN2 | 後左脚 | DM4310 | MIT | `0x0A` | `0x0A` | `0x1A` | 新規 |

- DAMIAO の規則: MIT モードの指令フレーム ID = CAN_ID、速度モードの指令フレーム ID = `0x200 + CAN_ID`、フィードバックは MST_ID（= CAN_ID + 0x10）で返る。CAN_ID と MST_ID は DMTool（DAMIAO Debugging Assistant）で各モータに書き込む
- DM AC-Tools V1.8 での書き込み（2026-10-22 DM4310 で確認）: 左の「USB模式」を **UART** にした状態で Set parameters の「读参数 / 写参数」を使うと保存される。USB模式が CAN だと 写参数 は RAM に書くだけで、電源を切ると元に戻る。書き込み後は電源を入れ直し、Serial port タブの Debug Info（`CAN ID` / `MASTER ID`）で確認する。Test / Parse タブの P_MAX / V_MAX / T_MAX（12.5 / 45 / 18）はツールの初期値で、モータの設定（DM4310: 12.5 / 30 / 10）とは別
- LK（MHF5005）は `0x140 + モータ ID`。モータ ID = 1 なので `0x141`。送受信とも同じ ID
- 同じバスの中で指令フレーム ID と MST_ID がすべて異なるようにしてある
- 書き込んだら実フレームの ID をファームのログで確認する（beta の DM3507 では DMTool 上の表示 `0x142` に対して実フレームは `0x42` だった）

### ハード設計由来の制約

- 当面 `CONFIG_FREERTOS_UNICORE=y`（デュアルコアハング回避）
- GPIO26 / GPIO27 の USB 機能を無効化
- UART0 ログ出力を無効化
- TWAI のループバック自己試験は使わない

## beta_main からの変更点（ファーム関連）

| 項目 | beta_main | gamma_turret_fw | 状態 |
|---|---|---|---|
| マイコン | STM32F303K8（Nucleo） | **ESP32-P4（M5Stack Stamp-P4）** | ✅ |
| CAN | 1 系統（1 Mbps） | **2 系統（CAN1 = 砲塔・ジンバル、CAN2 = 車体）** | ✅ |
| 砲塔軸 | Yaw + Pitch | **Yaw + Pitch + Roll** | ✅ |
| 脚 | なし | **DM4310 × 4** | 💭 |
| 装填 | 市販 MD + DC モータ（PWM） | **ST3215 サーボ（半二重 UART）** | 💭 |
| IMU | BNO055（I2C、無効化中） | **BNO055（UART）** | 💭 |
| 開発環境 | PlatformIO + stm32pio（STM32Cube） | PlatformIO（ESP32-P4） | 🔧 |
| 共通ライブラリ | rabcl（サブモジュール） | rabcl（サブモジュール、継続して拡張） | ✅ |

## 共通仕様

### クロック・CAN ビットレート

<!-- TODO: TWAI のビットレート、制御周期 -->

| 項目 | 値 |
|---|---|

### モータ設定値・制御パラメータ

<!-- TODO -->

## 依存ライブラリ

| ライブラリ | 置き場 | 役割 |
|---|---|---|
| [rabcl](https://github.com/rabbits-robotics/rabcl) | `components/rabcl`（submodule） | プロトコル・制御。beta と同じくコミット固定 ✅ |
| [rabcl_esp32](https://github.com/rabbits-robotics/rabcl_esp32) | `components/rabcl_esp32`（submodule） | ESP-IDF のドライバのラッパー（`CanBus`、`DebugLed`） ✅ |

```bash
git submodule update --init
```

基板のピン割り当てと定数は `src/board.hpp` にまとめる。周辺機器の扱いは rabcl_esp32 に寄せ、ここには機体の制御・通信だけを置く。

rabcl は Rabbits の共通ライブラリとして継続して育てる方針。gamma で新たに必要になる以下は
gamma_turret_fw 側に閉じず rabcl に追加していく。

| 項目 | 内容 | 状態 |
|---|---|---|
| 既存資産 | UART パケット定義、CAN（LK MHF5005 / DAMIAO DM2325・DM3507 MIT）、`OmniDrive`、`PdGravityFf`、BNO055 | ✅ |
| DM4310 | 脚モータ（DAMIAO）。既存の DAMIAO 実装を流用できる見込み | 💭 |
| ST3215 | 装填サーボ（半二重 UART プロトコル） | 💭 |
| BNO055 UART | 既存は I2C 前提。UART モード対応を追加 | 💭 |
| ESP32 向け HAL 層 | TWAI（CAN）/ UART / LEDC（PWM）の抽象化。rabcl 側はプラットフォーム非依存に保つ | 💭 |

## 開発環境

beta に引き続き PlatformIO を使用する。開発は VS Code + PlatformIO 拡張機能で完結させる。✅

| 項目 | 内容 | 状態 |
|---|---|---|
| エディタ | VS Code + PlatformIO IDE 拡張機能 | ✅ |
| platform | [pioarduino](https://github.com/pioarduino/platform-espressif32) 55.03.312（公式の espressif32 は ESP32-P4 未対応） | ✅ |
| framework | espidf（ESP-IDF 5.5.5）。設計上の制約が sdkconfig で決まるため Arduino は使わない | ✅ |
| ターゲット | M5Stack Stamp-P4（ESP32-P4 v1.3、16MB Flash）。ボード定義は `boards/m5stack-stamp-p4.json` | ✅ |
| sdkconfig | `sdkconfig.defaults` が正。生成される `sdkconfig.stamp_p4` は追跡しない | ✅ |

注意点:

- pioarduino の platform は、依存として pioarduino 版の PlatformIO Core（6.2.0）を入れ、公式 Core を置き換える。platform と Core の版は対で使う（beta_main などの STM32 ビルドは Core 6.2.0 でも動く）
- ビルドは VS Code か PowerShell から行う。Git Bash（MSYS）から実行するとツールのインストーラが「MSys/Mingw は非対応」で止まる
- 手元の Stamp-P4 は ESP32-P4 **v1.3**。ESP-IDF の既定は rev 3.01 以上向けなので、`sdkconfig.defaults` で rev 3 未満を選んでいる（CPU は 360 MHz）。この設定を変えたときは `.pio/build` を消してクリーンビルドする

## ビルド・書き込み

Stamp-P4 の USB-C を PC につなぐ（USB-Serial-JTAG として COM ポートに見える）。24V を入れたままでも、USB だけでも書き込める。

```bash
# ビルド
pio run

# 書き込み（ポートは環境に合わせる）
pio run --target upload --upload-port COM3

# コンソール（USB-Serial-JTAG、115200 bps）
pio device monitor --port COM3
```

## ライセンス

MIT。[LICENSE](./LICENSE) を参照。
