# gamma ファームウェア

gamma 機体（beta の次世代機）のメインボード（`gamma_turret`、ESP32-P4）ファームウェア。

gamma は基板3枚セット（`gamma_turret` / `gamma_neck` / `gamma_chassis`）で構成されるが、
マイコンを載せるのは砲塔の `gamma_turret` のみ。本リポジトリはその制御ファームウェアを管理する。
`gamma_neck` / `gamma_chassis` はマイコン無しの受動分配基板（電源・CAN の分配のみ）。

ファームウェアは新規作成だが、通信・モータ・制御器の実装は共通ライブラリ
[rabcl](https://github.com/rabbits-robotics/rabcl) に寄せて育て、beta までの資産を使い回す方針。

> **ステータス凡例**
> - ✅ 確定・実装済み
> - 🔧 実装中
> - 💭 構想段階（変更の可能性あり）

---

## システム構成・データフロー

```
[ Jetson Orin Nano ]
  │ UART（3.3V TTL）
  ▼
[ gamma_turret（M5Stack Stamp-P4 / ESP32-P4） ]  ← BNO055 IMU（UART、基板直載せ）
  ├── CAN1（砲塔・ジンバル系）
  │     ├── DM3507  × 2 → 首 Roll / Pitch 軸
  │     └── MHF5005 × 1 → 首 Yaw 軸
  ├── CAN2（車体系）
  │     │   スリップリング経由で gamma_neck → gamma_chassis へ
  │     ├── CAN_CHASSIS 1〜4 → DM2325 × 4（オムニ 4 輪）
  │     └── CAN_LEG 1〜4     → DM4310 × 4（各車輪の持ち上げ機構）
  ├── PWM × 2 → ESC → 射出ローラー × 2
  └── UART（半二重、1kΩ 合流） → ST3215 サーボ（装填）
```

### 基板間接続

| 区間 | 内容 |
|---|---|
| gamma_turret ⇔ gamma_neck | 24V 幹線、CAN1（首モータへ）、CAN2（パススルー） |
| gamma_neck ⇔ gamma_chassis | スリップリング経由。24V、CAN2、ゲームモジュール線（PA 4P） |
| gamma_chassis → 各脚・車体機構 | 24V 分配、CAN2 分配（CAN_CHASSIS1〜4 / CAN_LEG1〜4） |

### 電源系統

| 電圧 | 生成元 | 供給先 |
|---|---|---|
| 24V | 幹線 | 首モータ3台、P4 キャリア、車体側 |
| 7.4V | 外部 DCDC（XT30 で往復） | ST3215 サーボ、ESC |
| 5V | M78AR05-1（1A、24V から） | Stamp-P4 |
| 3.3V | Stamp-P4 内蔵 DCDC | BNO055、CAN トランシーバ VIO、Jetson UART |

---

## 基板詳細

### gamma_turret（制御基板）

| 項目 | 内容 | 状態 |
|---|---|---|
| コア | M5Stack Stamp-P4（ESP32-P4、16MB Flash / 32MB PSRAM）、ピンソケットで差し替え | ✅ |
| CAN | 2ch（MCP2562FD × 2、VIO = 3.3V） | ✅ |
| ESC PWM | 2ch（射出ローラー） | ✅ |
| ST3215 UART | 半二重バス（1kΩ 合流回路） | ✅ |
| Jetson UART | 3.3V TTL | ✅ |
| IMU | BNO055 を基板直載せ、UART 接続 | ✅ |
| HMI | LED × 6（電源 / デバッグ / アクティビティ）、BOOT / RESET タクト SW | ✅ |
| 基板状態 | 設計確定済み。生基板到着、部品実装中 | 🔧 |

#### 受信: Jetson からの指令

<!-- TODO: プロトコル（rabcl の 28B/112B を踏襲するか、新規定義か）、ボーレート。別途相談 -->

#### アクチュエータ制御

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

#### CAN ID 一覧

<!-- TODO: CAN1 / CAN2 それぞれの ID 割り当て -->

| バス | 定数名 | TX ID | RX ID | 内容 |
|---|---|---|---|---|

#### ファームウェア申し送り（ハード設計からの制約）

- 当面 `CONFIG_FREERTOS_UNICORE=y`（デュアルコアハング回避）
- GPIO26 / GPIO27 の USB 機能を無効化
- UART0 ログ出力を無効化
- TWAI のループバック自己試験は使わない

### gamma_neck（受動分配基板）

- スリップリング（車体 ⇔ 砲塔）の受け口
- 24V 分配（首モータ3台 + P4 キャリア）
- CAN 分配（CAN1 → 首モータ、CAN2 → P4 へパススルー）
- ゲームモジュール線（PA 4P）パススルー
- CAN 終端抵抗は基板に載せず、モータ内蔵の切替終端で運用（バス最遠の1台のみ ON）
- 状態: 案A（純受動・最小構成）で設計確定。生基板到着、部品実装中 🔧

> ⚠️ リング電源入力と MHF5005 出力が同じ PH 6P コネクタ。挿し違えると 24V がモータの CAN ピンに入る。
> シルク表記と組立時確認で担保する。

### gamma_chassis（受動分配ハブ）

- 24V 入力（6P）と電源分配（Power1〜8 / Vout1）
- CAN2 分配: CAN_CHASSIS1〜4（DM2325 車輪）/ CAN_LEG1〜4（DM4310 脚）/ CAN_TURRET1
- 状態: 生基板到着、部品実装中 🔧

---

## beta からの変更点

| 項目 | beta | gamma | 状態 |
|---|---|---|---|
| マイコン | STM32F303K8（Nucleo） | **ESP32-P4（M5Stack Stamp-P4）** | ✅ |
| ボード構成 | 1 枚 | **3 枚（turret = 制御、neck / chassis = 受動分配）** | ✅ |
| 上位コントローラ | Raspberry Pi 5 | **Jetson Orin Nano** | ✅ |
| CAN | 1 系統（1 Mbps） | **2 系統（CAN1 = 砲塔・ジンバル、CAN2 = 車体）** | ✅ |
| 砲塔軸 | Yaw + Pitch の 2 軸 | **Yaw + Pitch + Roll の 3 軸ジンバル** | ✅ |
| Yaw 軸モータ | MHF5005（LK、トルク制御） | MHF5005（LK） | 🔧 |
| Pitch 軸モータ | DM3507（DAMIAO MIT モード） | DM3507（DAMIAO） | 🔧 |
| Roll 軸モータ | なし | **DM3507（DAMIAO）** | 🔧 |
| 車輪 | DM2325 × 4（CAN 速度制御、オムニ） | DM2325 × 4（CAN2、オムニ） | 💭 |
| 脚 | なし | **DM4310 × 4（CAN2）。アクティブダンパ・車高調整** | 💭 |
| 装填 | 市販 MD + DC モータ（PWM） | **ST3215 サーボ（半二重 UART）** | 💭 |
| 射出 | ESC × 2（PWM 50Hz） | ESC × 2（PWM 50Hz、パルス幅は beta 踏襲） | 💭 |
| IMU | BNO055（I2C、無効化中） | **BNO055（UART、基板直載せ）** | 💭 |
| 車体 ⇔ 砲塔接続 | スリップリング | スリップリング（継続） | ✅ |
| 開発環境 | PlatformIO + stm32pio（STM32Cube） | PlatformIO（ESP32-P4） | 🔧 |
| 共通ライブラリ | rabcl（サブモジュール） | rabcl（サブモジュール、継続して拡張） | ✅ |

---

## 共通仕様

### クロック・CAN ビットレート

<!-- TODO: TWAI のビットレート、制御周期 -->

| 項目 | 値 |
|---|---|

### モータ設定値・制御パラメータ

<!-- TODO -->

---

## 依存ライブラリ

[rabcl](https://github.com/rabbits-robotics/rabcl) を beta と同様にサブモジュールとして使用する。✅

```bash
git submodule update --init
```

rabcl は Rabbits の共通ライブラリとして継続して育てる方針。gamma で新たに必要になる以下は
gamma_turret_fw 側に閉じず rabcl に追加していく。

| 項目 | 内容 | 状態 |
|---|---|---|
| 既存資産 | UART パケット定義、CAN（LK MHF5005 / DAMIAO DM2325・DM3507 MIT）、`OmniDrive`、`PdGravityFf`、BNO055 | ✅ |
| DM4310 | 脚モータ（DAMIAO）。既存の DAMIAO 実装を流用できる見込み | 💭 |
| ST3215 | 装填サーボ（半二重 UART プロトコル） | 💭 |
| BNO055 UART | 既存は I2C 前提。UART モード対応を追加 | 💭 |
| ESP32 向け HAL 層 | TWAI（CAN）/ UART / LEDC（PWM）の抽象化。rabcl 側はプラットフォーム非依存に保つ | 💭 |

---

## ビルド方法

beta に引き続き PlatformIO を使用する。開発は VS Code + PlatformIO 拡張機能で完結させる。✅

<!-- TODO: ESP32-P4 対応の platform / framework 選定、board 定義、sdkconfig、書き込み手順 -->

| 項目 | 内容 | 状態 |
|---|---|---|
| エディタ | VS Code + PlatformIO IDE 拡張機能 | ✅ |
| ビルドシステム | PlatformIO | ✅ |
| platform / framework | 未定（ESP32-P4 対応の platform を選定中） | 💭 |
| ターゲット | M5Stack Stamp-P4（ESP32-P4） | ✅ |

```bash
# ビルド
pio run

# 書き込み
pio run --target upload
```
