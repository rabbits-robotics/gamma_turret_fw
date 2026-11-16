# gamma_turret_fw

gamma_turret 基板（M5Stack Stamp-P4 / ESP32-P4）のファームウェア。
上位 PC（現在は Raspberry Pi 5）からの UART 指令を受け、CAN 2 系統で首・車体・脚のモータを、PWM で射出 ESC を、半二重 UART で装填サーボを制御する。
通信・モータ・制御器の実装は共通ライブラリ [rabcl](https://github.com/rabbits-robotics/rabcl) に寄せて育て、beta までの資産を使い回す。

> **ステータス凡例**
> - ✅ 確定・実装済み
> - 🔧 実装中
> - 💭 構想段階（変更の可能性あり）

## システム構成

```
[ 上位 PC（Raspberry Pi 5） ]
  │ UART（USB-UART 変換 AE-FT234X 経由、3.3V TTL、921600 bps）
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
| 上位コントローラ | UART（3.3V TTL） | PC からの指令受信、フィードバック送信 | ✅ |
| CAN1 | TWAI、MCP2562FD | 砲塔・ジンバル系（首 3 軸） | ✅ |
| CAN2 | TWAI、MCP2562FD | 車体系（車輪 4 輪、脚 4 本） | ✅ |
| 射出 ESC × 2 | LEDC（PWM）、50Hz | 待機 1000µs / 射出 1200µs | ✅ |
| 装填サーボ | UART（半二重、1kΩ 合流） | ST3215 | ✅ |
| IMU | UART | BNO055（基板直載せ、UART モード）。`ImuTask`（rabcl の `BNO055UartDriver`）、100 Hz。実機確認済み（2026-11-01） | ✅ |
| HMI | GPIO | LED × 6（電源 / デバッグ / アクティビティ）、BOOT / RESET タクト SW | ✅ |

### 受信: PC からの指令

rabcl の共通プロトコル（下記「PC との通信」）。`CHASSIS_VEL`（視点基準の車体速度）、`BODY_POSE`（車高・ロール・ピッチ、カメラ座標系）、`JOINT_CMD`（首の視点目標）、`MODE`（load / fire / speed / chassis / stabilize）を受ける。操作の仕様は robot_ws/gamma の README「操作と制御の仕様」。

### アクチュエータ制御

| アクチュエータ | 通信 | 内容 | 状態 |
|---|---|---|---|
| 首 Yaw 軸 | CAN1（LK モータ） | MHF5005。視点の安定化（`Gimbal` → 目標を `YAW_RAMP_RATE` で寄せる → `PdGravityFf` → トルク指令、1000 Hz）。単体で追従と向きを確認（2026-11-15）、ゲインとゼロ点は機体に載せてから | 🔧 |
| 首 Pitch 軸 | CAN1（DAMIAO） | DM3507。`Gimbal` の pitch を可動角（−15°〜+30°）に収めて MIT 位置指令（1000 Hz）。単体で追従と向きを確認（2026-11-15）、ゼロ点（DMTool の原点リセット）は機体に載せてから | 🔧 |
| 首 Roll 軸 | — | モータは無い（関節番号だけ残す。PC 側の `roll_joint` と対） | — |
| 車体（CAN_CHASSIS 1〜4） | CAN2（DAMIAO） | DM2325 × 4、オムニ 4 輪（速度制御、`rabcl::OmniDrive`、視点基準 + 無限回転、各 250 Hz）。2026-11-16 浮かせて単体試験 OK、`WHEEL_SIGN` +1 で向き確定。CAN ID は beta のまま（組立後に変更） | ✅ |
| 脚（CAN_LEG 1〜4） | CAN2（DAMIAO） | DM4310 × 4、各車輪の持ち上げ機構。`BodyPosture` → `LegDamper`（MIT、各 250 Hz） | 🔧 |
| 射出ローラー × 2 | PWM → ESC | 50Hz サーボパルス。待機 1000µs / 射出 1200µs（beta 実測値を踏襲） | 💭 |
| 装填 | 半二重 UART | ST3215 サーボ（rabcl の `St3215`、ステップモード）。試験モードで単体確認中、制御本体への組み込みはこれから | 🔧 |

射出ローラーの ESC は beta と同じく起動時にキャリブレーションを行う（2000µs → 1000µs、各 3 秒待機）。
射出時のパルス幅 1200µs は beta の実機調整値（2025-03 時点）。gamma でも実機で再調整する。

脚（DM4310）は各車輪を独立に持ち上げる機構を駆動し、以下の 2 機能を担う。

| 機能 | 内容 | 状態 |
|---|---|---|
| アクティブダンパ | 弱い kp / kd の MIT 指令 + 荷重・摩擦分の tff（`rabcl::LegDamper`） | 🔧 |
| 車高調整 | `BODY_POSE` の車高 0〜1 とロール・ピッチ（カメラ座標系）を `rabcl::BodyPosture` で 4 脚の目標角に変換 | 🔧 |
| 安全策 | トルク / ロータ温度の上限超過でその脚にディセーブルを送り続け `error_flags` に立てる（復帰は再起動） | 🔧 |

制御の構成は下記「制御の構成」。

### CAN ID 一覧

DAMIAO のフィードバックの先頭バイトは「CAN_ID の下位 8 bit | 状態コード << 4」なので、CAN_ID が 16 以上だと状態コード（有効 / 無効 / 異常）に CAN_ID の bit4〜7 が混ざる（2026-10-22 に DM4310 で確認: CAN_ID 0x47 だと D[0] = 0x57）。beta から流用する首 Pitch と車輪は当面 beta の ID（0x42〜0x46）のまま使い、ファームが CAN_ID の分（bit2）を落として読む（`CanReceiver::HandleFrame`）。0 / 1（停止 / 有効）と異常（8 以上）の判定はできるが、異常コードの bit2 が消えるので **9（低電圧）と D（通信途絶）、8（過電圧）と C（ロータ過熱）が区別できない**。2026-11-15 の単体試験で Pitch の「9」が実は D だった件で時間を取られたので、**メカ組み立て後に原点リセットと一緒に DMTool で gamma の ID（Pitch `0x01` / `0x11`、車輪 `0x03`〜`0x06` / `0x13`〜`0x16`）へ書き換える**（ユーザー決定 2026-11-15）。定数は `src/board.hpp`。

| バス | 部位 | モータ | モード | CAN_ID（指令） | 指令フレーム ID | MST_ID（FB） | 状態 |
|---|---|---|---|---|---|---|---|
| CAN1 | 首 Yaw | MHF5005（LK） | トルク（0xA1） | `0x141` | `0x141` | `0x141` | beta と同じ |
| CAN1 | 首 Pitch | DM3507 | MIT | `0x42` | `0x42` | `0x52` | beta と同じ |
| CAN1 | 首 Roll | （モータ無し） | — | `0x02` | — | `0x12` | 送らない。番号だけ予約 |
| CAN2 | 前右輪 | DM2325 | 速度 | `0x43` | `0x243` | `0x53` | beta と同じ |
| CAN2 | 前左輪 | DM2325 | 速度 | `0x44` | `0x244` | `0x54` | beta と同じ |
| CAN2 | 後右輪 | DM2325 | 速度 | `0x45` | `0x245` | `0x55` | beta と同じ |
| CAN2 | 後左輪 | DM2325 | 速度 | `0x46` | `0x246` | `0x56` | beta と同じ |
| CAN2 | 前右脚 | DM4310 | MIT | `0x07` | `0x07` | `0x17` | 新規 |
| CAN2 | 前左脚 | DM4310 | MIT | `0x08` | `0x08` | `0x18` | 新規 |
| CAN2 | 後右脚 | DM4310 | MIT | `0x09` | `0x09` | `0x19` | 新規 |
| CAN2 | 後左脚 | DM4310 | MIT | `0x0A` | `0x0A` | `0x1A` | 新規 |

- DAMIAO の規則: MIT モードの指令フレーム ID = CAN_ID、速度モードの指令フレーム ID = `0x200 + CAN_ID`、フィードバックは MST_ID（= CAN_ID + 0x10）で返る。CAN_ID と MST_ID は DMTool（DAMIAO Debugging Assistant）で各モータに書き込む
- DM AC-Tools V1.8 での書き込み（2026-10-22 DM4310 で確認）: 左の「USB模式」を **UART** にした状態で Set parameters の「读参数 / 写参数」を使うと保存される。USB模式が CAN だと 写参数 は RAM に書くだけで、電源を切ると元に戻る。書き込み後は電源を入れ直し、Serial port タブの Debug Info（`CAN ID` / `MASTER ID`）で確認する。Test / Parse タブの P_MAX / V_MAX / T_MAX（12.5 / 45 / 18）はツールの初期値で、モータの設定（DM4310: 12.5 / 30 / 10）とは別
- LK（MHF5005）は `0x140 + モータ ID`。モータ ID = 1 なので `0x141`。送受信とも同じ ID
- 同じバスの中で指令フレーム ID と MST_ID がすべて異なるようにしてある
- 書き込んだら実フレームの ID をファームのログで確認する（beta の DM3507 では DMTool 上の表示 `0x142` に対して実フレームは `0x42` だった）

### CAN の通信周期（仕様と実測）

仕様: 各モータとの指令 / フィードバックの往復を **CAN2 は 250 Hz、CAN1 は 1000 Hz** で回す。指令はまとめて出さず等間隔に回す（CAN2 は 0.5 ms ごとに 1 台、CAN1 も 0.5 ms ごとに 1 台で 2 台）。CAN1 は Roll のモータが無くなって 2 関節になったので 1000 Hz に上げた（2026-11-15。MHF5005 と DM3507 とも毎秒 1000 返す）。

| バス | モータ | フレーム数 | バス負荷（1 Mbps、1 フレーム約 130 µs） |
|---|---|---|---|
| CAN2 | 8 台 × 250 Hz × 2 | 4,000 / 秒 | 約 52% |
| CAN1 | 3 台 × 500 Hz × 2 | 3,000 / 秒 | 約 39% |

実測（2026-10-22、DM4310 1 台、MIT 指令への応答。commit 8a07f66 の `src/main.cpp`、`RATE_SWEEP`）: 100 / 200 / 500 / 1000 / 2000 Hz のいずれも損失 0、応答遅延 259〜277 µs（指令と応答のフレーム 2 枚分）。2 kHz = 4,000 フレーム/秒で CAN2 の仕様と同じ量。

実測（2026-10-23、脚 4 本を 1 ms ごとに 1 本ずつ = 各脚 250 Hz、`src/main.cpp`）: 4 本とも毎秒 250 送信 / 250 受信、送信失敗 0、CAN2 のエラー 0。CAN1（2026-11-15、首 Yaw + Pitch を単体で接続、脚 4 本と同時）: 2 台とも毎秒 1000 送信 / 1000 受信、送信失敗 0。両バス同時で合計約 8,000 フレーム/秒、取りこぼし無し。

### 脚（DM4310）の実測（2026-10-22〜23、`src/main.cpp` の `HOLD_POSITION`。4 本とも ID 設定・原点リセット・配線済み）

- 可動域: 原点（SaveZero、メカスト）から約 1.6 rad。中点 0.8 rad
- MIT モードで中点保持（指令 250 Hz、esp_timer で等間隔）: kp 3.0 / kd 0.3 では 0.805 rad で静止し、0.27 rad 押すと 0.7 Nm で戻る。kp 1.0 / kd 0.1（現在の値）では押したあと 0.2〜0.3 rad ずれたまま止まる（戻りトルク 0.25 Nm が摩擦と自重に負ける）
- 脚機構の摩擦 + 自重は出力軸で **0.2〜0.3 Nm** 程度。4 点接地（柔らかい脚）は「弱い kp + 荷重分と摩擦分のフィードフォワード `tff`」で設計する
- 4 本同時の中点保持（kp 1.0 / kd 0.1、各脚 250 Hz）: 原点から 0.8 rad へ約 1.6 秒で移動し、FL 0.84 / BR 0.81 / BL 0.77 rad で静止（kp 1.0 の定常偏差）。FR は 0.96 rad から戻らず（同上）
- main タスクのスタックは 8192 B（`sdkconfig.defaults`）。既定の 3584 B では引数の多い ESP_LOGI で Stack protection fault

### PC との通信

[rabcl](https://github.com/rabbits-robotics/rabcl) の共通プロトコル（`rabcl::protocol`、設計書は rabcl の `docs/superpowers/specs/2026-10-22-robot-protocol-design.md`）を UART2（G26 TX / G27 RX、921600 bps）で流す。

| ファイル | 役割 |
|---|---|
| `src/robot.hpp` | 機体定義。関節番号（`robot::Joint`、rabbits_bridge の `config/gamma.yaml` の `joints` と同じ並び）、モード番号、関節とモータ / CAN ID の対応表 |
| `src/comm.hpp` / `src/comm.cpp` | `Comm`。送信タスク（持っている関節の `JOINT_STATE` を 100 Hz、`Controller::Tick` が集めた「そのティックにモータへ送った指令」を `MOTOR_COMMAND` で 100 Hz（PC 側では bridge の `motor_commands` → rviz の Command 表示）、IMU は値があれば 100 Hz、`INFO` / `SYSTEM`（`error_flags` 入り）を 1 Hz）と受信タスク（`CHASSIS_VEL` / `BODY_POSE` / `JOINT_CMD` / `MODE` を共有状態に置き、制御側が `Get*` で読む。最終受信時刻を途絶監視に使う） |
| `src/board.hpp` | `PC_UART_*`（ポート、ピン、ボーレート） |

PC 側は rabbits_bridge の `robot_bridge`（`config/gamma.yaml`）。関節状態の `effort` は DAMIAO がトルク [Nm]、LK が電流 [A]。

実機確認（2026-11-01、AE-FT234X を PC1 に接続、24V なし）: 上り `INFO`（version 1、joint_count 11、"gamma"）/ `SYSTEM` 1 Hz と `IMU` 100 Hz、下り `BODY_POSE` 100 Hz × 10 秒で 1000 / 1000 受信、CRC エラー 0、送信停止 0.5 s で `LOST`。usbipd（`usbipd bind --busid <FT234X>` を管理者で 1 回 → `usbipd attach --wsl --busid <FT234X>`。WSL 2 ディストロを 1 つ起動しておく）で Dev Container に `/dev/ttyUSB0` として渡し、`device: /dev/ttyUSB0` にした YAML で `robot_bridge` を起動すると `imu/data` が 100 Hz、`reference_command` 100 Hz でファームに 300 フレーム/秒（CHASSIS_VEL + BODY_POSE + MODE）届く。usbipd 経由は到着がまとまるので `imu/data` の間隔は 0〜50 ms でばらつく（平均は 100 Hz）。

### 制御の構成（2026-10-24、ビルドのみ。実機確認は帰宅後）

設計書 [docs/superpowers/specs/2026-10-23-gamma-control-design.md](docs/superpowers/specs/2026-10-23-gamma-control-design.md)、計画 [docs/superpowers/plans/2026-10-23-gamma-control.md](docs/superpowers/plans/2026-10-23-gamma-control.md)。

```
Comm（PC 指令）──┐                      ┌── CanScheduler CAN2（0.5 ms × 8 関節 = 各 250 Hz）── 車輪 4 + 脚 4
ImuTask（BNO055、UART3、100 Hz）→ ImuStore ──┼── Controller ── BuildCommand ─┤
CanReceiver ×2 ──┘   （Tick 100 Hz）   └── CanScheduler CAN1（0.5 ms × 2 関節 = 各 1000 Hz）── 首 Yaw + Pitch（Roll はモータ無し）
（フィードバック）
```

| ファイル | 役割 |
|---|---|
| `src/robot.hpp` | 関節 / モード番号、関節とモータの対応、送信の並び（`CAN2_ORDER` / `CAN1_ORDER`）、制御定数 |
| `src/imu.hpp` | `ImuStore`（IMU の共有状態: 姿勢、角速度、有効フラグ、計測時刻、計測時の首 Yaw 角） |
| `src/imu_task.hpp` / `.cpp` | `ImuTask`。BNO055 を UART3 で読む FreeRTOS タスク。rabcl の `BNO055UartDriver`（起動シーケンス・100 Hz 読み・再初期化はバス非依存の `BNO055Sequencer`）を `SerialPort` でつなぎ、「要求の直前に受信バッファを空にする → 要求 → ヘッダ 2 バイトを読み、BB ならその長さ分を追加で読む（約 2 ms）→ 10 ms 境界で次」で回す（読み周期はこのタスクの tick だけが持つ）。BNO055 の融合更新と読みが重なると `EE 07`（bus over run）が返るので、その直後だけは 10 ms 境界を待たず即再送して位相をずらす（実機では毎秒 3〜4 回、連続はしない）。起動時に待たない（`valid = false` で始まり、初期化できた時点から使う）。落ちたら `valid = false` と `Comm::ClearImu` |
| `src/control.hpp` / `.cpp` | `Controller`。車輪 = `OmniDrive`（視点基準、首 Yaw の角度で回す）+ 無限回転 `SPIN_SPEED`。脚 = `BodyPosture` → `LegDamper` → MIT。首 Yaw = `Gimbal`（視点目標 − 胴体の向き `BodyAttitude`）→ `PdGravityFf` → LK トルク。`Tick`（100 Hz）が Comm の指令を取り込み、`CommandWatchdog` と `BodyAttitude` を更新 |
| `src/can_sched.hpp` / `.cpp` | `CanScheduler`（esp_timer で 1 スロット 1 関節。1 Hz の枠で DM にはイネーブル、LK にはフィードバックが無い間だけ状態読み出し 0x9C を充てる。LK は聞かれた時しか返さないので、これが無いと最初のフィードバックが来ず指令も始まらない）、`CanReceiver`（受信タスク。フィードバック ID → 関節 → `Controller::SetFeedback`、`Comm::SetJointState` は 4 回に 1 回） |
| `src/main.cpp` | 生成と Start、1 秒ごとのログ |

挙動:

- 起動時: 脚はイネーブル応答が来た時点の角度を目標にして保持（`LegDamper::Reset`）。最初の `BODY_POSE` から `LEG_RAMP_RATE` で指令の車高へ動く。首 Yaw は `JOINT_CMD` が来るまでトルクを出さず（状態読み出しだけ送ってフィードバックを返す）、来たら **現在角から** `YAW_RAMP_RATE` で目標へ寄せる（2026-11-15: 目標との差 3 rad に飽和指令を出して電源が落ちたので入れた）。首 Pitch は `JOINT_CMD` が来るまでゲイン 0 の MIT を送り続ける（軸は自由。送らないと DM の CAN タイムアウトで「通信途絶 D」の赤点滅になる）
- 指令途絶（`WATCHDOG_S` = 0.5 s）: 車輪 0、無限回転解除、首は最後の目標を保持、脚は最後の姿勢を保持。`error_flags` bit5。**PC を止めても首の目標は残る**ので、机上でトルクを切りたいときは P4 をリセットする
- 脚の安全策: フィードバックのトルクが `LEG_TORQUE_LIMIT`、ロータ温度が `LEG_TEMP_LIMIT` を超えた脚、またはモータ自身が異常コード（8〜E: 過電圧 / 過電流 / 過熱 / 過負荷など）を返した脚はディセーブルを送り続け、イネーブルも止める。`error_flags` bit0〜3（FR, FL, BR, BL）。復帰は再起動
- 再イネーブル（電圧低下などで一度切れたあと）: その脚は今の角度を目標にし直してからランプで指令の姿勢へ戻る（段差を作らない）
- 車輪・首はフィードバックが来るまで指令を送らない（未配線のモータでバスを埋めない。1 Hz の枠で DM へのイネーブル、LK へは状態読み出しだけ出る）
- モータの電源断（HP 0 で 24V だけ切れ、P4 は上位 PC の USB 給電で生きる）: フィードバックが `FEEDBACK_TIMEOUT_MS`（200 ms）止まった関節は「電源断」扱いにして指令を送らず、DM はイネーブル済みの印も落とす。`error_flags` bit6。復電でフィードバックが戻ると 1 Hz のイネーブルで有効になり、脚は現在角（メカストまで下がった位置）から `LEG_RAMP_RATE` で指令の姿勢へ戻る。車輪に電源が無い間は旋回速度の積分（IMU 無効時の首の基準）も止める
- 胴体の向き: IMU 有効かつ `stabilize = 1` なら IMU − 首関節角（計測時刻の首角を `ImuState.neck_yaw` で渡す）、そうでなければ旋回速度の積分（beta の yaw_ff 相当）。yaw を追うのは無限回転中だけ（普通の旋回では首は胴体と一緒に回る。`BodyAttitude::SetYawTracking`）。`error_flags` bit4 = IMU 無効
- IMU の約束: BNO055 の融合モードは IMU（磁気なし。方位は起動時の向きが 0）。heading は時計回り正なので `yaw = WrapPi(-heading)`、pitch / roll の符号は `IMU_PITCH_SIGN / IMU_ROLL_SIGN`（実機で確定）。軸の入れ替えは BNO055 の `AXIS_MAP`（`IMU_AXIS_MAP_CONFIG / SIGN`）で行い、出力をカメラ座標系（x 前、y 左、z 上）にする。ジャイロのゼロ点は静止を検出したとき自動で合う（起動後に静止待ちはしない）。通信が途絶・エラー続き（連続 10 回）ならソフトリセットして初期化をやり直す（終端状態なし）
- 車体速度はファーム側で `CHASSIS_VEL_MAX` / `CHASSIS_ROT_MAX` にクランプ（操作の上限は PC 側の launch）

制御定数（`src/robot.hpp`。値は仮、実機で合わせる）:

| 定数 | 値 | 意味 |
|---|---|---|
| `LEG_LINKAGE` | Lc 41、α0 60°、Lr 39.75、Lb 60、B1 (−26, −49)、j (0, 17.5)、w (38.3, −44) mm、θmax 90° | 脚の 4 節リンクの寸法（CAD 2026-10-25）。rabbits_description の `config/gamma_linkage.yaml` と対で更新。記号は同リポジトリの `docs/gamma_leg_linkage.svg` |
| `LEG_DROP_MIN / MAX` | 0.0691 / 0.1153 m（θ 0.15 / 1.42 rad） | 車高 0 / 1 の車輪の下がり。メカスト（0 / 90°）に 0.15 rad の余裕 |
| `LEG_TILT_RADIUS`、`LEG_AZIMUTH` | 0.164 m、FR −π/4 / FL π/4 / BR −3π/4 / BL 3π/4 | 傾き → 脚の下がりの差（接地点の半径 × 角度） |
| `LEG_KP / KD` | 1.0 / 0.1 | MIT ゲイン（実測済みの柔らかさ） |
| `LEG_LOAD_FORCE`、`LEG_FRICTION_FF / FRICTION_BAND` | 0 N、0.25 Nm / 0.04 rad | tff = 荷重 × dd/dθ（姿勢で変わる）+ 摩擦分 × clamp(差 / band, ±1)。sign と不感帯だと縁で 15 Hz の小振動が出た（2026-11-01）。荷重は接地してから取り直す |
| `LEG_RAMP_RATE` | 4.0 rad/s | 目標の移動速度。実機（浮かせた状態）で 0.5 / 1.0 / 2.0 は「遅い」、4.0 で前脚が 0.5 rad を 0.15 s、立ち上がりに 3° の行き過ぎ（2026-11-01）。`kp` / `kd` と荷重 FF は接地してから |
| `LEG_TORQUE_LIMIT / TEMP_LIMIT` | 4.0 Nm / 80 ℃ | 安全策 |
| `YAW_ZERO_OFFSET` | 0（**首の配線後に実測**） | エンコーダ 0 と機体前方のずれ [rad]。beta は 240°。車輪の視点基準・脚の傾きの向き・首の目標すべてに効く |
| `YAW_KP / KD / OUTPUT_MAX` | 50 / 5 / 100（モータ単体・無負荷、2026-11-15） | LK トルク指令値（0.01 A/LSB）。beta の 5000 / 600 / 2000 は単体では激しく発振し、上限 2000（約 16 A）は機体全体の電流制限 10 A を 1 軸で超える。機体に載せたら慣性に合わせて再調整 |
| `YAW_RAMP_RATE` | 10 rad/s | 首 Yaw の目標の移動速度。無限回転中の関節目標（`SPIN_SPEED` + `CHASSIS_ROT_MAX` = 8 rad/s）より大きくしておく |
| `PITCH_KP / KD / GRAVITY_FF` | 4 Nm/rad / 0.2 Nm·s/rad / 0 Nm | DM3507 の MIT（beta の値）。単体で目標に追従（2026-11-15）。重力分は機体に載せてから |
| `PITCH_MIN / MAX` | −15° / +30° | 可動角（URDF の `pitch_joint` と対）。向きは単体で確認済み、モータのゼロ（DMTool の SaveZero）は機体に載せてから |
| `WHEEL_D / BODY_D`、`WHEEL_SIGN` | 0.075 / 0.328 m、+1 | beta の omni_drive（beta は −1。gamma は 1:1 ギア 1 段でモータと車輪が逆回転、2026-11-16 実機で確定）。BODY_D は車輪の接地点の配置径（中間の車高、±2% は無視） |
| `CHASSIS_VEL_MAX / ROT_MAX` | 1.0 m/s / 3.0 rad/s | ファーム側の上限 |
| `SPIN_SPEED` | 5.0 rad/s | 無限回転の旋回（一定） |
| `WATCHDOG_S` | 0.5 s | 途絶判定 |
| `FEEDBACK_TIMEOUT_MS` | 200 ms | モータのフィードバック途絶 = 電源断の判定 |
| `IMU_AXIS_MAP_CONFIG / SIGN` | 0x24 / 0x00（試験向きで確定、2026-11-01） | BNO055 の軸入れ替え。本番（部品面が後ろ・USB-C が左）に載せたら再確定。試験向き（部品面が上・USB-C が右・USB-C を右にして座ったときの奥が前）からの計算では 0x06 / 0x07 の見込み |
| `IMU_PITCH_SIGN / IMU_ROLL_SIGN` | +1 / **−1**（試験向きで確定） | BNO055 のオイラー角 → REP-103。前を下げて pitch +0.25 rad、右を下げて roll +0.23 rad（符号反転後） |
| `IMU_GYRO_INDEX / SIGN` | {1, 0, 2} / {+1, +1, +1}（試験向きで確定） | ジャイロ → 機体座標。BNO055 の X 軸が左右、Y 軸が前後（heading は +Y 基準）なので x / y を入れ替える |
| `IMU_OPERATION_MODE` | 0x08 | 融合モード（IMU、磁気なし） |
| `IMU_USE_EXTERNAL_CRYSTAL` | true | AE-BNO055-BO は 32.768 kHz 水晶（XT1、FC-135）実装済み（秋月説明書の回路図で確認、2026-10-24）。水晶の無いモジュールに替えたら false |
| `IMU_SAMPLE_PERIOD_MS / RESPONSE_TIMEOUT_MS / BOOT_WAIT_MS` | 10 / 20 / 700 | 読み周期、応答待ち、起動 / リセット後の待ち |
| `IMU_MAX_CONSECUTIVE_ERRORS` | 10 | 超えたらソフトリセット |

実機確認の手順（帰宅後、脚 4 本で）:

1. 24V + USB で起動。起動時に脚が動かないこと（現在位置を保持）。`error_flags` と `JOINT_STATE` が PC に出ること（上位 PC を載せる前は開発 PC の USB-UART + ROS 2 の robot_bridge）
2. `reference_command` で `body_height` を 0 → 0.5 → 1 → 0。各脚が `BodyPosture` の計算どおりに動き、ロール / ピッチで左右・前後の差が出ること — **2026-11-01 OK**（浮かせた状態。0 = 0.150、0.5 = 0.835、1 = 1.420 rad、ロール ±0.15 とピッチ ±0.15 で片側 0.345 / 反対側 1.273。詳細は `docs/2026-10-25-control-bringup-checklist.md`）
3. 指令を止めて 0.5 s 後に途絶の扱いになり、脚が据え置き、`error_flags` の bit5 が立つこと。再開で復帰 — **2026-11-01 OK**
4. 1 本の脚を手で押してトルク上限を超えさせ、その脚だけディセーブル（LED が変わる）になること
5. 起動ログで esp_timer タスクのスタック残量を確認する（`CanScheduler` のコールバックで `OmniDrive::CalcVel` の double 演算が走る。足りなければ `CONFIG_ESP_TIMER_TASK_STACK_SIZE` を上げる）
6. **IMU（BNO055 のモジュール）**: (a) ジャンパは UART モード（J3 接続・J4 接続・**J5 を完全に除去**・J2 開放）。**J3 と J5 が同時に閉じていると VOUT（2.8 V）が GND に短絡して LP5907 が発熱する**（2026-11-01 に実測。2 台がこれで止まった）。盛る前に無電源で VOUT（ピン 8）–GND が高抵抗であることを確認し、挿す前にジャンパ線給電で VOUT 2.8 V と発熱なしを確認。(b) 起動ログの `IMU:` が `RUNNING` になり `samples` が毎秒約 100 増えること（`errors` は連続エラー数、`reinits` はソフトリセット回数）— **2026-11-01 OK**（毎秒 99、60 秒で再初期化 0）。(c) 静止させて `calib` の bit5:4（ジャイロ）が 3 になるまでの時間 — **OK**（起動後数秒）。(d) 軸: カメラを前に向けた状態から、カメラを下に向ける → `pitch` が正、機体を右に傾ける → `roll` が正、上から見て左に回す → `yaw` が増える。合わなければ `IMU_AXIS_MAP_*` と `IMU_*_SIGN` を直す — **試験向きで OK**（`IMU_ROLL_SIGN = −1` に変更。本番の取り付け向きで再確認）。(e) 一定速度で回して `gyro[2]` と yaw の変化率が一致。ピッチ・ロールも振って `gyro[0]` / `gyro[1]` と roll / pitch の変化率の符号が合うこと — **試験向きで OK**（BNO055 の X 軸が左右、Y 軸が前後だったので `IMU_GYRO_INDEX` で入れ替え。時計回りで gyro[2] −0.7 rad/s、yaw も減少）。(f) 無限回転中に首が視点を保つ（`Gimbal` の経路）。(g) 起動ログで `SET_CLOCK` を抜けて `RUNNING` に進むこと（外付け水晶 true で止まるなら水晶の実装か CLK_SEL の扱いを疑う）。(h) 首を回したまま IMU を抜き差し（または安定化を切 → 入）しても砲塔が跳ばない（`AnchorImu` の経路）
7. **24V 断 → 復電**（上位 PC か開発 PC の USB で P4 を給電したまま）: 24V を切ると 1 秒以内にログに `NO-FEEDBACK` と `MOTOR-POWER-LOST` が出て指令が止まり、戻すと脚が今の位置から静かに指令の姿勢へ戻ること（跳ねない）。`error_flags` bit6 が立って消えること
8. 必要なゲイン・`LEG_RAMP_RATE`・tff の値をここに記録し、`robot.hpp` を更新してからコミット — `LEG_RAMP_RATE` 4.0 と `LEG_FRICTION_BAND` 0.04 は記録済み。`LEG_KP / KD`、`LEG_LOAD_FORCE` は接地後
9. **`MOTOR_COMMAND`**: `ros2 topic hz /motor_commands` が約 100 Hz、`ros2 topic echo /motor_commands --once` に車輪 4 つ（24V 入り）と脚 4 本（イネーブル後）が入り、脚の `position` が指令どおり（`JOINT_STATE` の `position` と同じ値へ追いつく）。rviz（`gamma.launch.py rviz:=true`）で Command（不透明）と Actual（半透明）が重なり、車高を変えると Command が先に動いて Actual が追いつく。IMU で機体を傾けると Actual の胴体だけ傾く（`ros2 param set /imu_footprint_node height_mode plane` との見え方も比べる）

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
| 脚 | なし | **DM4310 × 4** | 🔧 |
| 装填 | 市販 MD + DC モータ（PWM） | **ST3215 サーボ（半二重 UART）**。rabcl の `St3215` + rabcl_esp32 の `HalfDuplexSerial` | 🔧 |
| IMU | BNO055（I2C、無効化中） | **BNO055（UART）**。rabcl の `BNO055UartDriver` + `ImuTask`。実機で RUNNING・約 100 サンプル/s を確認（2026-11-01） | ✅ |
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
| BNO055 UART | rabcl に UART フレーム / バス非依存の状態機械 / UART・I2C 層を追加（2026-10-24）。実機で UART の RUNNING・約 100 サンプル/s を確認（2026-11-01）。bus over run（毎秒 3〜4 回）は即再送で吸収し、60 秒で再初期化 0 回 | ✅ |
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

## ST3215 の試験モード

装填サーボ（ST3215）を単体で動かすための別ビルド。制御本体（CAN、IMU）は起動せず、USB コンソールから 1 行コマンドでサーボを操作する。設計書は rabcl の `docs/superpowers/specs/2026-10-25-st3215-design.md`、測定の目的は `Rabbits/docs/superpowers/specs/2026-07-26-bb-feeder-trigger-wheel-design.md` 5.1 の 4 項目（多回転、到達判定、1 ステップの所要時間、負方向）。

配線は SRV1（XH 3P: 1 = +7.4V、2 = DATA、3 = GND）。サーボ側の 5264 は 1 = DATA（白）、2 = VCC（赤）、3 = GND（黒）。P4 側は G8 TX / G10 RX が 1 kΩ で DATA に合流し、送信は自分のエコーとして戻る（`HalfDuplexSerial` が送信バイト数ぶん読み捨てる）。7.4V は DCDC の戻りの XT30 から入れる（安定化電源 2 A 制限でも可）。24V は不要。

```bash
pio run -e st3215_test -t upload --upload-port COM3
pio device monitor --port COM3
```

| コマンド | 動作 |
|---|---|
| `i` | PING と EEPROM / RAM（3〜70）のダンプ。モデル、ID、ボーレート、モード、現在位置など |
| `m <0-3>` | モード設定（0 位置、1 ホイール、2 PWM、3 ステップ）。EEPROM なので LOCK 0 → 書き → LOCK 1 |
| `t <0\|1>` | トルク OFF / ON |
| `s <steps> [speed=1000] [acc=50]` | 1 回の移動。到達（MOVING 0 かつ残り ±5）まで状態を読み続け（約 0.5 ms ごと）、10 ms ごとの残り・速度・MOVING の軌跡と所要時間を出す |
| `l <n> <steps> <period_ms>` | `s` を `period_ms` ごとに `n` 回（各回の到達を待つ）。最後に受理されたステップ量の合計 |
| `b <n> <steps> <period_ms> [speed] [acc]` | 連射。到達を待たずに `period_ms` ごとに `n` 回送り、10 ms ごとの残りステップと MOVING を出す（積算の確認） |
| `p [ms=50]` | 状態を流し続ける。何かキーで止まる |
| `w <reg> <val>` | 1 バイト書き込み |
| `r <reg> [len=1]` | 読み出し（16 進。2 バイトなら u16 / s16 も） |
| `h` | コマンド一覧 |

### サーボの初期設定（1 回だけ。EEPROM に残る）

出荷時は角度制限 0〜4095 が有効で、ステップモードでも絶対位置 4095 で止まる。多回転させるには角度制限を 0 / 0 にする。

```
w 55 0      # LOCK 解除
w 9 0       # MIN_ANGLE 下位
w 10 0      # MIN_ANGLE 上位
w 11 0      # MAX_ANGLE 下位
w 12 0      # MAX_ANGLE 上位
w 55 1      # LOCK
r 9 4       # 00 00 00 00 を確認
m 3         # ステップモード
```

EEPROM を書いた直後は応答が遅れて `FAILED` と出ることがある（値は書けている）。`r` で読み戻して確認する。

### 実測（2026-10-31、7.4V 安定化電源、無負荷）

| 項目 | 結果 |
|---|---|
| 多回転 | 角度制限 0/0 で +1024 × 5、±8192（2 回転）とも全量動く |
| 位置レジスタ（56）の意味 | ステップモードでは「残りステップ数」。絶対位置は位置モードで読む（0〜4095 で巻き戻る） |
| 到達確認 | MOVING（66）は acc = 0 だと当てにならない（移動中に 0）。残りステップ数 ±5 で判定する |
| 指令の積算 | 到達を待たず 100 ms 周期で送っても相対ステップは積算される（絶対位置の差で確認） |
| 171 カウント（15°）の到達 | 1000/50: 約 370 ms、2000/100: 約 280 ms、3000/254: 約 160 ms（行き過ぎ 3）、0/0（最高速）: 約 60 ms（行き過ぎ 22、整定 130 ms） |
| 負方向 | 正方向と同じ |

詳細は `Rabbits/docs/superpowers/specs/2026-07-26-bb-feeder-trigger-wheel-design.md` 5.1 の実測結果。

`-DGAMMA_ST3215_TEST` で `src/st3215_test.cpp` の `app_main` に切り替わる（`src/main.cpp` の `app_main` は外れる）。通常ビルド（`pio run`）のバイナリには影響しない。

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

派生環境: `-e st3215_test` は ST3215 の試験モード（後述）。`.pio/build/<env>/` が分かれるので通常ビルドには影響しない。

## ライセンス

MIT。[LICENSE](./LICENSE) を参照。
