# gamma 制御本体 設計

- 日付: 2026-10-23
- 対象リポジトリ: rabcl（制御部品・プロトコル追加）、rabbits_msgs、rabcl_ros2、rabbits_bridge、rabbits_interface、gamma_turret_fw（組み立て）
- 操作仕様: robot_ws/gamma の README「操作と制御の仕様」（2026-10-23）が正。本書はそれを実現する作り
- 状態: ユーザー確認済み（2026-10-23）

## 1. 目的と背景

gamma_turret_fw の `main.cpp` は脚 4 本の試験用の一本道で、制御の本体が無い。ゲーム本番の操作（キーボード・マウス、`GameDataInput`）から「走る・旋回・視点・射出・装填・歩き/走り・しゃがみ・無限回転・リーン」を実現する制御を、2 層（首 = 視点の安定化、脚 = アクティブダンパと車体姿勢）で作る。

計算の置き場はユーザー決定: **機体に依らない部品は rabcl（ユニットテストで固める）、gamma のファームは機体定数とタスクの組み立てだけ**。rabcl から先に実装する（実機なしで確認できるため）。

## 2. ゴール / 非ゴール / やってはいけないこと

**ゴール**

- PC からの指令（車体速度、砲塔の向き、車高・ロール・ピッチ、モード）を受けて、車輪・脚・首の全関節に仕様の周期（CAN2 各 250 Hz、CAN1 各 500 Hz）で指令を出す。
- 脚: 車高 0〜1 とロール・ピッチから 4 本の脚の目標角を作り、弱い kp + kd + フィードフォワード tff の MIT 指令で柔らかく保持する（アクティブダンパ）。しゃがみは下限まで素早く沈む。
- 首: IMU（カメラと同じリンク）で測った向きを、マウスで決めた「地面に対する向き」に保つ。無限回転中も視点を保つ。
- 指令途絶（0.5 s）で「立ったまま止まる」。
- IMU が無い（ドライバ未実装、故障）状態でも、安定化なしで全部が動く。
- 制御部品は rabcl にユニットテスト付きで入り、gamma のファームは実機なしでビルドが通る。

**非ゴール**

- ジャンプ（保留）。
- リーンに首のロールを併用すること（保留。胴体ロールで作る）。
- 首の Pitch / Roll の制御の深掘り。首の Pitch / Roll は **2 つの DM3507 による差動リンク**で、機構の設計がまだ途中（長引けば Pitch 軸だけ先に入れる可能性あり）。本書では「論理関節（pitch / roll）→ 2 モータの角度」の変換を 1 段置くところまでにし、差動の非線形性や 2 モータの協調・ゲインは機構が決まってから別の設計で扱う。
- BNO055 の UART ドライバ本体（別設計 B。本書は IMU の口だけ決める）。
- URDF（別件 C）。
- ゲイン・レバー長・tff の実値（実機で合わせる。本書は定数の置き場と意味だけ）。
- 装填（ST3215、半二重 UART）と射出（ESC PWM）の実装。モードの受け口だけ用意し、駆動は別設計。

**やってはいけないこと**

- rabcl に機体の値（ピン、CAN ID、レバー長、ゲイン）を書く。
- 指令途絶時に脚を脱力させる（車体が落ちる）。
- メカストに当たる目標角を出す（使用範囲でクランプする）。

## 3. 制約

- rabcl は C++17、標準ライブラリのみ、動的確保なし。STM32 / ESP32 / Linux で同じコード。
- CAN は 1 Mbps。CAN2 は車輪 4 + 脚 4 = 8 台を 0.5 ms ごとに 1 台（4,000 フレーム/秒、負荷約 52%、実測で損失 0）。CAN1 は首 3 台を 0.67 ms ごとに 1 台（3,000 フレーム/秒）。周期は定数で変えられる。
- IMU（BNO055）の融合出力は 100 Hz が上限。首のループ 500 Hz に対して姿勢は 100 Hz 更新。
- 脚: DM4310、MIT モード、0 rad = 最低車高（SaveZero はメカスト）、約 1.6 rad = 最高。PMAX 12.5 / VMAX 30 / TMAX 10。
- 車輪: DM2325、速度モード（指令フレーム 0x200 + CAN_ID）。首: Pitch / Roll は DM3507 の MIT、Yaw は MHF5005（LK）のトルク指令（beta と同じ）。
- 座標系は ROS REP-103: x 前、y 左、z 上。ロール正 = 右に傾く（右側が下がる）、ピッチ正 = 前が下がる、ヨー正 = 左回り。
- 命名・include・テストの規則は従来どおり（UPPER_SNAKE_CASE、rabcl 系はテスト必須）。
- **HP が 0 になるとゲームモジュールが 24V 幹線を切る**（2026-10-23 確認）。P4 キャリアも幹線から給電しているので、モータと一緒に P4 も落ちる（v1.0 に切られない電源は無い）。Jetson は別電源で落ちない。したがって (1) 電源断で脚は脱力し車体は最低車高のメカストまで落ちる（メカ側の前提: 無通電でメカストが車体を支える）、(2) 復電後はファームが試合中にコールドスタートする。起動時に勝手に動かないこと（4.4 の起動時の扱い）と、Jetson 側の bridge が再接続を待てることが要件。

## 4. 設計

### 4.1 全体の流れ（ファーム）

```
PC ──UART──> Comm ──(共有状態: 指令 + 途絶監視)──┐
                                               ▼
IMU task(100 Hz) ─> ImuState ──> [首の計算 Gimbal] ──> CAN1 スケジューラ(0.67 ms) ──> 首 3 台
                                               │
                  [車体姿勢 BodyPosture → 脚 LegDamper] ──> CAN2 スケジューラ(0.5 ms) ──> 脚 4 + 車輪 4
                  [車輪 OmniDrive + 無限回転]            ──┘
CAN RX task ──> 各関節のフィードバック（位置・速度・トルク・温度）──> Comm(JOINT_STATE 100 Hz)
```

- 各バスのスケジューラは esp_timer の周期コールバックで動き、呼ばれるたびに次の関節の指令を**その場で計算して**送る（送る直前の最新の指令・フィードバック・IMU を使う）。計算は数十 µs に収める。
- 共有状態は FreeRTOS の mutex で守る（Comm と同じ流儀）。

### 4.2 PC → ファームの指令（プロトコルの追加）

rabcl `protocol/` に **`BODY_POSE`（0x84）** を足す: `height f32`（0〜1、0 = 最低車高）、`roll f32 [rad]`、`pitch f32 [rad]`（12 B）。ロール・ピッチは視点（カメラ）座標系で、ファームが首 Yaw の関節角で車体座標系へ回す。`MODE` の番号表（gamma）: `load = 0`、`fire = 1`、`speed = 2`（0 = 歩き、1 = 走り）、`chassis = 3`（1 = 無限回転）、`stabilize = 4`（1 = 安定化あり。既定 1）。

ROS 2 側: `rabbits_msgs/ReferenceCommand` に `float32 body_height`、`float32 body_roll`、`float32 body_pitch` を足す（`chassis_vel` と同じく常に送る）。rabcl_ros2 `Convert::ReferenceToBodyPose`。bridge は `BODY_POSE` を毎回送る。YAML の `modes` に `stabilize` を足す。

game_client（本番）と gamepad（デバッグ）の割り当て:

| 操作 | game_client | gamepad（案） |
|---|---|---|
| 並進 / 旋回 | WASD / Z C | 十字キー / トリガー（残りの割り当ては実装時にユーザーと決める） |
| 視点 | マウス | 右スティック |
| 射出 / 装填 / 逆転 | 右 / 左（右と同時） / R | RB / LB / A |
| 歩き（押している間） | Ctrl → `speed = 0`。既定 `speed = 1` | LT（押している間） |
| しゃがみ（押している間） | Shift → `body_height = 0`。既定 `BODY_HEIGHT_DEFAULT` | X |
| 無限回転（押している間） | Space → `chassis = 1` | B |
| リーン（押している間） | Q / E → `body_roll = ∓LEAN_ROLL` | 未定（デバッグなので省略可） |
| 安定化の入切 | なし（常時オン。IMU 無効時は自動で安定化なし） | Y（トグル、デバッグ用） |

`BODY_HEIGHT_DEFAULT`（通常の車高、0〜1）と `LEAN_ROLL [rad]` は PC 側ノードのパラメータ（機体の値ではなく操作の値）。

### 4.3 rabcl に足す部品（`controller/`）

すべて機体の値は引数・設定構造体で受け取る。float、`<cmath>` のみ。

**`BodyPosture`** — 車高・ロール・ピッチ → 4 本の脚の目標角（脚は円状に等間隔）

```cpp
struct BodyPostureConfig
{
  float angle_min;       // 使用範囲 [rad]（メカストの余裕込み）
  float angle_max;
  float tilt_gain;       // [rad/rad] 車体の傾き 1 rad あたり、脚の取り付け位置で脚角に付ける差
  float leg_azimuth[4];  // 各脚の方位角 [rad]（車体 x 軸から左回り）。gamma: FR −π/4, FL π/4, BR −3π/4, BL 3π/4
};
class BodyPosture
{
  explicit BodyPosture(const BodyPostureConfig &);
  // height 0〜1 を [angle_min, angle_max] に線形に割り、脚 i に
  //   delta_i = tilt_gain × ( sin(φ_i − yaw_offset) × roll − cos(φ_i − yaw_offset) × pitch )
  // を足して範囲でクランプ。roll / pitch は視点（カメラ）座標系、yaw_offset は首 Yaw の関節角
  void Solve(float height, float roll, float pitch, float yaw_offset, float out[4]) const;
};
```

符号: ロール正（カメラから見て右が下がる）→ 右側の脚は縮む（角が小さい）、左側は伸びる。ピッチ正（前が下がる）→ 前側の脚は縮む。首が横を向いていれば「右・前」はカメラ基準で回る（車体速度と同じ視点基準）。`tilt_gain` は「脚の角度 1 rad で車体がどれだけ傾くか」の逆数で、実機のリンク寸法から決める（最初は実測で合わせる）。

**`LegDamper`** — 脚 1 本の MIT 指令（アクティブダンパ）

```cpp
struct LegDamperConfig
{
  float kp;
  float kd;
  float load_ff;      // [Nm] 荷重分（車体質量 / 4 の分担をレバーで換算。組み上がってから実測）
  float friction_ff;  // [Nm] 摩擦分（目標と現在の差の向きに足す）
  float deadband;     // [rad] この範囲内では摩擦分を足さない
  float ramp_rate;    // [rad/s] 目標の移動速度の上限（しゃがみの速さ）
};
struct MitCommand { float pos; float vel; float kp; float kd; float tff; };
class LegDamper
{
  explicit LegDamper(const LegDamperConfig &);
  // dt ごとに呼ぶ。目標を ramp_rate で現在の目標に近づけ、MIT 指令を返す
  MitCommand Update(float target, float angle, float vel, float dt);
  void Reset(float angle);  // 目標を現在角に合わせる（起動時、復帰時）
  float ramped_target() const;
};
```

tff = load_ff + friction_ff × sign(ramped_target − angle)（|差| > deadband のとき）。安全停止や途絶時は呼び出し側が target を据え置くだけ（脱力しない）。

**`BodyAttitude`** — 車体（首の根元）の地面に対する向きの推定

```cpp
struct Attitude { float yaw; float pitch; float roll; };    // [rad] 地面に対する向き
struct NeckAngles { float yaw; float pitch; float roll; };  // [rad] 首の関節角（モータ）
class BodyAttitude
{
  // IMU の更新があったとき: 車体の向き = IMU の向き − そのときの関節角（3 軸独立の小角度近似）
  // 注: pitch / roll は厳密には「首 Yaw 軸の先のリンク」の傾き（首ピッチ・ロールの軸が付く座標系）。Gimbal にはこれが正しい値
  void UpdateFromImu(const Attitude & imu, const NeckAngles & joint_at_imu_time);
  // IMU が無いとき（無効、または stabilize = 0）: ヨーだけ指令の旋回速度で積分する（beta の yaw_ff と同じ）。ピッチ・ロールは据え置き
  void UpdateFromChassisRate(float wz, float dt);
  Attitude Get() const;  // 起動時は 0（= 起動時の車体の向きが世界の基準）
};
```

**`Gimbal`** — 首 3 軸の目標角（視点の安定化）

```cpp
class Gimbal
{
  // 目標の向き（地面基準）から車体の向きの推定を引いて関節目標にする。yaw は ±π に正規化
  // joint_target = wrap(desired − body)
  NeckAngles Solve(const Attitude & desired, const Attitude & body) const;
};
```

- 安定化あり / なしの違いは `BodyAttitude` の更新の仕方だけ（IMU から / 旋回速度の積分から）。関節目標の式は同じなので、X キーの入切や IMU の有効・無効が変わっても目標の意味が変わらず、砲塔が跳ばない。IMU が再び有効になったときは積分の誤差分だけ段差が出るが、操縦者のマウスで吸収する。
- 車体の向きは IMU 更新時（100 Hz）に「その時刻の関節角」と組で計算するので、首のループ（500 Hz）が関節を動かしても推定が逃げない（IMU を据え置いたまま最新の関節角を足す形は取らない）。
- `ViewTarget` の 0 は「ファーム起動時の車体の向き」。起動直後は desired = 0、body = 0 で関節目標 0 になる。

**`ViewTarget`** — マウスの相対量から「地面に対する目標の向き」

```cpp
class ViewTarget
{
  void AddDelta(float dyaw, float dpitch);  // マウスの相対量 [rad]
  void SetPitchLimits(float min, float max);
  Attitude Get() const;                     // yaw は ±π に正規化、pitch はクランプ、roll は 0（リーン保留）
  void Reset(const Attitude &);
};
```

積分は **PC 側**（game_client / gamepad が `ViewTarget` を使う。rabcl なので同じコード）。`JOINT_CMD` の `yaw_joint` / `pitch_joint` / `roll_joint` の `position` を「地面に対する目標の向き」と解釈する（設計者の判断）。

**首の Pitch / Roll は保留**: 機構（2 つの DM3507 による差動リンク）が未決定のため、論理関節 ⇔ モータ角の変換（`Differential`）と首 Pitch / Roll の制御は機構が決まってから別の設計で足す。本計画のファームは首 Yaw（LK、`Gimbal` の yaw → `PdGravityFf`）だけを制御し、首 Pitch / Roll のモータには指令を送らない（CAN1 のスロットは空けておく）。

**`CommandWatchdog`** — 指令途絶の監視

```cpp
class CommandWatchdog
{
  explicit CommandWatchdog(float timeout_s);
  void Feed(float now_s);
  bool Expired(float now_s) const;  // 一度も Feed されていなければ true
};
```

無限回転は `SPIN_SPEED`（定数）で常に最高速度で回る（beta の sin 重ね合わせは使わない。実戦では一定の最高速度の方が良かったため）。rabcl に部品は置かない。

既存の `OmniDrive`（車輪の速度分配）と `PdGravityFf`（LK Yaw のトルク PD）はそのまま使う。`interface/can.hpp` に `PrepareDMMotorDisable`（FF FF FF FF FF FF FF FD）を足す。

### 4.4 ファーム（gamma_turret_fw）の構成

```
src/
├── board.hpp          ピン、ボーレート、CAN ID（既存）
├── robot.hpp          関節番号、モード番号、関節とモータの表（既存）+ 制御定数（4.5）
├── comm.hpp/.cpp      PC 通信（既存。BODY_POSE の受け口と最終受信時刻を追加）
├── imu.hpp            ImuState（姿勢・角速度・有効フラグ・時刻）の共有状態。BNO055 ドライバ（設計 B）がここに書く
├── control.hpp/.cpp   Controller: 共有状態を読んで各関節の指令を作る（4.3 の部品を機体定数で組む）
├── can_sched.hpp/.cpp 2 本のバスのスケジューラ（esp_timer、関節の順番と間隔）と CAN RX タスク（フィードバックの振り分け）
└── main.cpp           生成と Start だけ
```

- `Controller::Command(joint, now) → CanFrame`: 関節の種別で分岐（車輪: `OmniDrive` + 無限回転 → 速度指令。脚: `BodyPosture` の目標 → `LegDamper` → MIT。首 Pitch / Roll: 保留（機構決定後）。首 Yaw: `Gimbal` の目標 → `PdGravityFf` → LK トルク）。
- 首の Pitch / Roll はプロトコル上の関節番号（`pitch_joint` / `roll_joint`）だけ確保し、機構が決まるまで指令も `JOINT_STATE` も出さない。
- 車体速度の基準は **視点（砲塔）基準**: beta と同じく `OmniDrive::CalcVel` の `offset_rot` に首 Yaw の関節角を渡す（WASD は見ている方向に対して効く。無限回転中も狙った向きに走れる）。首 Yaw のフィードバックは CAN RX タスクが共有状態に入れ、CAN2 の計算が読む。
- 歩き / 走り: PC 側が `speed = 0`（Ctrl）のとき車体速度を `WALK_SCALE` 倍にして送る（操作の値は PC 側）。ファームは `CHASSIS_VEL` を機体定数の上限（`CHASSIS_VEL_MAX`、`CHASSIS_ROT_MAX`）でクランプするだけで、`speed` モードは受け取って保持するが制御には使わない（将来の用途と上りの状態用）。
- イネーブル: DM モータには起動時と 1 Hz でイネーブルを送る（beta と同じ）。
- 脚の安全策（試験用 main.cpp から引き継ぐ）: 脚ごとに「イネーブル応答（err = 1）が来るまで MIT 指令を送らない」「フィードバックのトルクが `LEG_TORQUE_LIMIT` を超えたらその脚に Disable 指令（`PrepareDMMotorDisable`、FF…FD）を送ってイネーブルを切り、`error_flags` に立てる（モータの LED で分かる。復帰の仕組みは今後決める。それまでは再起動）」「ロータ温度が `LEG_TEMP_LIMIT` を超えたら同じく Disable」。1 本だけ応答しない脚があっても他は動かし続ける（姿勢は崩れるがログと `error_flags` で分かる）。
- 起動時の脚: `LegDamper::Reset(現在角)` で今の位置を目標にして保持し、最初の `BODY_POSE` が届いた時点から `ramp_rate` で指令の車高へ動く（起動直後に勝手に立ち上がらない）。HP 0 の電源断からの復電（試合中のコールドスタート）もこの経路で、`ViewTarget` の基準（起動時の車体の向き）が変わるので PC 側は `INFO` を受け取ったら `ViewTarget::Reset` する（bridge が `INFO` の受信を上りの状態として出す。詳細は計画で）。
- 途絶時（`CommandWatchdog`）: 車輪 0、`fire` / `load` 0、`chassis` 0、首は最後の目標を保持、脚は `LegDamper` の目標据え置き。
- IMU 無効時: `Gimbal` は目標をそのまま関節角にする（安定化なし）。`stabilize = 0` も同じ扱い。
- フィードバックは CAN RX タスクが関節番号で振り分け、`Comm::SetJointState`（100 Hz 相当に間引き）と制御の共有状態に入れる。

### 4.5 機体定数（`robot.hpp`、最初の値は仮）

| 定数 | 意味 | 初期値（仮） |
|---|---|---|
| `LEG_ANGLE_MIN / MAX` | 脚の使用範囲 [rad] | 0.15 / 1.45 |
| `LEG_TILT_GAIN` | 体の傾き → 脚角の差 [rad/rad] | 1.0（実測で合わせる） |
| `LEG_AZIMUTH[4]` | 脚の方位角 [rad]（FR, FL, BR, BL） | −π/4, π/4, −3π/4, 3π/4 |
| `LEG_KP / KD` | 脚の MIT ゲイン | 1.0 / 0.1（実測済みの柔らかさ） |
| `LEG_LOAD_FF / FRICTION_FF / DEADBAND` | tff [Nm] | 0 / 0.25 / 0.02（組み上がってから取り直す） |
| `LEG_RAMP_RATE` | 目標の移動速度 [rad/s] | 0.5（しゃがみの速さ。実機で上げる） |
| `NECK_KP / KD` | DM3507 の MIT ゲイン（首 Pitch / Roll を入れるとき） | beta の PITCH_KP / KD |
| `NECK_DIFFERENTIAL` | 論理関節 → 2 モータの係数 | 仮: a = p + r、b = p − r（機構確定後に差し替え。Pitch のみなら roll 係数 0） |
| `YAW_PD` | LK Yaw の PD（kp, kd, gravity_ff, max） | beta の yaw_pd |
| `CAN2_SLOT_US / CAN1_SLOT_US` | 送信間隔 | 500 / 667 |
| `WATCHDOG_S` | 途絶判定 | 0.5 |
| `LEG_TORQUE_LIMIT / LEG_TEMP_LIMIT` | 脚の安全策 [Nm] / [℃] | 4.0 / 80 |
| `CHASSIS_VEL_MAX / CHASSIS_ROT_MAX` | 車体速度の上限 [m/s] / [rad/s] | beta の launch の値 |
| `SPIN_SPEED` | 無限回転の旋回 [rad/s]（一定） | 5.0（beta の最高速度） |

### 4.6 段階と実機確認

1. rabcl: `BODY_POSE` + `controller/` の 6 部品 + テスト。
2. rabbits_msgs / rabcl_ros2 / rabbits_bridge / rabbits_interface: `body_*` と `stabilize`、game_client のキー割り当て、gamepad の割り当て。
3. gamma ファーム: 4.4 の構成に組み替え。脚 4 本で実機確認（車高 0〜1、しゃがみ、ロール・ピッチ、途絶時の停止）。車輪と首は配線後。IMU は設計 B のあと。

## 5. 設計者の判断（ユーザー未確認）

- `BODY_POSE` を新規メッセージにし、`ReferenceCommand` に `body_height / roll / pitch` を足す（常に送る）。
- `MODE` に `stabilize`（番号 4）を足す。`speed` は 0 = 歩き、1 = 走り。
- 視点の目標の積分は PC 側（`ViewTarget` を rabcl に置き、game_client / gamepad で使う）。`JOINT_CMD` の首 3 軸は「地面に対する目標の向き」。
- 首の安定化は 3 軸独立の小角度近似。IMU の方位はジャイロ積分主体（磁気を使わない。設計 B で決める）。
- 脚の姿勢変換は線形（車高 0〜1 → 角度、ロール・ピッチ → 逆符号のオフセット）。非線形な機構の補正は実機で必要なら足す。
- 摩擦分の tff は「目標と現在の差の向き」に足す（速度の向きではなく）。
- 無限回転は一定の最高速度（`SPIN_SPEED`）。beta の sin 重ね合わせは使わない。視点の保持は IMU 有効なら `Gimbal`、無効なら beta と同じ送り成分（yaw_ff）で補う。
- 送信スケジューラは rabcl には置かず gamma 側（esp_timer 依存のため）。
- 首の目標の基準合わせ: `BodyAttitude`（車体の向きの推定）を 1 つ持ち、安定化の有無は推定の更新元（IMU / 旋回速度の積分）の違いだけにする。これで入切や IMU の有効・無効で砲塔が跳ばない（ビルダーチェック 1・5）。
- 車体速度は視点基準（beta 踏襲、ビルダーチェック 2）。
- 歩き / 走りは PC 側の倍率、ファームは上限クランプのみ（ビルダーチェック 3）。
- 脚の安全策（イネーブル待ち、トルク / 温度のトリップ、1 本欠けても継続、起動時は現在位置を保持）（ビルダーチェック 4）。

## 6. ユーザー確認事項

（2026-10-23 決定）

- **Q1** ゲームパッド: 移動は十字キー。残りの割り当ては実装時にユーザーと決める。
- **Q2** 安定化: 常時オン。キーボードに入切は割り当てない。`stabilize` はプロトコルに残し、ゲームパッド（デバッグ）だけで切れる。
- **Q3** 車体速度: カメラ座標系（視点基準、beta と同じ）。
- **Q4** 脚のトリップ: その脚のイネーブルを切る（Disable 指令）。モータの LED で分かる。復帰の仕組みは今後考える。
