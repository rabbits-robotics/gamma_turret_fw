# gamma 制御本体 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 設計書（`docs/superpowers/specs/2026-10-23-gamma-control-design.md`）の制御を、rabcl の部品（テスト付き）→ ROS 2 側 → gamma ファームの順に実装する。実機確認（脚 4 本）は帰宅後。

**Architecture:** 2 層制御（首 = 視点の安定化、脚 = アクティブダンパと車体姿勢、車輪 = 視点基準の速度分配）。機体に依らない計算は rabcl `controller/` に置き、gamma ファームは `robot.hpp` の定数と、esp_timer の送信スケジューラから各関節の指令をその場で計算する `Controller` の組み立てだけ。PC → ファームに `BODY_POSE`（車高・ロール・ピッチ）を追加。

**Tech Stack:** C++17 / gtest（rabcl、Docker `rabbits/ros2-jazzy-dev`）、ROS 2 Jazzy（rabbits_msgs / rabcl_ros2 / rabbits_bridge / rabbits_interface）、ESP-IDF 5.5.5 + PlatformIO（gamma_turret_fw、PowerShell で `pio run`）。

**Spec:** `fw_ws/gamma_turret_fw/docs/superpowers/specs/2026-10-23-gamma-control-design.md`。操作仕様は `robot_ws/gamma/README.md`「操作と制御の仕様」。

## Global Constraints

- rabcl: `<cstdint>` / `<cmath>` と標準ライブラリのみ、動的確保・例外なし。機体の値（ピン、CAN ID、レバー長、ゲイン）を書かない。すべて設定構造体か引数で受ける。
- 座標系は REP-103（x 前、y 左、z 上。ロール正 = 右が下がる、ピッチ正 = 前が下がる、ヨー正 = 左回り）。角度は rad、角速度は rad/s、トルクは Nm。
- プロトコル追加: `BODY_POSE = 0x84`、payload `height f32（0〜1）, roll f32, pitch f32`（12 B）。gamma の `MODE` 番号: `load 0, fire 1, speed 2（0 歩き / 1 走り）, chassis 3（1 無限回転）, stabilize 4（1 あり、既定 1）`。
- 脚: 0 rad = 最低車高、使用範囲 `[angle_min, angle_max]` でクランプ。tff = load_ff + friction_ff × sign(目標 − 現在)（|差| > deadband）。
- 指令途絶 0.5 s: 車輪 0、fire / load 0、chassis 0、首は最後の目標、脚は目標据え置き（脱力しない）。
- 脚のトリップ（トルク / 温度）: その脚に Disable（`FF FF FF FF FF FF FF FD`）を送り `error_flags` を立てる。復帰は再起動。
- 安定化は常時オン。IMU 無効なら `BodyAttitude` は旋回速度の積分で更新（beta の yaw_ff）。
- 車体速度は視点基準（`OmniDrive::CalcVel` の `offset_rot` に首 Yaw の関節角）。歩きは PC 側の倍率、ファームは上限クランプ。
- ゲイン・レバー長・tff の実値は仮。`robot.hpp` にまとめ、実機で合わせる。
- 命名: 定数 UPPER_SNAKE_CASE、include は外部 `<>` / 自リポジトリ `""`。rabcl 系はテスト必須、uncrustify（行長 100）。コミットはユーザー承認後、1 行の日本語。

## Review Focus

- `BodyPosture::Solve` に height < 0 / > 1 や大きな roll を渡しても出力が `[angle_min, angle_max]` に収まる → Task 2 `SolveClampsEveryLeg`。
- `LegDamper::Update` の目標ランプが dt = 0 や負の dt で発散しない → Task 2 `UpdateWithZeroDtHoldsTarget`。
- `Gimbal` の yaw で desired − body が ±π をまたぐとき最短側に回る → Task 3 `YawWrapsAcrossPi`。
- `CommandWatchdog` が一度も `Feed` されていない起動直後に `Expired() == true` → Task 4 `ExpiredBeforeFirstFeed`。
- bridge が `SYSTEM` の `uptime_ms` の減少（ファーム再起動）で `firmware_reboot` を 1 回だけ出す → Task 5 `RebootIsDetectedFromUptime`。

## File Structure

| リポジトリ | 作る / 変える | 役割 |
|---|---|---|
| rabcl | `include/rabcl/protocol/protocol.hpp`, `src/protocol/protocol.cpp`, `test/test_protocol.cpp` | `BODY_POSE` |
| rabcl | `include/rabcl/interface/can.hpp`, `src/interface/can.cpp`, `test/test_can.cpp` | `PrepareDMMotorDisable` |
| rabcl | `include/rabcl/controller/{body_posture,leg_damper,body_attitude,gimbal,view_target,differential,command_watchdog,infinite_rotation}.hpp` と `src/controller/*.cpp`、`test/test_*.cpp`、`CMakeLists.txt`、`README.md` | 制御部品 |
| rabbits_msgs | `msg/ReferenceCommand.msg`, `README.md` | `body_height / body_roll / body_pitch` |
| rabcl_ros2 | `include/rabcl_ros2/convert.hpp`, `src/convert.cpp`, `test/test_convert.cpp` | `ReferenceToBodyPose` |
| rabbits_bridge | `src/robot_bridge_node.cpp`, `include/.../robot_bridge_node.hpp`, `config/*.yaml`, `test/test_robot_bridge.cpp`, `README.md` | `BODY_POSE` 送信、`stabilize`、`firmware_reboot` |
| rabbits_interface | `include/rabbits_interface/reference_builder.hpp`, `src/game_client_node.cpp`, `src/gamepad_node.cpp`, 各ヘッダ・テスト・launch、`README.md` | 操作の割り当て、`ViewTarget` |
| gamma_turret_fw | `src/robot.hpp`, `src/imu.hpp`, `src/control.hpp/.cpp`, `src/can_sched.hpp/.cpp`, `src/comm.hpp/.cpp`, `src/main.cpp`, `src/CMakeLists.txt`, `README.md` | 組み立て |

---

### Task 1: rabcl プロトコル `BODY_POSE` と `PrepareDMMotorDisable`

**Files:**
- Modify: `include/rabcl/protocol/protocol.hpp`, `src/protocol/protocol.cpp`, `test/test_protocol.cpp`, `include/rabcl/interface/can.hpp`, `src/interface/can.cpp`, `test/test_can.cpp`

**Interfaces:**
- Produces: `enum class Type { ..., BODY_POSE = 0x84 }`, `struct BodyPose { float height; float roll; float pitch; }`, `constexpr uint8_t BODY_POSE_LEN = 12`, `size_t EncodeBodyPose(const BodyPose &, uint8_t * buf, size_t cap)`, `bool DecodeBodyPose(const uint8_t * payload, uint8_t len, BodyPose &)`。`static void Can::PrepareDMMotorDisable(uint8_t can_data[8])`（`FF FF FF FF FF FF FF FD`）。

- [ ] **Step 1: テストを書く（RED）**

```cpp
TEST(Protocol, BodyPoseRoundTrip)        // {0.3f, -0.1f, 0.05f} → len 12、type 0x84、往復一致
TEST(Protocol, BodyPoseDecodeRejectsWrongLen)
TEST_F(CanTest, PrepareDMMotorDisable)   // 8 バイトが FF FF FF FF FF FF FF FD
```

- [ ] **Step 2: RED を確認** Run（rabcl、Docker）: `colcon build --packages-select rabcl`（以降 rabcl のビルド / テストはすべてこのコマンドの流儀: `docker run --rm -v <rabcl>:/mnt/rabcl:ro rabbits/ros2-jazzy-dev bash -lc 'cp -r /mnt/rabcl ~/ws/src/rabcl && cd ~/ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select rabcl && colcon test --packages-select rabcl && colcon test-result --verbose'`） Expected: `BodyPose` / `PrepareDMMotorDisable` 未定義のコンパイルエラー

- [ ] **Step 3: 実装**（`EncodeBodyPose` / `DecodeBodyPose` は `CHASSIS_VEL` と同じ形。`MAX_FRAME` 等は変更なし）

- [ ] **Step 4: GREEN** Expected: `Summary: N tests, 0 failures`（N = 192 + 3）

- [ ] **Step 5: コミット（承認後）** `BODY_POSE と DM のディセーブル指令を追加`

---

### Task 2: rabcl `BodyPosture` と `LegDamper`

**Files:**
- Create: `include/rabcl/controller/body_posture.hpp`, `src/controller/body_posture.cpp`, `include/rabcl/controller/leg_damper.hpp`, `src/controller/leg_damper.cpp`, `test/test_body_posture.cpp`, `test/test_leg_damper.cpp`
- Modify: `CMakeLists.txt`（IDF の SRCS と ament の `add_library`、`ament_add_gtest` × 2）

**Interfaces:**
- Produces（名前空間 `rabcl`）:
  ```cpp
  struct BodyPostureConfig { float angle_min; float angle_max; float tilt_gain; float leg_azimuth[4]; };
  class BodyPosture { explicit BodyPosture(const BodyPostureConfig &);
    void Solve(float height, float roll, float pitch, float yaw_offset, float out[4]) const; };  // out は leg_azimuth の順（gamma: FR, FL, BR, BL）。roll / pitch はカメラ座標系
  struct LegDamperConfig { float kp; float kd; float load_ff; float friction_ff; float deadband; float ramp_rate; };
  struct MitCommand { float pos; float vel; float kp; float kd; float tff; };
  class LegDamper { explicit LegDamper(const LegDamperConfig &);
    MitCommand Update(float target, float angle, float vel, float dt);
    void Reset(float angle); float ramped_target() const; };
  ```
- `Solve`: base = angle_min + height × (angle_max − angle_min)（height は 0〜1 にクランプ）。脚 i: delta = tilt_gain × ( sin(φ_i − yaw_offset) × roll − cos(φ_i − yaw_offset) × pitch )。最後に各脚を `[angle_min, angle_max]` でクランプ。
- `Update`: ramped を target へ ramp_rate × dt だけ近づける（dt ≤ 0 なら動かさない）。tff = load_ff + (|ramped − angle| > deadband ? friction_ff × sign(ramped − angle) : 0)。戻り値 `{ramped, 0, kp, kd, tff}`。`Reset(angle)` で ramped = angle。初期値は 0。

- [ ] **Step 1: テストを書く（RED）**

```cpp
// BodyPosture（config: min 0.15, max 1.45, tilt_gain √2, azimuth −π/4, π/4, −3π/4, 3π/4）
TEST(BodyPosture, HeightMapsLinearly)        // height 0 → 全脚 0.15、1 → 1.45、0.5 → 0.80
TEST(BodyPosture, RollPositiveLowersRightSide) // height 0.5, roll 0.1 → FR = BR = 0.70, FL = BL = 0.90
TEST(BodyPosture, PitchPositiveLowersFront)  // height 0.5, pitch 0.2 → FR = FL = 0.60, BR = BL = 1.00
TEST(BodyPosture, YawOffsetRotatesTiltIntoChassisFrame) // yaw_offset π/2, pitch 0.2 → FR = BR = 1.00, FL = BL = 0.60
TEST(BodyPosture, YawOffsetPiFlipsSign)      // yaw_offset π, roll 0.1 → FR = BR = 0.90, FL = BL = 0.70
TEST(BodyPosture, SolveClampsEveryLeg)       // height 2.0, roll 5.0 → 全脚が [0.15, 1.45] 内
// LegDamper（config: kp 1, kd 0.1, load_ff 0.5, friction_ff 0.25, deadband 0.02, ramp_rate 0.5）
TEST(LegDamper, ResetSetsRampedTarget)
TEST(LegDamper, RampLimitsTargetSpeed)       // Reset(0), Update(1.0, 0, 0, 0.1) → pos 0.05、繰り返して 1.0 で止まる
TEST(LegDamper, FrictionFfFollowsErrorSign)  // ramped 0.5, angle 0.4 → tff 0.75、angle 0.6 → 0.25、angle 0.505 → 0.5
TEST(LegDamper, GainsPassThrough)            // kp 1, kd 0.1、vel 0
TEST(LegDamper, UpdateWithZeroDtHoldsTarget) // dt 0 と dt −1 で ramped が動かない
```

- [ ] **Step 2: RED を確認** Expected: ヘッダ無しのコンパイルエラー
- [ ] **Step 3: 実装と CMake**
- [ ] **Step 4: GREEN** Expected: 0 failures（uncrustify 含む）
- [ ] **Step 5: コミット（承認後）** `車体姿勢と脚のアクティブダンパを追加`

---

### Task 3: rabcl `BodyAttitude`、`Gimbal`、`ViewTarget`

**Files:**
- Create: `include/rabcl/controller/{body_attitude,gimbal,view_target}.hpp`、対応する `src/controller/*.cpp`、`test/test_{body_attitude,gimbal,view_target}.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces（`rabcl`）:
  ```cpp
  struct Attitude { float yaw; float pitch; float roll; };
  struct NeckAngles { float yaw; float pitch; float roll; };
  class BodyAttitude { void UpdateFromImu(const Attitude & imu, const NeckAngles & joint_at_imu_time);
    void UpdateFromChassisRate(float wz, float dt); Attitude Get() const; void Reset(); };
  class Gimbal { NeckAngles Solve(const Attitude & desired, const Attitude & body) const; };  // 各軸 wrap(desired − body)、yaw は ±π
  class ViewTarget { void AddDelta(float dyaw, float dpitch); void SetPitchLimits(float min, float max);
    Attitude Get() const; void Reset(const Attitude &); };
  ```
- `Utils::ShortestPathMultiTurn` は使わず、±π への正規化は `Utils` に `WrapPi(float)` を足して共用する（`utils.hpp`、テスト 1 本）。

- [ ] **Step 1: テストを書く（RED）**

```cpp
TEST(Utils, WrapPi)                            // 4.0 → 4.0 − 2π、−4.0 → −4.0 + 2π、π → π か −π のどちらか（|x| ≤ π）
TEST(BodyAttitude, ImuMinusJointGivesBody)     // imu yaw 1.0, joint yaw 0.3 → body yaw 0.7（pitch / roll も同様）
TEST(BodyAttitude, ChassisRateIntegratesYawOnly) // UpdateFromChassisRate(2.0, 0.5) → yaw +1.0、pitch / roll 不変
TEST(BodyAttitude, ResetZeroes)
TEST(Gimbal, SubtractsBody)                    // desired yaw 0.5, body 0.2 → joint 0.3
TEST(Gimbal, YawWrapsAcrossPi)                 // desired 3.0, body −3.0 → joint ≈ −0.283（最短側）
TEST(ViewTarget, AddDeltaAccumulatesAndWrapsYaw)
TEST(ViewTarget, PitchClampedToLimits)         // limits (−π/12, π/6)
TEST(ViewTarget, ResetSetsTarget)
```

- [ ] **Step 2: RED を確認** Expected: ヘッダ無しのコンパイルエラー
- [ ] **Step 3: 実装と CMake**
- [ ] **Step 4: GREEN** Expected: 0 failures
- [ ] **Step 5: コミット（承認後）** `首の安定化の部品を追加`

---

### Task 4: rabcl `CommandWatchdog`、README

**Files:**
- Create: `include/rabcl/controller/command_watchdog.hpp`、`src/controller/command_watchdog.cpp`、`test/test_command_watchdog.cpp`
- Modify: `CMakeLists.txt`、`README.md`（`controller/` の行に 6 部品を追記）

**Interfaces:**
- Produces:
  ```cpp
  class CommandWatchdog { explicit CommandWatchdog(float timeout_s); void Feed(float now_s); bool Expired(float now_s) const; };
  ```

- [ ] **Step 1: テストを書く（RED）**

```cpp
TEST(CommandWatchdog, ExpiredBeforeFirstFeed)       // Expired(0) == true
TEST(CommandWatchdog, NotExpiredWithinTimeout)      // Feed(1.0); Expired(1.4) == false（timeout 0.5）
TEST(CommandWatchdog, ExpiredAfterTimeout)          // Expired(1.6) == true
```

- [ ] **Step 2: RED → Step 3: 実装 → Step 4: GREEN**（0 failures）
- [ ] **Step 5: README 更新、コミット（承認後）** `指令途絶の監視を追加`

---

### Task 5: ROS 2 側（rabbits_msgs / rabcl_ros2 / rabbits_bridge）

**Files:**
- Modify: `rabbits_msgs/msg/ReferenceCommand.msg`（`float32 body_height`, `float32 body_roll`, `float32 body_pitch` を `chassis_vel` の次に）、`rabbits_msgs/README.md`
- Modify: `rabcl_ros2/include/rabcl_ros2/convert.hpp`, `src/convert.cpp`, `test/test_convert.cpp`（`static void ReferenceToBodyPose(const ReferenceCommand &, rabcl::protocol::BodyPose &)`）
- Modify: `rabbits_bridge/src/robot_bridge_node.cpp`, `include/.../robot_bridge_node.hpp`, `config/beta.yaml`（`modes` に `stabilize` を追加。beta は無視する）, `config/gamma.yaml`（同）, `test/test_robot_bridge.cpp`, `README.md`

**Interfaces:**
- Consumes: Task 1 の `BodyPose` / `EncodeBodyPose` / `DecodeSystem`
- Produces: bridge は `reference_command` ごとに `CHASSIS_VEL` → `BODY_POSE` → `JOINT_CMD`（あれば）→ `MODE`（あれば）の順に送る。`SYSTEM` を受けたら `uptime_ms` を記録し、前回より小さい（再起動）か、bridge 起動後初めて受けたときに `std_msgs/Empty` を `firmware_reboot` に 1 回出す。

- [ ] **Step 1: rabbits_msgs を変更してビルド** Expected: `Finished <<< rabbits_msgs`
- [ ] **Step 2: rabcl_ros2 のテスト（RED）** `TEST(Convert, ReferenceToBodyPoseCopiesFields)` → RED → 実装 → GREEN
- [ ] **Step 3: bridge のテスト（RED）**

```cpp
TEST_F(RobotBridgeTest, ReferenceCommandWritesBodyPose)   // chassis_vel のみの msg → CHASSIS_VEL と BODY_POSE の 2 フレーム
TEST_F(RobotBridgeTest, RebootIsDetectedFromUptime)       // SYSTEM uptime 5000 → firmware_reboot 1 回（初回）、6000 → 出ない、1000 → 1 回
```

既存の `EmptyArraysSendOnlyChassisVel` / `ReferenceCommandWritesFrames` / `BackpressureKeepsEveryFrame` の期待フレーム数を +1（`BODY_POSE`）に更新する。

- [ ] **Step 4: RED → 実装 → GREEN**（`colcon test --packages-select rabbits_bridge` 0 failures）
- [ ] **Step 5: README 更新、コミット（承認後）** rabbits_msgs `ReferenceCommand に車体姿勢を追加`、rabcl_ros2 `車体姿勢の変換を追加`、rabbits_bridge `BODY_POSE の送信と再起動の検出を追加`

---

### Task 6: rabbits_interface（game_client のキー割り当て、gamepad、`ViewTarget`）

**Files:**
- Modify: `include/rabbits_interface/reference_builder.hpp`（`MODE_STABILIZE = "stabilize"`、`InitReference` が mode 5 個・`body_height = BODY_HEIGHT_DEFAULT` 引数で初期化）、`include/rabbits_interface/game_client_node.hpp`, `src/game_client_node.cpp`, `include/rabbits_interface/gamepad_node.hpp`, `src/gamepad_node.cpp`, `test/test_game_client.cpp`, `test/test_gamepad.cpp`, `launch/*.launch.py`（新パラメータ）, `README.md`, `CMakeLists.txt` / `package.xml`（rabcl 依存を戻す: `ViewTarget` を使うため）, `.repos`（rabcl を戻す）

**Interfaces:**
- Consumes: Task 3 `rabcl::ViewTarget`、Task 5 の `ReferenceCommand` の `body_*`
- Produces: game_client の割り当て（操作仕様）: Ctrl → `chassis_vel` を `walk_scale`（パラメータ、既定 0.4）倍、`speed = 0`。Shift → `body_height = 0`、それ以外 `body_height_default`（既定 0.5）。Space → `chassis = 1`。Q → `body_roll = −lean_roll`、E → `+lean_roll`（既定 0.15 rad）。`stabilize = 1` 固定。マウスは `ViewTarget::AddDelta(dx / 400, dy / 400)`（今の係数）。`firmware_reboot` を購読したら `ViewTarget::Reset({0,0,0})`。
- gamepad: 十字キーで並進（axes 6 / 7）、他は **ユーザーと相談して決める**（Step 1 で聞く）。

- [ ] **Step 1: gamepad の割り当てをユーザーに確認する**（候補: 左スティックも並進、右スティック視点、RB 射出、LB 装填、A 逆転、LT 歩き、X しゃがみ、B 無限回転、Y 安定化トグル。十字キーの並進は確定）
- [ ] **Step 2: テストを書く（RED）**

```cpp
// game_client
TEST_F(GameClientTest, CtrlScalesVelocityAndSetsWalk)    // key_w + key_ctrl → chassis_vel.x == max × walk_scale、mode speed == 0
TEST_F(GameClientTest, DefaultIsRunAndDefaultHeight)      // speed == 1、body_height == body_height_default
TEST_F(GameClientTest, ShiftCrouches)                      // body_height == 0
TEST_F(GameClientTest, SpaceSetsInfiniteRotation)         // chassis == 1（Shift では立たない）
TEST_F(GameClientTest, QELean)                             // Q → body_roll == −lean_roll、E → +lean_roll
TEST_F(GameClientTest, StabilizeAlwaysOn)                  // stabilize == 1
TEST_F(GameClientTest, RebootResetsView)                   // yaw を積分 → firmware_reboot → yaw 0
// gamepad: Step 1 の割り当てに合わせて同種のテスト
```

- [ ] **Step 3: RED → 実装 → GREEN**（`colcon test --packages-select rabbits_interface` 0 failures）
- [ ] **Step 4: README 更新（操作表）、コミット（承認後）** `操作仕様に合わせて指令を組み立てる`

---

### Task 7: gamma ファームの組み替え（ビルドまで）

**Files:**
- Create: `src/imu.hpp`（`struct ImuState { rabcl::Attitude attitude; float gyro[3]; bool valid; int64_t time_us; }` と mutex 付きの置き場）、`src/control.hpp`, `src/control.cpp`、`src/can_sched.hpp`, `src/can_sched.cpp`
- Modify: `src/robot.hpp`（4.5 の定数。`Joint` の PITCH / ROLL は番号だけ確保し、機構決定まで指令を送らない）、`src/comm.hpp/.cpp`（`BODY_POSE` の受け口 `GetBodyPose`、最終受信時刻 `last_rx_time_us()`、`SetErrorFlags`）、`src/main.cpp`（生成と Start だけ。試験ループは削除）、`src/CMakeLists.txt`、`README.md`
- rabcl submodule を Task 4 のコミットに更新

**Interfaces:**
- Consumes: Task 1〜4 の rabcl 部品、既存 `rabcl_esp32::CanBus` / `Comm`
- Produces:
  ```cpp
  class Controller {  // 共有状態（指令、フィードバック、IMU）を持ち、関節ごとの指令を作る
    void SetFeedback(robot::Joint, const rabcl::MotorInfo &, uint8_t err);
    bool BuildCommand(robot::Joint, int64_t now_us, rabcl_esp32::CanFrame & out);  // false = 送らない（未イネーブル、トリップ）
    void Tick(int64_t now_us);  // 途絶監視、BodyAttitude の積分
    uint32_t error_flags() const; };
  class CanScheduler {  // バス 1 本分: 関節の並びと間隔。esp_timer から Controller::BuildCommand → CanBus::Send。1 Hz で DM にイネーブル
    CanScheduler(rabcl_esp32::CanBus &, Controller &, const robot::Joint * joints, size_t n, uint32_t slot_us); esp_err_t Start(); };
  ```
- CAN2 の並び: `WHEEL_FR, LEG_FR, WHEEL_FL, LEG_FL, WHEEL_BR, LEG_BR, WHEEL_BL, LEG_BL`（0.5 ms）。CAN1: `YAW, PITCH, ROLL`（0.67 ms。PITCH / ROLL のスロットは機構決定まで送らない = `BuildCommand` が false）。
- 途絶・起動・トリップは設計書 4.4 のとおり。`error_flags`: bit0〜3 = 脚 FR..BL のトリップ、bit4 = IMU 無効、bit5 = 指令途絶。`SYSTEM` の `error_flags` に載せる。

- [ ] **Step 1: `robot.hpp` に定数と首モータ A / B の対応を書く**
- [ ] **Step 2: `comm` に `BODY_POSE` の受け口と最終受信時刻、`error_flags` を足す**
- [ ] **Step 3: `Controller` と `CanScheduler` を実装し、`main.cpp` を組み立てだけにする**
- [ ] **Step 4: ビルド** Run（PowerShell）: `pio run -d fw_ws/gamma_turret_fw` Expected: SUCCESS、警告なし
- [ ] **Step 5: README 更新（構成図、定数の意味、実機確認の手順）、コミット（承認後）** `制御本体の構成に組み替える`

---

### Task 8（帰宅後、実機）: 脚 4 本での確認

- [ ] 24V + USB で起動。起動時に脚が動かないこと（現在位置を保持）。`error_flags` と `JOINT_STATE` が PC に出ること（Jetson 接続前は USB-UART + ROS 2）。
- [ ] `reference_command` で `body_height` を 0 → 0.5 → 1 → 0（しゃがみ）。各脚が `BodyPosture` の計算どおりの角度に動き、ロール / ピッチで左右・前後の差が出ること。
- [ ] 指令を止めて 0.5 s 後に途絶の扱いになり、脚が据え置き、`error_flags` の bit5 が立つこと。再開で復帰。
- [ ] 1 本の脚を手で押してトルク上限を超えさせ、その脚だけ Disable（LED が変わる）になること。
- [ ] 結果（必要なゲイン・ramp_rate・tff の値）を README に記録し、`robot.hpp` を更新。
