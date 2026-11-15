// gamma の機体定義（関節番号、モード番号、関節とモータの対応、制御定数）。
// 関節番号は rabbits_bridge の config/gamma.yaml の joints の並びと同じにする。
// ピンや CAN ID の値そのものは board.hpp に置く。制御定数は設計書 4.5（最初の値は仮。実機で合わせる）
#ifndef GAMMA_TURRET_FW__ROBOT_HPP_
#define GAMMA_TURRET_FW__ROBOT_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <rabcl/controller/leg_linkage.hpp>

#include "board.hpp"

namespace robot
{

constexpr char FIRMWARE_ID[8] = "gamma";

// 関節番号（rabcl::protocol の JointState / JointCmd の id）
enum Joint : uint8_t
{
  YAW = 0,   // yaw_joint（MHF5005、CAN1）
  PITCH,     // pitch_joint（DM3507、CAN1）
  ROLL,      // roll_joint。モータは無い（番号は PC 側の関節の並びと対なので残す。送らない）
  WHEEL_FR,  // right_front_wheel_joint（DM2325、CAN2）
  WHEEL_FL,  // left_front_wheel_joint
  WHEEL_BR,  // right_back_wheel_joint
  WHEEL_BL,  // left_back_wheel_joint
  LEG_FR,    // right_front_leg_joint（DM4310、CAN2）
  LEG_FL,    // left_front_leg_joint
  LEG_BR,    // right_back_leg_joint
  LEG_BL,    // left_back_leg_joint
  JOINT_COUNT
};

// モード番号（rabcl::protocol の ModeCmd の id）。config/gamma.yaml の modes の並び
enum Mode : uint8_t
{
  LOAD = 0,   // 0: 停止, 1: 正転, 2: 逆転
  FIRE,       // 0: 停止, 1: 射出
  SPEED,      // 0: 歩き, 1: 走り（速度の倍率は PC 側。ファームは保持するだけ）
  CHASSIS,    // 0: 通常, 1: 無限回転
  STABILIZE,  // 0: 首の安定化なし, 1: あり（既定）
  MODE_COUNT
};

enum class MotorType : uint8_t
{
  LK_MHF5005,
  DM3507,
  DM2325,
  DM4310,
};

enum class Bus : uint8_t
{
  CAN1,
  CAN2,
};

struct JointSpec
{
  Bus bus;
  MotorType motor;
  uint32_t can_id;  // 指令 ID（DAMIAO は CAN_ID、LK は 0x140 + ID）
  uint32_t mst_id;  // フィードバック ID（DAMIAO は MST_ID、LK は指令と同じ）
};

constexpr JointSpec JOINTS[JOINT_COUNT] = {
  {Bus::CAN1, MotorType::LK_MHF5005, board::YAW_ID, board::YAW_ID},
  {Bus::CAN1, MotorType::DM3507, board::PITCH_CAN_ID, board::PITCH_MST_ID},
  {Bus::CAN1, MotorType::DM3507, board::ROLL_CAN_ID, board::ROLL_MST_ID},
  {Bus::CAN2, MotorType::DM2325, board::CHASSIS_CAN_ID[0], board::CHASSIS_MST_ID[0]},
  {Bus::CAN2, MotorType::DM2325, board::CHASSIS_CAN_ID[1], board::CHASSIS_MST_ID[1]},
  {Bus::CAN2, MotorType::DM2325, board::CHASSIS_CAN_ID[2], board::CHASSIS_MST_ID[2]},
  {Bus::CAN2, MotorType::DM2325, board::CHASSIS_CAN_ID[3], board::CHASSIS_MST_ID[3]},
  {Bus::CAN2, MotorType::DM4310, board::LEG_CAN_ID[0], board::LEG_MST_ID[0]},
  {Bus::CAN2, MotorType::DM4310, board::LEG_CAN_ID[1], board::LEG_MST_ID[1]},
  {Bus::CAN2, MotorType::DM4310, board::LEG_CAN_ID[2], board::LEG_MST_ID[2]},
  {Bus::CAN2, MotorType::DM4310, board::LEG_CAN_ID[3], board::LEG_MST_ID[3]},
};

constexpr bool IsDm(MotorType motor)
{
  return motor != MotorType::LK_MHF5005;
}

constexpr bool IsLeg(Joint joint)
{
  return joint >= LEG_FR && joint <= LEG_BL;
}

constexpr bool IsWheel(Joint joint)
{
  return joint >= WHEEL_FR && joint <= WHEEL_BL;
}

// フィードバック ID から関節を引く。無ければ -1
inline int JointForFeedbackId(Bus bus, uint32_t id)
{
  for (uint8_t j = 0; j < JOINT_COUNT; ++j) {
    if (JOINTS[j].bus == bus && JOINTS[j].mst_id == id) {
      return j;
    }
  }
  return -1;
}

// 送信の並び（等間隔に 1 スロット 1 関節）。CAN2 は車輪と脚を交互に
constexpr size_t CAN2_JOINT_COUNT = 8;
constexpr Joint CAN2_ORDER[CAN2_JOINT_COUNT] = {
  WHEEL_FR, LEG_FR, WHEEL_FL, LEG_FL, WHEEL_BR, LEG_BR, WHEEL_BL, LEG_BL};
constexpr size_t CAN1_JOINT_COUNT = 2;
constexpr Joint CAN1_ORDER[CAN1_JOINT_COUNT] = {YAW, PITCH};  // Roll はモータが無い
constexpr uint32_t CAN2_SLOT_US = 500;  // 8 関節 → 各 250 Hz
constexpr uint32_t CAN1_SLOT_US = 500;  // 2 関節 → 各 1000 Hz（MHF5005 が全部返すかは実機で確認。足りなければ 1000 µs）
constexpr uint32_t DM_ENABLE_PERIOD_MS = 1000;  // DM モータへイネーブルを送る周期

// --- 制御定数（設計書 4.5。実機で合わせる） ---
constexpr float PI_F = static_cast<float>(M_PI);

// 脚（DM4310、MIT）。4 節リンクの寸法 [m / rad] は rabbits_description の config/gamma_linkage.yaml と **対で更新する**
// （記号は同リポジトリの docs/gamma_leg_linkage.svg、設計書は rabcl docs/superpowers/specs/2026-10-25-gamma-leg-linkage-design.md）
constexpr rabcl::LegLinkageConfig LEG_LINKAGE = {
  0.041f,         // crank_length Lc
  PI_F / 3.0f,    // crank_angle_zero α0（θ = 0 でのクランクの向き）
  0.03975f,       // coupler_length Lr
  0.060f,         // bar_length Lb
  -0.026f, -0.049f,  // bar_pivot B1（モータ軸から x, z）
  0.0f, 0.0175f,     // coupler_pin H1 → J
  0.0383f, -0.044f,  // wheel H1 → W（外へ 38.3、下へ 44）
  PI_F / 2.0f};   // theta_max（メカスト間 90°）
constexpr float LEG_DROP_MIN = 0.06906f;  // [m] 車高 0 の車輪の下がり（θ = 0.15 rad。メカストの余裕込み）
constexpr float LEG_DROP_MAX = 0.11527f;  // [m] 車高 1（θ = 1.42 rad）
constexpr float LEG_TILT_RADIUS = 0.164f;  // [m] 車体中心から車輪の接地点まで（傾き → 脚の下がりの差）
constexpr float LEG_AZIMUTH[4] = {-PI_F / 4.0f, PI_F / 4.0f, -3.0f * PI_F / 4.0f, 3.0f * PI_F / 4.0f};  // FR, FL, BR, BL
constexpr float LEG_KP = 1.0f;           // [Nm/rad]
constexpr float LEG_KD = 0.1f;           // [Nm·s/rad]
constexpr float LEG_LOAD_FORCE = 0.0f;   // [N] 脚 1 本の荷重（車体質量の分担。組み上がってから取り直す）。トルクは × dd/dθ
constexpr float LEG_FRICTION_FF = 0.25f;  // [Nm] 摩擦分（脚モータ単体の実測 0.2〜0.3）
constexpr float LEG_FRICTION_BAND = 0.04f;  // [rad] 摩擦分を差に比例させる幅（局所の傾き 0.25 / 0.04 ≈ 6 Nm/rad）。縁の振動対策（2026-11-01）
constexpr float LEG_RAMP_RATE = 4.0f;    // [rad/s] 目標の移動速度（しゃがみ 0.17 s）。0.5 / 1.0 / 2.0 は実機で「遅い」（2026-11-01）
constexpr float LEG_TORQUE_LIMIT = 4.0f;  // [Nm] 超えたらその脚をディセーブル（DM4310 定格 10 Nm）
constexpr float LEG_TEMP_LIMIT = 80.0f;   // [℃] ロータ温度

// 首 Yaw（MHF5005、LK トルク指令。beta の yaw_pd）
constexpr float YAW_ZERO_OFFSET = 0.0f;  // [rad] エンコーダ 0 と機体前方のずれ。首の配線後に実測して入れる（beta は 240°）
// モータ単体（無負荷）で決めた値（2026-11-15）。beta の 5000 / 600 / 2000 は砲塔とギアが付いた状態では発振せず、
// 単体では激しく発振した。機体に載せたら慣性に合わせて再調整する（上限はバッテリー運転で上げる）
constexpr float YAW_KP = 50.0f;
constexpr float YAW_KD = 5.0f;
constexpr float YAW_GRAVITY_FF = 0.0f;
constexpr float YAW_OUTPUT_MAX = 100.0f;  // LK のトルク指令値（電流の LSB 単位 0.01 A）
// 目標の移動速度の上限 [rad/s]。起動時 / 復電時は現在角から始めるので、目標との差が大きくても全力で突進しない
// （2026-11-15、机上で 1 A 制限の電源が落ちた）。無限回転中の関節目標は SPIN_SPEED + CHASSIS_ROT_MAX（8 rad/s）で
// 動くので、それより大きくしておく
constexpr float YAW_RAMP_RATE = 10.0f;

// 首 Pitch（DM3507、MIT 位置制御。beta の値）。モータのゼロは DM のツール（SaveZero）で機体の水平に合わせる。
// 向きの符号（REP-103: 前が下がると正）は機体に載せてから確定する
constexpr float PITCH_KP = 4.0f;    // [Nm/rad] MIT の剛性（0〜500。LSB 約 0.12）
constexpr float PITCH_KD = 0.2f;    // [Nm s/rad] MIT の減衰（0〜5）
constexpr float PITCH_GRAVITY_FF = 0.0f;  // [Nm] tff = PITCH_GRAVITY_FF × sin(現在角)
constexpr float PITCH_MIN = -PI_F / 12.0f;  // [rad] 見上げ −15°（URDF の pitch_joint と対）
constexpr float PITCH_MAX = PI_F / 6.0f;    // [rad] 見下ろし +30°

// 車輪（DM2325、速度モード。beta の omni_drive）
constexpr double WHEEL_D = 0.075;  // [m] 車輪径（φ75）
constexpr double BODY_D = 0.328;   // [m] 車輪の接地点の配置径（= 2 × LEG_TILT_RADIUS、中間の車高。車高で ±2% 変わるのは無視）
constexpr float WHEEL_SIGN = -1.0f;  // beta と同じ回転向き
constexpr float CHASSIS_VEL_MAX = 1.0f;  // [m/s] ファーム側の上限（操作の上限は PC 側の launch）
constexpr float CHASSIS_ROT_MAX = 3.0f;  // [rad/s]
constexpr float SPIN_SPEED = 5.0f;       // [rad/s] 無限回転の旋回（一定。beta の最高速度）

// IMU（BNO055、UART）。設計書 rabcl docs/superpowers/specs/2026-10-24-bno055-uart-design.md 5.2
// 軸の入れ替えは BNO055 の AXIS_MAP で行い、出力をカメラ座標系（x 前、y 左、z 上）にする。
// 実機で確定するまで既定値（本番: 部品面が後ろ・USB-C が左 / 試験: 部品面が上・USB-C が右。README の手順で合わせる）
// 試験向き（部品面が上・USB-C が右・USB-C を右にして座ったときの奥を前）で確定（2026-11-01）。
// 本番向き（部品面が後ろ・USB-C が左）に載せたら再確定する。試験向きからの計算値は CONFIG 0x06 / SIGN 0x07 の見込み（未確認）
constexpr uint8_t IMU_AXIS_MAP_CONFIG = 0x24;   // X→X, Y→Y, Z→Z
constexpr uint8_t IMU_AXIS_MAP_SIGN = 0x00;
constexpr float IMU_PITCH_SIGN = 1.0f;          // BNO055 の pitch → REP-103（前が下がると正）。試験向きで確定（前を下げて +0.19 rad）
constexpr float IMU_ROLL_SIGN = -1.0f;          // BNO055 の roll → REP-103（右が下がると正）。試験向きで確定（右を下げて BNO055 は −0.25 rad）
// ジャイロ → 機体座標（x 前、y 左、z 上）。body[i] = IMU_GYRO_SIGN[i] * sample.gyro[IMU_GYRO_INDEX[i]]。
// 試験向きの実測: 前を下げると gyro[0] が正、右を下げると gyro[1] が正、左回りで gyro[2] が正（BNO055 の X は左右軸、Y は前後軸）
constexpr int IMU_GYRO_INDEX[3] = {1, 0, 2};
constexpr float IMU_GYRO_SIGN[3] = {1.0f, 1.0f, 1.0f};
constexpr uint8_t IMU_OPERATION_MODE = 0x08;    // IMU（加速度 + ジャイロ、磁気なし）
constexpr bool IMU_USE_EXTERNAL_CRYSTAL = true;  // AE-BNO055-BO は XT1（FC-135 32.768 kHz）を XIN32/XOUT32 に実装（秋月説明書 2022-04-13 の回路図）
constexpr uint32_t IMU_SAMPLE_PERIOD_MS = 10;   // 融合出力 100 Hz
constexpr uint32_t IMU_RESPONSE_TIMEOUT_MS = 20;
constexpr uint32_t IMU_BOOT_WAIT_MS = 700;      // 電源投入 / ソフトリセットから最初の要求まで
constexpr uint32_t IMU_CLOCK_SETTLE_MS = 20;
constexpr uint8_t IMU_MAX_CONSECUTIVE_ERRORS = 10;

// 指令途絶
constexpr float WATCHDOG_S = 0.5f;

// モータの電源断（HP 0 で 24V だけ切れ、P4 は USB 給電で生きる場合）: フィードバックがこの時間止まったら
// 「電源が落ちた」とみなし、復電でフィードバックが戻るまで指令を送らない。脚は復帰時に現在角から（再イネーブルと同じ）
constexpr uint32_t FEEDBACK_TIMEOUT_MS = 200;

// SYSTEM の error_flags
constexpr uint32_t ERROR_LEG_TRIP_BIT = 0;   // bit0〜3 = 脚 FR, FL, BR, BL のトリップ
constexpr uint32_t ERROR_IMU_INVALID_BIT = 4;
constexpr uint32_t ERROR_CMD_LOST_BIT = 5;
constexpr uint32_t ERROR_MOTOR_POWER_BIT = 6;  // フィードバックのある DM モータすべてが途絶（24V 断）

}  // namespace robot

#endif  // GAMMA_TURRET_FW__ROBOT_HPP_
