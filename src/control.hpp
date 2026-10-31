// Controller: PC の指令（Comm）、モータのフィードバック（CAN 受信タスク）、IMU（ImuStore）を持ち、
// 送信スケジューラから呼ばれて関節ごとの CAN 指令をその場で作る。
//   車輪: OmniDrive（視点基準）+ 無限回転 → 速度指令
//   脚:   BodyPosture の目標 → LegDamper → MIT 指令。トルク / 温度超過でディセーブル
//   首 Yaw: Gimbal（視点の目標 − 胴体の向き）→ PdGravityFf → LK トルク
//   首 Pitch / Roll: 機構が決まるまで送らない
// 共有状態は短い critical section（スピンロック）で守る。critical section の中では Comm の mutex を取らない
// （Comm は Tick / ImuTask / CanReceiver のタスク文脈からだけ呼ぶ）
#ifndef GAMMA_TURRET_FW__CONTROL_HPP_
#define GAMMA_TURRET_FW__CONTROL_HPP_

#include <cstdint>

#include <freertos/FreeRTOS.h>

#include <rabcl/controller/body_attitude.hpp>
#include <rabcl/controller/body_posture.hpp>
#include <rabcl/controller/command_watchdog.hpp>
#include <rabcl/controller/gimbal.hpp>
#include <rabcl/controller/leg_damper.hpp>
#include <rabcl/controller/omni_drive.hpp>
#include <rabcl/controller/pd_gravity_ff.hpp>
#include <rabcl/protocol/protocol.hpp>
#include <rabcl/utils/type.hpp>
#include <rabcl_esp32/can_bus.hpp>

#include "comm.hpp"
#include "imu.hpp"
#include "robot.hpp"

class Controller
{
public:
  // ログ用の関節状態
  struct JointStatus
  {
    rabcl::MotorInfo info;
    uint8_t err;      // DM: フィードバック D[0] 上位 4 bit。LK: 0
    bool enabled;     // DM: err == 1
    bool tripped;     // 脚: トルク / 温度超過でディセーブル中
    bool stale;       // フィードバックが FEEDBACK_TIMEOUT_MS 止まっている（モータの電源断）
    uint32_t rx;      // 受けたフィードバック数
    float target;     // 脚: ランプ後の目標角。首 Yaw: 関節角の目標
  };

  Controller(Comm & comm, ImuStore & imu);

  // CAN 受信タスクから。関節のフィードバックを置き、脚の安全策を判定する
  void SetFeedback(robot::Joint joint, const rabcl::MotorInfo & info, uint8_t err);
  // 送信スケジューラから。false = このスロットでは送らない（未イネーブル、機構未定、指令未受信）
  bool BuildCommand(robot::Joint joint, int64_t now_us, rabcl_esp32::CanFrame & out);
  // 送信スケジューラから。DM モータで、トリップしていなければ true（1 Hz のイネーブル対象）
  bool NeedsEnable(robot::Joint joint) const;
  // 制御タスクから（100 Hz）。Comm の指令を取り込み、途絶監視と胴体姿勢の更新をする
  void Tick(int64_t now_us);

  uint32_t error_flags() const {return error_flags_;}
  bool command_lost() const {return lost_;}
  bool motor_power_lost() const {return motor_power_lost_;}
  float yaw_angle() const;  // 首 Yaw の関節角（最後のフィードバック）。IMU タスクが計測時刻の値として使う
  JointStatus GetStatus(robot::Joint joint) const;

private:
  struct JointState
  {
    rabcl::MotorInfo info = {};
    uint8_t err = 0;
    bool enabled = false;
    bool tripped = false;
    bool reanchor = false;  // 脚: イネーブルが切 → 入になった（LegDamper を現在角に合わせ直す）
    bool stale = false;     // フィードバック途絶（Tick が判定。復帰のフィードバックで解除）
    int64_t last_fb_us = 0;
    uint32_t rx = 0;
    float target = 0.0f;
  };

  struct LegControl
  {
    explicit LegControl(const rabcl::LegDamperConfig & config) : damper(config) {}
    rabcl::LegDamper damper;
    int64_t last_cmd_us = 0;
  };

  // Tick が作り BuildCommand が読む指令のスナップショット
  struct CommandSnapshot
  {
    rabcl::protocol::ChassisVel chassis_vel = {};
    rabcl::protocol::BodyPose body_pose = {};
    bool body_pose_present = false;
    rabcl::protocol::JointCmd yaw_cmd = {};
    bool yaw_cmd_present = false;
    bool infinite_rotation = false;
    bool lost = true;
    bool wheels_powered = false;  // 車輪のどれかからフィードバックが来ている（旋回速度の積分に使う）
    rabcl::Attitude body = {0.0f, 0.0f, 0.0f};
  };

  static constexpr float MAX_DT_S = 0.05f;  // ランプ・積分の dt 上限（スロットが飛んだとき用）
  static constexpr uint8_t DM_ERR_ENABLED = 1;
  static constexpr uint8_t DM_ERR_FAULT_MIN = 8;  // 8〜E はモータ側の異常（過電圧 / 過電流 / 過熱 / 過負荷など）

  bool BuildWheel(robot::Joint joint, const CommandSnapshot & cmd, rabcl_esp32::CanFrame & out);
  bool BuildLeg(
    robot::Joint joint, int64_t now_us, const CommandSnapshot & cmd, rabcl_esp32::CanFrame & out);
  bool BuildYaw(const CommandSnapshot & cmd, rabcl_esp32::CanFrame & out);
  float ChassisWz(const CommandSnapshot & cmd) const;

  Comm & comm_;
  ImuStore & imu_;
  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;

  JointState joints_[robot::JOINT_COUNT];
  LegControl legs_[4];
  CommandSnapshot snapshot_;

  rabcl::OmniDrive omni_;
  rabcl::BodyPosture posture_;
  rabcl::BodyAttitude attitude_;
  rabcl::Gimbal gimbal_;
  rabcl::PdGravityFf yaw_pd_;
  rabcl::CommandWatchdog watchdog_;

  int64_t last_tick_us_ = 0;
  int64_t fed_rx_time_us_ = 0;
  int64_t last_imu_time_us_ = 0;
  bool imu_used_ = false;  // 前回の Tick で IMU 経路を使ったか（切 → 入 で AnchorImu）
  volatile bool lost_ = true;
  volatile bool motor_power_lost_ = false;
  volatile uint32_t error_flags_ = 0;
};

#endif  // GAMMA_TURRET_FW__CONTROL_HPP_
