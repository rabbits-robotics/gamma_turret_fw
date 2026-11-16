#include "control.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <esp_timer.h>
#include <rabcl/interface/can.hpp>
#include <rabcl/utils/utils.hpp>

namespace proto = rabcl::protocol;

namespace
{

class Critical
{
public:
  explicit Critical(portMUX_TYPE & lock) : lock_(lock) {taskENTER_CRITICAL(&lock_);}
  ~Critical() {taskEXIT_CRITICAL(&lock_);}

private:
  portMUX_TYPE & lock_;
};

constexpr rabcl::LegDamperConfig LEG_CONFIG = {
  robot::LEG_KP, robot::LEG_KD, robot::LEG_FRICTION_FF, robot::LEG_FRICTION_BAND, robot::LEG_RAMP_RATE};

void FillFrame(uint32_t id, const uint8_t data[8], rabcl_esp32::CanFrame & out)
{
  out.id = id;
  out.dlc = 8;
  out.ext = false;
  out.rtr = false;
  std::memcpy(out.data, data, 8);
}

}  // namespace

Controller::Controller(Comm & comm, ImuStore & imu)
: comm_(comm), imu_(imu),
  legs_{LegControl(LEG_CONFIG), LegControl(LEG_CONFIG), LegControl(LEG_CONFIG), LegControl(LEG_CONFIG)},
  omni_(robot::WHEEL_D, robot::BODY_D),
  posture_(rabcl::BodyPostureConfig{
      robot::LEG_LINKAGE, robot::LEG_DROP_MIN, robot::LEG_DROP_MAX, robot::LEG_TILT_RADIUS,
      {robot::LEG_AZIMUTH[0], robot::LEG_AZIMUTH[1], robot::LEG_AZIMUTH[2], robot::LEG_AZIMUTH[3]}}),
  yaw_pd_(robot::YAW_KP, robot::YAW_KD, robot::YAW_GRAVITY_FF, robot::YAW_OUTPUT_MAX),
  watchdog_(robot::WATCHDOG_S)
{
}

// --- フィードバック ---

void Controller::SetFeedback(robot::Joint joint, const rabcl::MotorInfo & info, uint8_t err)
{
  if (joint >= robot::JOINT_COUNT) {
    return;
  }
  Critical critical(lock_);
  JointState & js = joints_[joint];
  js.info = info;
  js.err = err;
  const bool first_or_resumed = js.rx == 0 || js.stale;
  js.rx = js.rx + 1;
  js.last_fb_us = esp_timer_get_time();
  js.stale = false;
  const bool was_enabled = js.enabled;
  if (robot::IsDm(robot::JOINTS[joint].motor)) {
    js.enabled = err == DM_ERR_ENABLED;
  } else {
    js.enabled = true;
  }
  if (joint == robot::YAW && first_or_resumed) {
    js.reanchor = true;  // 起動時 / 復電時の最初の指令は現在角から（突進しない）
  }
  if (robot::IsLeg(joint)) {
    if (js.enabled && !was_enabled) {
      js.reanchor = true;  // 再イネーブル後の最初の指令は現在角から（段差を作らない）
    }
    // モータ側の異常コードは復帰させない（復帰は再起動）
    if (err >= DM_ERR_FAULT_MIN) {
      js.tripped = true;
    }
    if (js.enabled && !js.tripped) {
      const bool over_torque = std::fabs(info.torque_) > robot::LEG_TORQUE_LIMIT;
      const bool over_temp = info.temperature_ > robot::LEG_TEMP_LIMIT;
      if (over_torque || over_temp) {
        js.tripped = true;
      }
    }
  }
}

Controller::JointStatus Controller::GetStatus(robot::Joint joint) const
{
  JointStatus status = {};
  if (joint >= robot::JOINT_COUNT) {
    return status;
  }
  Critical critical(lock_);
  const JointState & js = joints_[joint];
  status.info = js.info;
  status.err = js.err;
  status.enabled = js.enabled;
  status.tripped = js.tripped;
  status.stale = js.stale;
  status.rx = js.rx;
  status.target = js.target;
  return status;
}

float Controller::yaw_angle() const
{
  Critical critical(lock_);
  return joints_[robot::YAW].info.position_;
}

bool Controller::NeedsEnable(robot::Joint joint) const
{
  if (joint >= robot::JOINT_COUNT || joint == robot::ROLL) {
    return false;  // Roll はモータが無い
  }
  Critical critical(lock_);
  const JointState & js = joints_[joint];
  if (!robot::IsDm(robot::JOINTS[joint].motor)) {
    return js.rx == 0 || js.stale;  // LK: 最初の / 復電後のフィードバックを引き出す状態読み出し
  }
  return !js.tripped;
}

// --- 指令 ---

bool Controller::BuildCommand(robot::Joint joint, int64_t now_us, rabcl_esp32::CanFrame & out)
{
  CommandSnapshot cmd;
  {
    Critical critical(lock_);
    cmd = snapshot_;
  }
  if (robot::IsWheel(joint)) {
    return BuildWheel(joint, cmd, out);
  }
  if (robot::IsLeg(joint)) {
    return BuildLeg(joint, now_us, cmd, out);
  }
  if (joint == robot::YAW) {
    return BuildYaw(now_us, cmd, out);
  }
  if (joint == robot::PITCH) {
    return BuildPitch(cmd, out);
  }
  return false;  // ROLL: モータが無い
}

float Controller::ChassisWz(const CommandSnapshot & cmd) const
{
  if (cmd.lost) {
    return 0.0f;
  }
  float wz = std::clamp(cmd.chassis_vel.wz, -robot::CHASSIS_ROT_MAX, robot::CHASSIS_ROT_MAX);
  if (cmd.infinite_rotation) {
    wz += robot::SPIN_SPEED;
  }
  return wz;
}

bool Controller::BuildWheel(
  robot::Joint joint, const CommandSnapshot & cmd, rabcl_esp32::CanFrame & out)
{
  float vx = 0.0f;
  float vy = 0.0f;
  if (!cmd.lost) {
    vx = std::clamp(cmd.chassis_vel.vx, -robot::CHASSIS_VEL_MAX, robot::CHASSIS_VEL_MAX);
    vy = std::clamp(cmd.chassis_vel.vy, -robot::CHASSIS_VEL_MAX, robot::CHASSIS_VEL_MAX);
  }
  const float wz = ChassisWz(cmd);
  float yaw_angle;
  uint32_t rx;
  bool stale;
  {
    Critical critical(lock_);
    yaw_angle = joints_[robot::YAW].info.position_;
    rx = joints_[joint].rx;
    stale = joints_[joint].stale;
  }
  if (rx == 0 || stale) {
    return false;  // フィードバックが来るまで / 途絶中（電源断）は送らない
  }
  double wheel[4];
  omni_.CalcVel(vx, vy, wz, wheel[0], wheel[1], wheel[2], wheel[3], yaw_angle);
  const float v = robot::WHEEL_SIGN * static_cast<float>(wheel[joint - robot::WHEEL_FR]);

  uint8_t data[8];
  rabcl::Can::PrepareDMMotorVelocityCmd(v, data);
  FillFrame(board::DM_VELOCITY_CMD_BASE + robot::JOINTS[joint].can_id, data, out);
  {
    Critical critical(lock_);
    joints_[joint].target = v;
    joints_[joint].effort = 0.0f;
    joints_[joint].commanded = true;
  }
  return true;
}

bool Controller::BuildLeg(
  robot::Joint joint, int64_t now_us, const CommandSnapshot & cmd, rabcl_esp32::CanFrame & out)
{
  const size_t i = joint - robot::LEG_FR;
  LegControl & leg = legs_[i];
  JointState state;
  float yaw_angle;
  {
    Critical critical(lock_);
    state = joints_[joint];
    yaw_angle = joints_[robot::YAW].info.position_;
  }
  uint8_t data[8];
  if (state.tripped) {
    // トルク / 温度超過: ディセーブルを送り続ける（モータの LED で分かる。復帰は再起動）
    rabcl::Can::PrepareDMMotorDisable(data);
    FillFrame(robot::JOINTS[joint].can_id, data, out);
    return true;
  }
  if (!state.enabled || state.rx == 0 || state.stale) {
    return false;  // イネーブル応答が来るまで / 途絶中（電源断）は MIT 指令を送らない
  }
  if (state.reanchor) {
    // 起動時と再イネーブル時: 今の位置を目標にして保持（勝手に動かない。目標はここからランプで動く）
    leg.damper.Reset(state.info.position_);
    leg.last_cmd_us = now_us;
    Critical critical(lock_);
    joints_[joint].reanchor = false;
  }
  float target = leg.damper.ramped_target();
  if (cmd.body_pose_present) {
    float targets[4];
    posture_.Solve(
      cmd.body_pose.height, cmd.body_pose.roll, cmd.body_pose.pitch, yaw_angle, targets);
    target = targets[i];
  }
  float dt = static_cast<float>(now_us - leg.last_cmd_us) / 1e6f;
  leg.last_cmd_us = now_us;
  dt = std::clamp(dt, 0.0f, MAX_DT_S);
  // 荷重分の tff = 荷重 [N] × 車輪の下がりのヤコビアン [m/rad]（姿勢で変わる）
  const float load_torque =
    robot::LEG_LOAD_FORCE * posture_.linkage().DropJacobian(state.info.position_);
  const rabcl::MitCommand mit =
    leg.damper.Update(target, state.info.position_, state.info.velocity_, dt, load_torque);
  rabcl::Can::PrepareDMMotorMITCmd(
    mit.pos, mit.vel, mit.kp, mit.kd, mit.tff,
    rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, data);
  FillFrame(robot::JOINTS[joint].can_id, data, out);
  {
    Critical critical(lock_);
    joints_[joint].target = mit.pos;
    joints_[joint].effort = mit.tff;
    joints_[joint].commanded = true;  // トリップ時のディセーブル送信では立てない（指令ではない）
  }
  return true;
}

bool Controller::BuildYaw(int64_t now_us, const CommandSnapshot & cmd, rabcl_esp32::CanFrame & out)
{
  JointState state;
  {
    Critical critical(lock_);
    state = joints_[robot::YAW];
    joints_[robot::YAW].reanchor = false;
  }
  if (state.rx == 0 || state.stale) {
    return false;  // フィードバックが無い / 古い（電源断）間はトルクを出さない（1 Hz の状態読み出しが呼び起こす）
  }
  uint8_t data[8];
  if (state.reanchor || !cmd.yaw_cmd_present) {
    // 起動時 / 復電時、および目標が無い間はランプを現在角に置く（次の指令は今の位置から始まる）
    yaw_ramp_ = state.info.position_;
    yaw_last_us_ = now_us;
  }
  if (!cmd.yaw_cmd_present) {
    // PC から視点の目標が来るまでトルクは出さず、状態読み出しでフィードバックだけ続ける（PC に関節角を返す）
    rabcl::Can::PrepareLKMotorReadMotorState2(data);
    FillFrame(robot::JOINTS[robot::YAW].can_id, data, out);
    return true;
  }
  // 視点の目標（地面基準）− 胴体の向き = 首の関節角。途絶中は最後の目標を保持。
  // 目標へは YAW_RAMP_RATE で寄せる（差が大きくても全力で突進しない）
  const rabcl::Attitude desired = {cmd.yaw_cmd.position, 0.0f, 0.0f};
  const float goal = gimbal_.Solve(desired, cmd.body).yaw;
  const float dt = std::clamp(static_cast<float>(now_us - yaw_last_us_) / 1e6f, 0.0f, MAX_DT_S);
  yaw_last_us_ = now_us;
  const float step = robot::YAW_RAMP_RATE * dt;
  const float diff = std::clamp(rabcl::Utils::WrapPi(goal - yaw_ramp_), -step, step);
  yaw_ramp_ = rabcl::Utils::WrapPi(yaw_ramp_ + diff);
  const float target = yaw_ramp_;
  const float torque = yaw_pd_.CalcAngular(target, state.info.position_, state.info.velocity_);
  rabcl::Can::PrepareLKMotorTorqueCmd(static_cast<int16_t>(torque), data);
  FillFrame(robot::JOINTS[robot::YAW].can_id, data, out);
  {
    Critical critical(lock_);
    joints_[robot::YAW].target = target;
    joints_[robot::YAW].effort = torque;
    joints_[robot::YAW].commanded = true;
  }
  return true;
}

bool Controller::BuildPitch(const CommandSnapshot & cmd, rabcl_esp32::CanFrame & out)
{
  JointState state;
  {
    Critical critical(lock_);
    state = joints_[robot::PITCH];
  }
  if (!state.enabled || state.rx == 0 || state.stale) {
    return false;  // イネーブル応答が来るまで / 途絶中（電源断）は MIT 指令を送らない
  }
  uint8_t data[8];
  if (!cmd.pitch_cmd_present) {
    // PC から視点の目標が来るまでトルクは出さない。ただし DM の CAN タイムアウト（通信途絶 D で赤点滅）に
    // 入らないよう、ゲイン 0 の MIT 指令を送り続ける（軸は自由。フィードバックも毎スロット返る）
    rabcl::Can::PrepareDMMotorMITCmd(
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      rabcl::Can::DM3507_PMAX, rabcl::Can::DM3507_VMAX, rabcl::Can::DM3507_TMAX, data);
    FillFrame(robot::JOINTS[robot::PITCH].can_id, data, out);
    return true;
  }
  // 視点の目標（地面基準）− 胴体の向き = 首の関節角。可動角に収める
  const rabcl::Attitude desired = {0.0f, cmd.pitch_cmd.position, 0.0f};
  const float target =
    std::clamp(gimbal_.Solve(desired, cmd.body).pitch, robot::PITCH_MIN, robot::PITCH_MAX);
  const float tff = robot::PITCH_GRAVITY_FF * std::sin(state.info.position_);
  rabcl::Can::PrepareDMMotorMITCmd(
    target, 0.0f, robot::PITCH_KP, robot::PITCH_KD, tff,
    rabcl::Can::DM3507_PMAX, rabcl::Can::DM3507_VMAX, rabcl::Can::DM3507_TMAX, data);
  FillFrame(robot::JOINTS[robot::PITCH].can_id, data, out);
  {
    Critical critical(lock_);
    joints_[robot::PITCH].target = target;
    joints_[robot::PITCH].effort = tff;
    joints_[robot::PITCH].commanded = true;
  }
  return true;
}

// --- 制御タスク（100 Hz） ---

void Controller::Tick(int64_t now_us)
{
  const float dt = last_tick_us_ == 0 ?
    0.0f : std::clamp(static_cast<float>(now_us - last_tick_us_) / 1e6f, 0.0f, MAX_DT_S);
  last_tick_us_ = now_us;

  // PC の指令を取り込む（Comm の mutex はここだけ）
  CommandSnapshot cmd;
  comm_.GetChassisVel(cmd.chassis_vel);
  cmd.body_pose_present = comm_.GetBodyPose(cmd.body_pose);
  cmd.yaw_cmd_present = comm_.GetJointCmd(robot::YAW, cmd.yaw_cmd);
  cmd.pitch_cmd_present = comm_.GetJointCmd(robot::PITCH, cmd.pitch_cmd);
  uint8_t mode = 0;
  cmd.infinite_rotation = comm_.GetMode(robot::CHASSIS, mode) && mode == 1;
  uint8_t fire = 0;
  cmd.fire = comm_.GetMode(robot::FIRE, fire) && fire == 1;
  uint8_t load = 0;
  cmd.load = comm_.GetMode(robot::LOAD, load) ? load : 0;
  uint8_t stabilize = 1;
  comm_.GetMode(robot::STABILIZE, stabilize);

  // 途絶監視
  const int64_t rx_time_us = comm_.last_rx_time_us();
  if (rx_time_us != 0 && rx_time_us != fed_rx_time_us_) {
    watchdog_.Feed(static_cast<float>(rx_time_us) / 1e6f);
    fed_rx_time_us_ = rx_time_us;
  }
  cmd.lost = watchdog_.Expired(static_cast<float>(now_us) / 1e6f);
  if (cmd.lost) {
    cmd.infinite_rotation = false;
    cmd.fire = false;
    cmd.load = 0;
  }
  fire_pulse_us_ = cmd.fire ? robot::FIRE_PULSE_US : robot::FIRE_IDLE_US;
  load_mode_ = cmd.load;

  // フィードバックの鮮度: FEEDBACK_TIMEOUT_MS 止まった関節は「電源断」扱い。DM はイネーブルも落とす
  // （復電でフィードバックが戻ると 切 → 入 になり、脚は現在角から合わせ直す）
  bool any_dm_seen = false;
  bool all_dm_stale = true;
  bool wheels_powered = false;
  float yaw_angle;
  {
    Critical critical(lock_);
    for (uint8_t j = 0; j < robot::JOINT_COUNT; ++j) {
      JointState & js = joints_[j];
      if (js.rx == 0) {
        continue;
      }
      if (now_us - js.last_fb_us > static_cast<int64_t>(robot::FEEDBACK_TIMEOUT_MS) * 1000) {
        js.stale = true;
        if (robot::IsDm(robot::JOINTS[j].motor)) {
          js.enabled = false;
        }
      }
      if (robot::IsDm(robot::JOINTS[j].motor)) {
        any_dm_seen = true;
        all_dm_stale = all_dm_stale && js.stale;
      }
      if (robot::IsWheel(static_cast<robot::Joint>(j)) && !js.stale) {
        wheels_powered = true;
      }
    }
    yaw_angle = joints_[robot::YAW].info.position_;
  }
  const bool motor_power_lost = any_dm_seen && all_dm_stale;
  cmd.wheels_powered = wheels_powered;

  // 胴体の向き: IMU が有効で安定化ありなら IMU − 首関節角、そうでなければ旋回速度の積分（yaw_ff 相当）。
  // yaw は無限回転中だけ追う（視点を世界に保つ）。普通の旋回（L2 / R2）では据え置き = 首は胴体と一緒に回る。
  // 車輪に電源が無い（回っていない）間は積分しない
  const ImuState imu = imu_.Get();
  attitude_.SetYawTracking(cmd.infinite_rotation);
  const bool use_imu = imu.valid && stabilize != 0;
  if (use_imu && !imu_used_) {
    // IMU が使えるようになった / 再初期化された / 安定化を入れた: IMU の yaw 基準（モード投入時 = 0）を
    // 今の推定に合わせて繋ぐ。これで切り替えや再初期化で砲塔が跳ばない
    attitude_.AnchorImu(imu.attitude, rabcl::NeckAngles{imu.neck_yaw, 0.0f, 0.0f});
    last_imu_time_us_ = 0;
  }
  imu_used_ = use_imu;
  if (use_imu) {
    if (imu.time_us != last_imu_time_us_) {
      // 計測時刻の首角（ImuTask が入れる）。IMU はカメラ側なので引いて胴体の向きにする
      attitude_.UpdateFromImu(imu.attitude, rabcl::NeckAngles{imu.neck_yaw, 0.0f, 0.0f});
      last_imu_time_us_ = imu.time_us;
    }
  } else if (cmd.wheels_powered) {
    attitude_.UpdateFromChassisRate(ChassisWz(cmd), dt);
  }
  cmd.body = attitude_.Get();

  // エラーフラグ
  uint32_t flags = 0;
  {
    Critical critical(lock_);
    for (size_t i = 0; i < 4; ++i) {
      if (joints_[robot::LEG_FR + i].tripped) {
        flags |= 1u << (robot::ERROR_LEG_TRIP_BIT + i);
      }
    }
    snapshot_ = cmd;
  }
  if (!imu.valid) {
    flags |= 1u << robot::ERROR_IMU_INVALID_BIT;
  }
  if (cmd.lost) {
    flags |= 1u << robot::ERROR_CMD_LOST_BIT;
  }
  if (motor_power_lost) {
    flags |= 1u << robot::ERROR_MOTOR_POWER_BIT;
  }
  lost_ = cmd.lost;
  motor_power_lost_ = motor_power_lost;
  error_flags_ = flags;
  comm_.SetErrorFlags(flags);

  // 前回の Tick 以降にモータへ送った指令を集めて PC に返す（MOTOR_COMMAND）。
  // 位置制御（脚・首 Yaw）は position、速度制御（車輪）は velocity、effort はフィードフォワード
  proto::MotorCommand motor_cmds[robot::JOINT_COUNT];
  uint8_t n_motor_cmds = 0;
  {
    Critical critical(lock_);
    for (uint8_t j = 0; j < robot::JOINT_COUNT; ++j) {
      JointState & js = joints_[j];
      if (!js.commanded) {
        continue;
      }
      js.commanded = false;
      proto::MotorCommand & mc = motor_cmds[n_motor_cmds++];
      mc.id = j;
      mc.position = robot::IsWheel(static_cast<robot::Joint>(j)) ? 0.0f : js.target;
      mc.velocity = robot::IsWheel(static_cast<robot::Joint>(j)) ? js.target : 0.0f;
      mc.effort = js.effort;
    }
  }
  if (n_motor_cmds > 0) {
    comm_.SetMotorCommands(motor_cmds, n_motor_cmds);  // Comm の mutex はタスク文脈（Tick）から。0 件なら未送信の組を消さない
  }
}
