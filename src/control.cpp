#include "control.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <esp_timer.h>
#include <rabcl/interface/can.hpp>

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
  js.rx = js.rx + 1;
  js.last_fb_us = esp_timer_get_time();
  js.stale = false;
  const bool was_enabled = js.enabled;
  if (robot::IsDm(robot::JOINTS[joint].motor)) {
    js.enabled = err == DM_ERR_ENABLED;
  } else {
    js.enabled = true;
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
  if (joint >= robot::JOINT_COUNT || !robot::IsDm(robot::JOINTS[joint].motor)) {
    return false;
  }
  if (joint == robot::PITCH || joint == robot::ROLL) {
    return false;  // 機構が決まるまで触らない
  }
  Critical critical(lock_);
  return !joints_[joint].tripped;
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
    return BuildYaw(cmd, out);
  }
  return false;  // PITCH / ROLL: 機構が決まるまで送らない
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
  }
  return true;
}

bool Controller::BuildYaw(const CommandSnapshot & cmd, rabcl_esp32::CanFrame & out)
{
  if (!cmd.yaw_cmd_present) {
    return false;  // PC から視点の目標が来るまで送らない
  }
  JointState state;
  {
    Critical critical(lock_);
    state = joints_[robot::YAW];
  }
  if (state.rx == 0 || state.stale) {
    return false;  // フィードバックが無い / 古い（電源断）間はトルクを出さない
  }
  // 視点の目標（地面基準）− 胴体の向き = 首の関節角。途絶中は最後の目標を保持
  const rabcl::Attitude desired = {cmd.yaw_cmd.position, 0.0f, 0.0f};
  const float target = gimbal_.Solve(desired, cmd.body).yaw;
  const float torque = yaw_pd_.CalcAngular(target, state.info.position_, state.info.velocity_);
  uint8_t data[8];
  rabcl::Can::PrepareLKMotorTorqueCmd(static_cast<int16_t>(torque), data);
  FillFrame(robot::JOINTS[robot::YAW].can_id, data, out);
  {
    Critical critical(lock_);
    joints_[robot::YAW].target = target;
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
  uint8_t mode = 0;
  cmd.infinite_rotation = comm_.GetMode(robot::CHASSIS, mode) && mode == 1;
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
  }

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
}
