#include "can_sched.hpp"

#include <cstring>

#include <esp_log.h>
#include <rabcl/interface/can.hpp>
#include <rabcl/protocol/protocol.hpp>
#include <rabcl/utils/type.hpp>
#include <rabcl/utils/utils.hpp>

namespace
{

constexpr const char * TAG = "can_sched";
constexpr uint8_t DM_ERR_FAULT_MIN = 8;

}  // namespace

// --- CanScheduler ---

CanScheduler::CanScheduler(
  rabcl_esp32::CanBus & can, Controller & controller,
  const robot::Joint * joints, size_t joint_count, uint32_t slot_us, const char * name)
: can_(can), controller_(controller), joints_(joints), joint_count_(joint_count),
  slot_us_(slot_us), name_(name)
{
  // 1 周（全関節 1 スロットずつ）の時間から、イネーブルを挟む周回数を決める
  const uint32_t round_us = slot_us_ * static_cast<uint32_t>(joint_count_);
  enable_every_rounds_ = round_us == 0 ? 1 : (robot::DM_ENABLE_PERIOD_MS * 1000) / round_us;
  if (enable_every_rounds_ == 0) {
    enable_every_rounds_ = 1;
  }
}

CanScheduler::~CanScheduler()
{
  if (timer_ != nullptr) {
    esp_timer_stop(timer_);
    esp_timer_delete(timer_);
  }
}

esp_err_t CanScheduler::Start()
{
  if (timer_ != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_timer_create_args_t args = {};
  args.callback = TimerCallback;
  args.arg = this;
  args.name = name_;
  const esp_err_t err = esp_timer_create(&args, &timer_);
  if (err != ESP_OK) {
    return err;
  }
  return esp_timer_start_periodic(timer_, slot_us_);
}

CanScheduler::Counters CanScheduler::counters() const
{
  return {sent_, send_fail_, skipped_, enables_};
}

void CanScheduler::TimerCallback(void * arg)
{
  static_cast<CanScheduler *>(arg)->OnSlot();
}

void CanScheduler::OnSlot()
{
  const robot::Joint joint = joints_[next_];
  if (++next_ >= joint_count_) {
    next_ = 0;
    ++round_;
  }
  rabcl_esp32::CanFrame frame;
  const bool enable_slot = (round_ % enable_every_rounds_) == 0;
  if (enable_slot && controller_.NeedsEnable(joint)) {
    frame.id = robot::JOINTS[joint].can_id;  // beta と同じく速度モードの車輪も CAN_ID へ送る
    frame.dlc = 8;
    frame.ext = false;
    frame.rtr = false;
    if (robot::IsDm(robot::JOINTS[joint].motor)) {
      rabcl::Can::PrepareDMMotorEnable(frame.data);
    } else {
      rabcl::Can::PrepareLKMotorReadMotorState2(frame.data);  // LK は聞かれた時だけ返す。最初のフィードバックを引き出す
    }
    enables_ = enables_ + 1;
  } else if (!controller_.BuildCommand(joint, esp_timer_get_time(), frame)) {
    skipped_ = skipped_ + 1;
    return;
  }
  if (can_.Send(frame, 0) == ESP_OK) {
    sent_ = sent_ + 1;
  } else {
    send_fail_ = send_fail_ + 1;
  }
}

// --- CanReceiver ---

CanReceiver::CanReceiver(
  rabcl_esp32::CanBus & can, robot::Bus bus, Controller & controller, Comm & comm,
  const char * task_name)
: can_(can), bus_(bus), controller_(controller), comm_(comm), task_name_(task_name)
{
}

CanReceiver::~CanReceiver()
{
  if (task_ != nullptr) {
    vTaskDelete(task_);
  }
}

esp_err_t CanReceiver::Start()
{
  if (task_ != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  if (xTaskCreate(TaskEntry, task_name_, TASK_STACK, this, 6, &task_) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

void CanReceiver::TaskEntry(void * arg)
{
  static_cast<CanReceiver *>(arg)->Task();
}

void CanReceiver::Task()
{
  rabcl_esp32::CanFrame frame;
  while (true) {
    if (can_.Receive(frame, portMAX_DELAY)) {
      HandleFrame(frame);
    }
  }
}

void CanReceiver::HandleFrame(const rabcl_esp32::CanFrame & frame)
{
  const int j = robot::JointForFeedbackId(bus_, frame.id);
  if (j < 0) {
    unknown_frames_ = unknown_frames_ + 1;
    ESP_LOGD(TAG, "rx (other): id=0x%03lX", static_cast<unsigned long>(frame.id));
    return;
  }
  const robot::Joint joint = static_cast<robot::Joint>(j);
  rabcl::MotorInfo info = {};
  uint8_t err = 0;
  switch (robot::JOINTS[joint].motor) {
    case robot::MotorType::LK_MHF5005:
      rabcl::Can::ParseLKMotorFeedback(frame.data, info);
      info.position_ = rabcl::Utils::WrapPi(info.position_ - robot::YAW_ZERO_OFFSET);
      break;
    case robot::MotorType::DM3507:
      rabcl::Can::ParseDMMotorFeedback(
        frame.data, info, rabcl::Can::DM3507_PMAX, rabcl::Can::DM3507_VMAX,
        rabcl::Can::DM3507_TMAX, 1.0f);
      err = rabcl::Can::DMFeedbackError(frame.data);
      break;
    case robot::MotorType::DM2325:
      rabcl::Can::ParseDMMotorFeedback(
        frame.data, info, rabcl::Can::DM2325_PMAX, rabcl::Can::DM2325_VMAX,
        rabcl::Can::DM2325_TMAX, rabcl::Can::DM2325_GR);
      err = rabcl::Can::DMFeedbackError(frame.data);
      break;
    case robot::MotorType::DM4310:
      rabcl::Can::ParseDMMotorFeedback(
        frame.data, info, rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX,
        rabcl::Can::DM4310_TMAX, 1.0f);
      err = rabcl::Can::DMFeedbackError(frame.data);
      break;
  }
  // DAMIAO のフィードバック先頭バイトは「CAN_ID の下位 8 bit | 状態 << 4」。CAN_ID が 16 以上（beta の 0x42〜0x46）だと
  // 状態の上位ニブルに CAN_ID の bit4〜7 が混ざるので落とす。0 / 1（停止 / イネーブル）は正しく戻り、異常（8 以上）の判定も
  // 効くが、異常コードの bit2 は区別できない（例: 過電圧 8 とロータ過熱 C が両方 8 に、低電圧 9 と通信途絶 D が両方 9 に見える）
  err &= static_cast<uint8_t>(~((robot::JOINTS[joint].can_id >> 4) & 0x0F));
  if (err != last_err_[joint]) {
    if (err >= DM_ERR_FAULT_MIN) {
      ESP_LOGW(TAG, "joint %u: motor fault code 0x%X", static_cast<unsigned>(joint), err);
    }
    last_err_[joint] = err;
  }
  controller_.SetFeedback(joint, info, err);

  if (++rx_count_[joint] % JOINT_STATE_DECIMATION == 0) {
    rabcl::protocol::JointState js = {};
    js.id = joint;
    js.position = info.position_;
    js.velocity = info.velocity_;
    js.effort = robot::IsDm(robot::JOINTS[joint].motor) ? info.torque_ : info.current_;
    js.temperature = info.temperature_;
    js.error = (robot::IsDm(robot::JOINTS[joint].motor) && err != 1) ? err : 0;
    comm_.SetJointState(js);
  }
}
