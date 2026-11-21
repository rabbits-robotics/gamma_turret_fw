#include "feeder_task.hpp"

#include <esp_log.h>
#include <esp_timer.h>

#include "board.hpp"
#include "robot.hpp"

namespace
{

constexpr const char * TAG = "feeder";

uint32_t NowMs()
{
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

rabcl::Feeder::Mode ToFeederMode(uint8_t load)
{
  switch (load) {
    case 1: return rabcl::Feeder::Mode::FORWARD;
    case 2: return rabcl::Feeder::Mode::REVERSE;
    default: return rabcl::Feeder::Mode::STOP;
  }
}

}  // namespace

const char * FeederTask::StateName(rabcl::JamRecovery::State state)
{
  switch (state) {
    case rabcl::JamRecovery::State::NORMAL: return "NORMAL";
    case rabcl::JamRecovery::State::BACKING: return "BACKING";
    case rabcl::JamRecovery::State::RETRYING: return "RETRYING";
    case rabcl::JamRecovery::State::JAMMED: return "JAMMED";
    case rabcl::JamRecovery::State::RECONNECT: return "RECONNECT";
    default: return "?";
  }
}

FeederTask::FeederTask(const Controller & controller)
: controller_(controller),
  servo_(bus_, board::SERVO_ID, robot::FEED_RESPONSE_TIMEOUT_MS),
  feeder_(rabcl::FeederConfig{robot::FEED_COUNTS_PER_CELL, robot::FEED_PERIOD_MS / 1000.0f}),
  recovery_(rabcl::JamRecoveryConfig{robot::FEED_COUNTS_PER_CELL, robot::FEED_JAM_LOAD, robot::FEED_JAM_CURRENT,
      robot::FEED_JAM_READS, robot::FEED_QUEUE_LIMIT_CELLS, robot::FEED_RETRY_LIMIT,
      robot::FEED_RECOVERY_TIMEOUT_MS / 1000.0f, robot::FEED_SETTLE_COUNTS})
{
}

FeederTask::~FeederTask()
{
  if (task_ != nullptr) {
    vTaskDelete(task_);
  }
}

esp_err_t FeederTask::Start()
{
  if (task_ != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  const esp_err_t err = bus_.Start();
  if (err != ESP_OK) {
    return err;
  }
  if (xTaskCreate(TaskEntry, "feeder", TASK_STACK, this, TASK_PRIORITY, &task_) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

void FeederTask::TaskEntry(void * arg)
{
  static_cast<FeederTask *>(arg)->Task();
}

void FeederTask::Task()
{
  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TICK_MS));
    const uint32_t now_ms = NowMs();
    if (state_ == State::NOT_READY) {
      if (last_connect_ms_ == 0 || now_ms - last_connect_ms_ >= CONNECT_PERIOD_MS) {
        last_connect_ms_ = now_ms;
        if (Connect(static_cast<float>(esp_timer_get_time()) / 1e6f)) {
          taskENTER_CRITICAL(&lock_);
          state_ = State::READY;
          consecutive_fail_ = 0;
          taskEXIT_CRITICAL(&lock_);
          feeder_.Cancel();  // 接続前の押下は捨てる（勝手に回らない）
          ESP_LOGI(TAG, "READY (id=%u, mode was %u, torque on, pos=%ld align=%+d)", servo_.id(), mode_,
                   static_cast<long>(align_position_), align_steps_);
        }
      }
      continue;
    }
    Drive(now_ms);
  }
}

bool FeederTask::Connect(float now_s)
{
  if (!servo_.Ping()) {
    return false;
  }
  // 一度切る: 過負荷保護の解除と、トルク ON のまま位置モードに切り替えて古い目標角へ飛ぶのを防ぐ
  if (!servo_.SetTorque(false)) {
    return false;
  }
  uint8_t mode = 0xFF;
  if (!servo_.ReadU8(rabcl::St3215Protocol::REG_MODE, mode)) {
    return false;
  }
  taskENTER_CRITICAL(&lock_);
  mode_ = mode;
  taskEXIT_CRITICAL(&lock_);
  // 絶対角はステップモードでは読めない（トルク ON からの相対カウントしか返さない）。位置モードに切り替えて読み、戻す。
  // トルクは切れたまま（位置モードで古い目標へ動かない）。ステップに戻せなければトルクは入れない
  using Mode = rabcl::St3215Protocol::Mode;
  if (!servo_.SetMode(Mode::POSITION)) {
    return false;
  }
  rabcl::St3215Status st;
  const bool read_ok = servo_.ReadStatus(st);
  if (!servo_.SetMode(Mode::STEP) || !read_ok) {
    return false;
  }
  if (!servo_.SetTorque(true)) {
    return false;
  }
  // ジャム起因なら前（噛んだ球の方）へ進まず、後ろの穴に合わせる
  const int16_t align = jam_reconnect_ ?
    feeder_.AlignStepsBackward(st.position, robot::FEED_PHASE_OFFSET, robot::FEED_SETTLE_COUNTS) :
    feeder_.AlignSteps(st.position, robot::FEED_PHASE_OFFSET);
  if (align != 0 && !servo_.Move(align, robot::FEED_SPEED, robot::FEED_ACC)) {
    return false;
  }
  taskENTER_CRITICAL(&lock_);
  align_position_ = st.position;
  align_steps_ = align;
  taskEXIT_CRITICAL(&lock_);
  if (jam_reconnect_) {
    jam_reconnect_ = false;
    recovery_.ResumeAfterReconnect(now_s);
  } else {
    recovery_.Reset();
  }
  action_ = {};
  pending_move_ = 0;
  if (recovery_.state() == rabcl::JamRecovery::State::JAMMED) {
    action_.block_forward = true;
    ESP_LOGW(TAG, "jam: reconnected %u times without a clean cell: JAMMED (reverse to clear)", recovery_.tries());
  }
  return true;
}

void FeederTask::Drive(uint32_t now_ms)
{
  if (controller_.command_lost()) {
    feeder_.Cancel();  // 途絶: 保留の 1 発も出さない（load_mode() は 0 になっている）
  }
  const float now_s = static_cast<float>(esp_timer_get_time()) / 1e6f;
  bool failed = false;
  bool talked = false;
  // 状態読みと詰まりの判定を先に。あとの Feeder のステップが判定より前に出ると、戻しの k に数えられず残る
  if (now_ms - last_status_ms_ >= robot::FEED_STATUS_PERIOD_MS) {
    last_status_ms_ = now_ms;
    talked = true;
    rabcl::St3215Status st;
    const bool ok = servo_.ReadStatus(st);
    taskENTER_CRITICAL(&lock_);
    status_ok_ = ok;
    if (ok) {
      status_ = st;
    }
    taskEXIT_CRITICAL(&lock_);
    failed = !ok;
    if (ok && Recover(st, now_s, now_ms)) {
      return;  // 再接続へ
    }
  }
  if (pending_move_ != 0) {
    talked = true;
    failed = !SendRecoveryMove(pending_move_) || failed;
  }
  // 出させない向きは STOP に置き換える（ステップを作ってから捨てると Feeder の端数と勘定がずれる）
  rabcl::Feeder::Mode mode = ToFeederMode(controller_.load_mode());
  if ((action_.block_forward && mode == rabcl::Feeder::Mode::FORWARD) ||
    (action_.block_reverse && mode == rabcl::Feeder::Mode::REVERSE))
  {
    mode = rabcl::Feeder::Mode::STOP;
  }
  const int16_t steps = feeder_.Update(mode, now_s);
  if (steps < 0) {
    recovery_.NotifyReverse();  // 操作者優先: JAMMED の解除、復旧の中断
    action_.block_forward = false;
    pending_move_ = 0;
  }
  if (steps != 0) {
    talked = true;
    const uint16_t speed = steps > 0 ? robot::FEED_SPEED : robot::FEED_REVERSE_SPEED;
    if (servo_.Move(steps, speed, robot::FEED_ACC)) {
      taskENTER_CRITICAL(&lock_);
      ++moves_;
      taskEXIT_CRITICAL(&lock_);
    } else {
      failed = true;
      taskENTER_CRITICAL(&lock_);
      ++move_fail_;
      taskEXIT_CRITICAL(&lock_);
    }
  }
  if (!talked) {
    return;  // 何も話していない tick は失敗の判定に数えない
  }
  if (failed) {
    ++consecutive_fail_;
    if (consecutive_fail_ >= robot::FEED_FAIL_LIMIT) {
      taskENTER_CRITICAL(&lock_);
      state_ = State::NOT_READY;
      status_ok_ = false;
      taskEXIT_CRITICAL(&lock_);
      feeder_.Cancel();
      last_connect_ms_ = now_ms;
      ESP_LOGW(TAG, "NOT_READY: %lu consecutive failures (timeouts=%lu err=0x%02X). reconnecting",
               static_cast<unsigned long>(consecutive_fail_), static_cast<unsigned long>(servo_.timeouts()),
               servo_.last_error());
      consecutive_fail_ = 0;
    }
  } else {
    consecutive_fail_ = 0;
  }
}

bool FeederTask::Recover(const rabcl::St3215Status & st, float now_s, uint32_t now_ms)
{
  const auto before = recovery_.state();
  action_ = recovery_.Observe(st.position, st.load, st.current, servo_.last_error(), now_s);
  const auto after = recovery_.state();
  if (after != before) {
    ESP_LOGW(TAG, "jam: %s -> %s remaining=%d load=%d cur=%d move=%+d tries=%u", StateName(before),
             StateName(after), st.position, st.load, st.current, action_.move, recovery_.tries());
  }
  if (action_.move != 0) {
    SendRecoveryMove(action_.move);  // 通らなければ pending_move_ に残して次の tick で送り直す
  }
  if (!action_.reconnect) {
    return false;
  }
  servo_.SetTorque(false);  // 押し続けない。Connect でも切るので結果は見ない
  taskENTER_CRITICAL(&lock_);
  state_ = State::NOT_READY;
  status_ok_ = false;
  taskEXIT_CRITICAL(&lock_);
  feeder_.Cancel();
  pending_move_ = 0;
  last_connect_ms_ = now_ms - CONNECT_PERIOD_MS;  // 次の tick で Connect（1 s 待たない）
  consecutive_fail_ = 0;
  jam_reconnect_ = true;
  ESP_LOGW(TAG, "jam: torque off and reconnecting (err=0x%02X, tries=%u)", servo_.last_error(), recovery_.tries());
  return true;
}

// 復旧の Move（戻し / 進め直し）。通れば true。通らなければ pending_move_ に残す（送れないまま到達と見なすと、その分の球が出ない）
bool FeederTask::SendRecoveryMove(int16_t move)
{
  const uint16_t speed = move > 0 ? robot::FEED_SPEED : robot::FEED_REVERSE_SPEED;
  const bool ok = servo_.Move(move, speed, robot::FEED_ACC);
  taskENTER_CRITICAL(&lock_);
  if (ok) {
    ++recovery_moves_;
  } else {
    ++move_fail_;
  }
  taskEXIT_CRITICAL(&lock_);
  pending_move_ = ok ? 0 : move;
  return ok;
}

FeederTask::Counters FeederTask::counters() const
{
  Counters c;
  taskENTER_CRITICAL(&lock_);
  c.state = state_;
  c.moves = moves_;
  c.move_fail = move_fail_;
  c.mode = mode_;
  c.align_position = align_position_;
  c.align_steps = align_steps_;
  c.status_ok = status_ok_;
  c.status = status_;
  c.recovery_moves = recovery_moves_;
  taskEXIT_CRITICAL(&lock_);
  c.jam_state = recovery_.state();
  c.jam_tries = recovery_.tries();
  c.recoveries = recovery_.recoveries();
  c.jams = recovery_.jams();
  c.cells = feeder_.cells();
  c.sent_counts = feeder_.sent_counts();
  c.timeouts = servo_.timeouts();
  c.last_error = servo_.last_error();
  return c;
}
