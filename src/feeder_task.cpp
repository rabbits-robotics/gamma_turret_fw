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

FeederTask::FeederTask(const Controller & controller)
: controller_(controller),
  servo_(bus_, board::SERVO_ID, robot::FEED_RESPONSE_TIMEOUT_MS),
  feeder_(rabcl::FeederConfig{robot::FEED_COUNTS_PER_CELL, robot::FEED_PERIOD_MS / 1000.0f})
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
        if (Connect()) {
          taskENTER_CRITICAL(&lock_);
          state_ = State::READY;
          consecutive_fail_ = 0;
          taskEXIT_CRITICAL(&lock_);
          feeder_.Cancel();  // 接続前の押下は捨てる（勝手に回らない）
          ESP_LOGI(TAG, "READY (id=%u, mode=%u, torque on)", servo_.id(), mode_);
        }
      }
      continue;
    }
    Drive(now_ms);
  }
}

bool FeederTask::Connect()
{
  if (!servo_.Ping()) {
    return false;
  }
  uint8_t mode = 0xFF;
  if (!servo_.ReadU8(rabcl::St3215Protocol::REG_MODE, mode)) {
    return false;
  }
  taskENTER_CRITICAL(&lock_);
  mode_ = mode;
  taskEXIT_CRITICAL(&lock_);
  if (mode != static_cast<uint8_t>(rabcl::St3215Protocol::Mode::STEP)) {
    // EEPROM はここでは書かない（試験モードの `m 3` で設定する）
    ESP_LOGW(TAG, "servo mode is %u, expected %u (STEP). set it with the st3215_test build", mode,
             static_cast<unsigned>(rabcl::St3215Protocol::Mode::STEP));
    return false;
  }
  return servo_.SetTorque(true);
}

void FeederTask::Drive(uint32_t now_ms)
{
  if (controller_.command_lost()) {
    feeder_.Cancel();  // 途絶: 保留の 1 発も出さない（load_mode() は 0 になっている）
  }
  const float now_s = static_cast<float>(esp_timer_get_time()) / 1e6f;
  const int16_t steps = feeder_.Update(ToFeederMode(controller_.load_mode()), now_s);
  bool failed = false;
  if (steps != 0) {
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
  if (now_ms - last_status_ms_ >= robot::FEED_STATUS_PERIOD_MS) {
    last_status_ms_ = now_ms;
    rabcl::St3215Status st;
    const bool ok = servo_.ReadStatus(st);
    taskENTER_CRITICAL(&lock_);
    status_ok_ = ok;
    if (ok) {
      status_ = st;
    }
    taskEXIT_CRITICAL(&lock_);
    failed = failed || !ok;
  } else if (steps == 0) {
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

FeederTask::Counters FeederTask::counters() const
{
  Counters c;
  taskENTER_CRITICAL(&lock_);
  c.state = state_;
  c.moves = moves_;
  c.move_fail = move_fail_;
  c.mode = mode_;
  c.status_ok = status_ok_;
  c.status = status_;
  taskEXIT_CRITICAL(&lock_);
  c.cells = feeder_.cells();
  c.sent_counts = feeder_.sent_counts();
  c.timeouts = servo_.timeouts();
  c.last_error = servo_.last_error();
  return c;
}
