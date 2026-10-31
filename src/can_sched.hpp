// CAN バス 1 本分の送受信。
//   CanScheduler: esp_timer から slot_us ごとに関節を 1 つずつ回し、Controller::BuildCommand の指令を送る。
//                 DM モータには DM_ENABLE_PERIOD_MS ごとに 1 スロットをイネーブルに充てる
//   CanReceiver:  受信タスク。フィードバック ID で関節を引き、モータ種別ごとに復号して
//                 Controller::SetFeedback と Comm::SetJointState（間引き）に渡す
#ifndef GAMMA_TURRET_FW__CAN_SCHED_HPP_
#define GAMMA_TURRET_FW__CAN_SCHED_HPP_

#include <cstddef>
#include <cstdint>

#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rabcl_esp32/can_bus.hpp>

#include "comm.hpp"
#include "control.hpp"
#include "robot.hpp"

class CanScheduler
{
public:
  struct Counters
  {
    uint32_t sent;
    uint32_t send_fail;
    uint32_t skipped;  // BuildCommand が false（送らないスロット）
    uint32_t enables;
  };

  CanScheduler(
    rabcl_esp32::CanBus & can, Controller & controller,
    const robot::Joint * joints, size_t joint_count, uint32_t slot_us, const char * name);
  ~CanScheduler();
  CanScheduler(const CanScheduler &) = delete;
  CanScheduler & operator=(const CanScheduler &) = delete;

  esp_err_t Start();
  Counters counters() const;

private:
  static void TimerCallback(void * arg);
  void OnSlot();

  rabcl_esp32::CanBus & can_;
  Controller & controller_;
  const robot::Joint * joints_;
  size_t joint_count_;
  uint32_t slot_us_;
  const char * name_;
  esp_timer_handle_t timer_ = nullptr;
  size_t next_ = 0;
  uint32_t round_ = 0;
  uint32_t enable_every_rounds_ = 1;
  volatile uint32_t sent_ = 0;
  volatile uint32_t send_fail_ = 0;
  volatile uint32_t skipped_ = 0;
  volatile uint32_t enables_ = 0;
};

class CanReceiver
{
public:
  CanReceiver(
    rabcl_esp32::CanBus & can, robot::Bus bus, Controller & controller, Comm & comm,
    const char * task_name);
  ~CanReceiver();
  CanReceiver(const CanReceiver &) = delete;
  CanReceiver & operator=(const CanReceiver &) = delete;

  esp_err_t Start();
  uint32_t unknown_frames() const {return unknown_frames_;}

private:
  static constexpr uint32_t TASK_STACK = 4096;
  static constexpr uint32_t JOINT_STATE_DECIMATION = 4;  // Comm へ渡す間引き（250 Hz → 約 60 Hz）

  static void TaskEntry(void * arg);
  void Task();
  void HandleFrame(const rabcl_esp32::CanFrame & frame);

  rabcl_esp32::CanBus & can_;
  robot::Bus bus_;
  Controller & controller_;
  Comm & comm_;
  const char * task_name_;
  TaskHandle_t task_ = nullptr;
  uint32_t rx_count_[robot::JOINT_COUNT] = {};
  uint8_t last_err_[robot::JOINT_COUNT] = {};
  volatile uint32_t unknown_frames_ = 0;
};

#endif  // GAMMA_TURRET_FW__CAN_SCHED_HPP_
