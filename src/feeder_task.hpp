// 装填サーボ（ST3215、半二重 UART1）を回す FreeRTOS タスク。
// Controller の load モード（0 停止 / 1 正転 / 2 逆転、途絶で 0）を rabcl::Feeder に通して「今送るステップ数」を決め、
// rabcl::St3215 のステップ指令（相対カウント）で送る。サーボと話すのはこのタスクだけ（UART の往復がブロックするので制御タスクと分ける）。
// 起動時は 1 Hz で Ping → MODE 0（位置）に切り替えて絶対角を読む → MODE 3（ステップ）に戻す → トルク ON →
// 一番近い穴まで動かして READY（位相合わせ。MODE の往復で EEPROM を 2 回書く）。
// 設計書 docs/superpowers/specs/2026-11-16-feeder-design.md、位相合わせは docs/superpowers/specs/2026-11-21-feeder-phase-align-design.md
#ifndef GAMMA_TURRET_FW__FEEDER_TASK_HPP_
#define GAMMA_TURRET_FW__FEEDER_TASK_HPP_

#include <cstdint>

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rabcl/component/st3215.hpp>
#include <rabcl/controller/feeder.hpp>

#include "control.hpp"
#include "servo_bus.hpp"

class FeederTask
{
public:
  enum class State : uint8_t
  {
    NOT_READY,  // サーボが応答しない / MODE の切り替えや位相合わせに失敗。1 Hz で再接続
    READY,
  };

  struct Counters
  {
    State state;
    int32_t cells;          // Feeder が送ったセル数（逆転は負）
    int32_t sent_counts;    // Feeder が送ったカウントの合計
    uint32_t moves;         // Move が通った回数
    uint32_t move_fail;     // Move が失敗した回数
    uint32_t timeouts;      // St3215 の応答タイムアウト累計
    uint8_t last_error;     // 応答の ERR
    uint8_t mode;           // 接続時に読んだ MODE レジスタ（0xFF = 未読）
    int32_t align_position; // 接続時に位置モードで読んだ絶対角 [count]
    int16_t align_steps;    // 接続時に穴へ合わせるために送った相対ステップ
    bool status_ok;         // 最後の ReadStatus が通った
    rabcl::St3215Status status;
  };

  explicit FeederTask(const Controller & controller);
  ~FeederTask();
  FeederTask(const FeederTask &) = delete;
  FeederTask & operator=(const FeederTask &) = delete;

  esp_err_t Start();
  Counters counters() const;

private:
  static constexpr uint32_t TASK_STACK = 4096;
  static constexpr UBaseType_t TASK_PRIORITY = 5;
  static constexpr uint32_t TICK_MS = 10;
  static constexpr uint32_t CONNECT_PERIOD_MS = 1000;

  static void TaskEntry(void * arg);
  void Task();
  bool Connect();  // Ping → MODE 0 で絶対角 → MODE 3 → トルク ON → 位相合わせ
  void Drive(uint32_t now_ms);

  const Controller & controller_;
  ServoBus bus_;
  rabcl::St3215 servo_;
  rabcl::Feeder feeder_;
  TaskHandle_t task_ = nullptr;
  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;

  State state_ = State::NOT_READY;
  uint32_t last_connect_ms_ = 0;
  uint32_t last_status_ms_ = 0;
  uint32_t consecutive_fail_ = 0;
  uint32_t moves_ = 0;
  uint32_t move_fail_ = 0;
  uint8_t mode_ = 0xFF;
  int32_t align_position_ = 0;
  int16_t align_steps_ = 0;
  bool status_ok_ = false;
  rabcl::St3215Status status_ = {};
};

#endif  // GAMMA_TURRET_FW__FEEDER_TASK_HPP_
