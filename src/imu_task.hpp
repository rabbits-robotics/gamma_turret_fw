// BNO055（UART）を読んで ImuStore に書く FreeRTOS タスク。
// 「要求を出す → 応答の長さだけブロック読み（約 2 ms で返る）→ RUNNING なら 10 ms 境界で次」。
// FreeRTOS の tick は 100 Hz のままでよい。起動時に待たない（valid = false で始め、初期化できた時点から使う）。
// 軸の入れ替えは BNO055 のレジスタ（robot.hpp の IMU_AXIS_MAP_*）、符号の約束（heading は時計回り正）だけここで直す
#ifndef GAMMA_TURRET_FW__IMU_TASK_HPP_
#define GAMMA_TURRET_FW__IMU_TASK_HPP_

#include <cstdint>

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rabcl/component/bno055.hpp>
#include <rabcl_esp32/serial_port.hpp>

#include "comm.hpp"
#include "control.hpp"
#include "imu.hpp"

class ImuTask
{
public:
  struct Counters
  {
    uint32_t samples;
    uint32_t errors;        // 連続エラー数
    uint32_t reinits;       // ソフトリセットした回数
    uint8_t calib_stat;
    rabcl::BNO055Sequencer::State state;
  };

  ImuTask(ImuStore & store, Comm & comm, const Controller & controller);
  ~ImuTask();
  ImuTask(const ImuTask &) = delete;
  ImuTask & operator=(const ImuTask &) = delete;

  esp_err_t Start();
  Counters counters() const;

private:
  static constexpr uint32_t TASK_STACK = 4096;
  static constexpr UBaseType_t TASK_PRIORITY = 5;

  static void TaskEntry(void * arg);
  void Task();
  void Publish(const rabcl::BNO055Sample & sample, uint32_t request_ms);

  ImuStore & store_;
  Comm & comm_;
  const Controller & controller_;
  rabcl_esp32::SerialPort port_;
  rabcl::BNO055UartDriver driver_;
  TaskHandle_t task_ = nullptr;
  volatile uint32_t samples_ = 0;
};

#endif  // GAMMA_TURRET_FW__IMU_TASK_HPP_
