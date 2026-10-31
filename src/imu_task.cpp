#include "imu_task.hpp"

#include <esp_log.h>
#include <esp_timer.h>
#include <rabcl/protocol/protocol.hpp>
#include <rabcl/utils/utils.hpp>

#include "board.hpp"
#include "robot.hpp"

namespace
{

constexpr const char * TAG = "imu";

rabcl_esp32::SerialPortConfig MakeSerialConfig()
{
  rabcl_esp32::SerialPortConfig cfg;
  cfg.port = board::IMU_UART_PORT;
  cfg.tx = board::IMU_UART_TX_PIN;
  cfg.rx = board::IMU_UART_RX_PIN;
  cfg.baud = board::IMU_UART_BAUD;
  cfg.rx_buffer = 256;
  cfg.tx_buffer = 0;  // 要求は 4〜5 B。書き終わるまでブロックしてよい
  return cfg;
}

rabcl::BNO055Config MakeDriverConfig()
{
  rabcl::BNO055Config cfg;
  cfg.axis_map_config = robot::IMU_AXIS_MAP_CONFIG;
  cfg.axis_map_sign = robot::IMU_AXIS_MAP_SIGN;
  cfg.operation_mode = robot::IMU_OPERATION_MODE;
  cfg.use_external_crystal = robot::IMU_USE_EXTERNAL_CRYSTAL;
  cfg.sample_period_ms = 0;  // 読み周期はこのタスクの tick が持つ（二重管理しない）。IMU_SAMPLE_PERIOD_MS は下の vTaskDelayUntil
  cfg.response_timeout_ms = robot::IMU_RESPONSE_TIMEOUT_MS;
  cfg.boot_wait_ms = robot::IMU_BOOT_WAIT_MS;
  cfg.clock_settle_ms = robot::IMU_CLOCK_SETTLE_MS;
  cfg.max_consecutive_errors = robot::IMU_MAX_CONSECUTIVE_ERRORS;
  return cfg;
}

uint32_t NowMs()
{
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

}  // namespace


// --- ImuTask ---

ImuTask::ImuTask(ImuStore & store, Comm & comm, const Controller & controller)
: store_(store), comm_(comm), controller_(controller), port_(MakeSerialConfig()),
  driver_(MakeDriverConfig())
{
}

ImuTask::~ImuTask()
{
  if (task_ != nullptr) {
    vTaskDelete(task_);
  }
}

esp_err_t ImuTask::Start()
{
  if (task_ != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  const esp_err_t err = port_.Start();
  if (err != ESP_OK) {
    return err;
  }
  if (xTaskCreate(TaskEntry, "imu", TASK_STACK, this, TASK_PRIORITY, &task_) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

ImuTask::Counters ImuTask::counters() const
{
  const rabcl::BNO055Sequencer & seq = driver_.sequencer();
  return {samples_, seq.error_count(), seq.reinit_count(), seq.calib_stat(), seq.state()};
}

void ImuTask::TaskEntry(void * arg)
{
  static_cast<ImuTask *>(arg)->Task();
}

void ImuTask::Task()
{
  TickType_t last_wake = xTaskGetTickCount();
  bool was_running = false;
  uint8_t tx[16];
  uint8_t rx[rabcl::BNO055Uart::MAX_READ + 2];
  while (true) {
    const size_t n = driver_.Tick(NowMs(), tx, sizeof(tx));
    if (n > 0) {
      // 要求の直前に受信バッファを空にする。遅れて届いた前の応答や部分応答が残っていると、以後ずっと
      // 「1 周期前の応答」を読み続けてしまう（エラーにならないので気づけない）。1 要求 1 応答を UART 層でも守る
      uint8_t junk[64];
      while (port_.Read(junk, sizeof(junk), 0) > 0) {
      }
      port_.Write(tx, n);
    }
    const size_t expected = driver_.expected_response_len();
    bool failed = false;
    if (expected > 0) {
      // 先にヘッダ 2 バイトだけ読む。EE（ACK / エラー）は 2 バイトで完結、BB（データ）は 2 バイト目が長さ。
      // 期待長をまとめて待つと、エラー応答（EE 07 = bus over run）のたびに読みがタイムアウト（30 ms = BNO055 の
      // 融合周期 10 ms の 3 倍）まで塞がり、再送が毎回同じ位相でぶつかって連続エラーになる（2026-11-01 の実機）
      const TickType_t timeout = pdMS_TO_TICKS(robot::IMU_RESPONSE_TIMEOUT_MS) + 1;  // tick の丸めで短くならないよう 1 tick 足す
      int got = port_.Read(rx, 2, timeout);
      if (got == 2 && rx[0] == rabcl::BNO055Uart::RESPONSE && rx[1] > 0 && rx[1] <= rabcl::BNO055Uart::MAX_READ) {
        const int more = port_.Read(rx + 2, rx[1], timeout);
        if (more > 0) {
          got += more;
        }
      }
      rabcl::BNO055Sample sample = {};
      const uint32_t err_before = driver_.sequencer().error_count();
      if (got > 0 && driver_.Feed(rx, static_cast<size_t>(got), NowMs(), sample)) {
        samples_ = samples_ + 1;
        Publish(sample, driver_.last_request_ms());
      }
      failed = driver_.sequencer().error_count() > err_before;
    }

    const bool running = driver_.sequencer().running();
    if (was_running && !running) {
      ESP_LOGW(TAG, "BNO055 lost (reinit %lu)", static_cast<unsigned long>(driver_.sequencer().reinit_count()));
      ImuState invalid;
      invalid.valid = false;
      store_.Set(invalid);
      comm_.ClearImu();
    } else if (!was_running && running) {
      ESP_LOGI(TAG, "BNO055 running");
    }
    was_running = running;

    if (running && failed) {
      // bus over run の直後は 10 ms 境界を待たずに即再送して、BNO055 の更新タイミングと位相をずらす
      taskYIELD();
    } else if (running) {
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(robot::IMU_SAMPLE_PERIOD_MS));
    } else {
      vTaskDelay(1);
      last_wake = xTaskGetTickCount();
    }
  }
}

void ImuTask::Publish(const rabcl::BNO055Sample & sample, uint32_t request_ms)
{
  // BNO055: euler = heading（時計回り正）, roll, pitch → REP-103（yaw 左回り正）。pitch / roll の符号は実機で確定
  ImuState state;
  state.attitude.yaw = rabcl::Utils::WrapPi(-sample.euler[0]);
  state.attitude.pitch = robot::IMU_PITCH_SIGN * sample.euler[2];
  state.attitude.roll = robot::IMU_ROLL_SIGN * sample.euler[1];
  for (int i = 0; i < 3; ++i) {
    state.gyro[i] = robot::IMU_GYRO_SIGN[i] * sample.gyro[robot::IMU_GYRO_INDEX[i]];
  }
  state.valid = true;
  state.time_us = static_cast<int64_t>(request_ms) * 1000;
  state.neck_yaw = controller_.yaw_angle();
  store_.Set(state);

  rabcl::protocol::Imu imu = {};
  for (int i = 0; i < 3; ++i) {
    imu.gyro[i] = state.gyro[i];
  }
  imu.euler[0] = state.attitude.yaw;
  imu.euler[1] = state.attitude.roll;
  imu.euler[2] = state.attitude.pitch;
  comm_.SetImu(imu);
}
