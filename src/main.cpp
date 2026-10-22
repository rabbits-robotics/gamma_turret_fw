// gamma_turret_fw: DM4310 の CAN 疎通試験
// CAN2 につないだ DM4310（前右脚、CAN_ID 0x47 / MST_ID 0x57）に 1 秒ごとにイネーブルを送り、
// 受信したフレームをすべてログする。MST_ID のフレームは rabcl で復号して位置・速度・トルク・温度も出す。
// SEND_MIT_ZERO_CMD を true にすると、イネーブル確認後にゲイン 0 の MIT 指令を 10 Hz で送り、
// フィードバックを連続で受ける（トルクは出ない）。モータは機体に組み込み済みなので段階的に進める。
// 制御・通信のロジックはタスクとして src/tasks/ に足していく。

#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rabcl/interface/can.hpp>
#include <rabcl/utils/type.hpp>
#include <rabcl_esp32/can_bus.hpp>
#include <rabcl_esp32/debug_led.hpp>

#include "board.hpp"

namespace
{

constexpr const char * TAG = "main";
constexpr TickType_t LOOP_PERIOD = pdMS_TO_TICKS(100);  // 10 Hz
constexpr uint32_t ENABLE_PERIOD_LOOPS = 10;            // 1 s ごとにイネーブル
constexpr uint32_t COUNTER_PERIOD_LOOPS = 100;          // 10 s ごとにカウンタ
constexpr TickType_t SEND_TIMEOUT = pdMS_TO_TICKS(10);
constexpr bool SEND_MIT_ZERO_CMD = false;  // イネーブル後にゲイン 0 の MIT 指令を送る（段階 2 で true に）
constexpr uint8_t DM_ERR_ENABLED = 1;     // フィードバック D[0] 上位 4 bit

// 試験対象: 前右脚
constexpr uint32_t TEST_CAN_ID = board::LEG_CAN_ID[0];
constexpr uint32_t TEST_MST_ID = board::LEG_MST_ID[0];

using rabcl_esp32::CanBus;
using rabcl_esp32::CanBusConfig;
using rabcl_esp32::CanFrame;
using rabcl_esp32::DebugLed;
using rabcl_esp32::DebugLedConfig;

CanBusConfig MakeCanConfig(gpio_num_t tx, gpio_num_t rx)
{
  CanBusConfig cfg;
  cfg.tx = tx;
  cfg.rx = rx;
  cfg.bitrate = board::CAN_BITRATE;
  return cfg;
}

void LogCounters(int bus, const CanBus & can)
{
  const auto c = can.GetCounters();
  twai_node_status_t st = {};
  can.GetStatus(st);
  ESP_LOGI(TAG, "CAN%d: state=%d tec=%u rec=%u tx_fail=%lu bus_error=%lu rx_dropped=%lu bus_off=%lu recovered=%lu",
           bus, st.state, st.tx_error_count, st.rx_error_count,
           static_cast<unsigned long>(c.tx_fail), static_cast<unsigned long>(c.bus_error),
           static_cast<unsigned long>(c.rx_dropped), static_cast<unsigned long>(c.bus_off),
           static_cast<unsigned long>(c.recovered));
}

void LogFrame(int bus, const CanFrame & f)
{
  ESP_LOGI(TAG, "CAN%d rx: id=0x%03lX dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
           bus, static_cast<unsigned long>(f.id), f.dlc,
           f.data[0], f.data[1], f.data[2], f.data[3], f.data[4], f.data[5], f.data[6], f.data[7]);
}

// DM4310 のフィードバックを復号してログ。イネーブル済みなら true
bool LogDMFeedback(const CanFrame & f)
{
  rabcl::MotorInfo m = {};
  rabcl::Can::ParseDMMotorFeedback(
    f.data, m, rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, 1.0f);
  const uint8_t err = rabcl::Can::DMFeedbackError(f.data);
  ESP_LOGI(TAG, "DM4310 fb: id=%u err=%u pos=%.3f rad vel=%.3f rad/s tor=%.3f Nm t_mos=%.0f t_rotor=%.0f",
           rabcl::Can::DMFeedbackId(f.data), err,
           static_cast<double>(m.position_), static_cast<double>(m.velocity_),
           static_cast<double>(m.torque_), static_cast<double>(m.temperature_mos_),
           static_cast<double>(m.temperature_));
  return err == DM_ERR_ENABLED;
}

void SendOrLog(CanBus & can, uint32_t id, const uint8_t data[8], const char * what)
{
  CanFrame f;
  f.id = id;
  f.dlc = 8;
  for (int i = 0; i < 8; ++i) {
    f.data[i] = data[i];
  }
  const esp_err_t err = can.Send(f, SEND_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "send %s (id=0x%03lX) failed: %s", what, static_cast<unsigned long>(id), esp_err_to_name(err));
  }
}

}  // namespace

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "gamma_turret_fw start (reset reason: %d)", static_cast<int>(esp_reset_reason()));

  DebugLed led3(DebugLedConfig{board::DEBUG_LED3_PIN, true});
  ESP_ERROR_CHECK(led3.Start());

  CanBus can1(MakeCanConfig(board::CAN1_TX_PIN, board::CAN1_RX_PIN));
  CanBus can2(MakeCanConfig(board::CAN2_TX_PIN, board::CAN2_RX_PIN));
  ESP_ERROR_CHECK(can1.Start());
  ESP_ERROR_CHECK(can2.Start());
  ESP_LOGI(TAG, "DM4310 test on CAN2: CAN_ID=0x%03lX MST_ID=0x%03lX",
           static_cast<unsigned long>(TEST_CAN_ID), static_cast<unsigned long>(TEST_MST_ID));

  uint8_t enable_cmd[8] = {};
  rabcl::Can::PrepareDMMotorEnable(enable_cmd);
  uint8_t mit_zero_cmd[8] = {};
  rabcl::Can::PrepareDMMotorMITCmd(
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, mit_zero_cmd);

  bool enabled = false;
  uint32_t count = 0;
  while (true) {
    ++count;

    // 受信は全部ログする（CAN1 は何もつないでいなくても念のため）
    CanFrame f;
    while (can2.Receive(f, 0)) {
      LogFrame(2, f);
      if (f.id == TEST_MST_ID) {
        enabled = LogDMFeedback(f);
      }
    }
    while (can1.Receive(f, 0)) {
      LogFrame(1, f);
    }

    if (count % ENABLE_PERIOD_LOOPS == 0) {
      led3.Toggle();
      SendOrLog(can2, TEST_CAN_ID, enable_cmd, "enable");
    } else if (SEND_MIT_ZERO_CMD && enabled) {
      SendOrLog(can2, TEST_CAN_ID, mit_zero_cmd, "MIT zero");
    }

    if (count % COUNTER_PERIOD_LOOPS == 0) {
      LogCounters(1, can1);
      LogCounters(2, can2);
    }
    vTaskDelay(LOOP_PERIOD);
  }
}
