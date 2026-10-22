// gamma_turret_fw: 骨組み
// 周辺機器（デバッグ LED、CAN1 / CAN2）を立ち上げ、LED を 1 秒周期で点滅させる。
// 制御・通信のロジックはタスクとして src/tasks/ に足していく。

#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rabcl/interface/can.hpp>
#include <rabcl_esp32/can_bus.hpp>
#include <rabcl_esp32/debug_led.hpp>

#include "board.hpp"

namespace
{

constexpr const char * TAG = "main";
constexpr TickType_t LOOP_PERIOD = pdMS_TO_TICKS(1000);

using rabcl_esp32::CanBus;
using rabcl_esp32::CanBusConfig;
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

  // rabcl がリンクされていることの確認（DAMIAO のイネーブルコマンド。送信はしない）
  uint8_t enable_cmd[8] = {};
  rabcl::Can::PrepareDMMotorEnable(enable_cmd);
  ESP_LOGI(TAG, "rabcl DM enable cmd: %02X %02X %02X %02X %02X %02X %02X %02X",
           enable_cmd[0], enable_cmd[1], enable_cmd[2], enable_cmd[3],
           enable_cmd[4], enable_cmd[5], enable_cmd[6], enable_cmd[7]);

  uint32_t count = 0;
  while (true) {
    ++count;
    led3.Toggle();
    if (count % 10 == 0) {
      LogCounters(1, can1);
      LogCounters(2, can2);
    }
    vTaskDelay(LOOP_PERIOD);
  }
}
