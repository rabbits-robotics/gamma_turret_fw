// gamma_turret_fw: CAN1 ⇔ CAN2 直結試験（CanBus / DebugLed クラス版）
// CAN1 と CAN2 のコネクタを CANH-CANH、CANL-CANL で直結して使う。
// 起動時に API の異常系（Start の二重呼び出し、dlc > 8）を 1 回だけ確認し、
// 5 秒後に 20 フレーム連続送信で受信キューの溢れ（rx_dropped）を確認する。

#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "board.hpp"
#include "can_bus.hpp"
#include "debug_led.hpp"

namespace
{

constexpr const char * TAG = "main";
constexpr TickType_t LOOP_PERIOD = pdMS_TO_TICKS(1000);
constexpr uint32_t CAN1_TEST_ID = 0x101;
constexpr uint32_t CAN2_TEST_ID = 0x202;
constexpr uint32_t BURST_COUNT = 20;  // rx_queue_depth（16）を超える数

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

CanFrame MakeTestFrame(uint32_t id, uint8_t marker, uint32_t count)
{
  CanFrame f;
  f.id = id;
  f.dlc = 8;
  f.data[0] = marker;
  f.data[1] = static_cast<uint8_t>(count);
  f.data[2] = static_cast<uint8_t>(count >> 8);
  return f;
}

void LogFrame(int bus, const CanFrame & f)
{
  ESP_LOGI(TAG, "CAN%d rx: id=0x%03lX dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
           bus, static_cast<unsigned long>(f.id), f.dlc,
           f.data[0], f.data[1], f.data[2], f.data[3], f.data[4], f.data[5], f.data[6], f.data[7]);
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

void DrainAndLog(int bus, CanBus & can)
{
  CanFrame f;
  while (can.Receive(f, 0)) {
    LogFrame(bus, f);
  }
}

}  // namespace

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "gamma_turret_fw CAN1 <-> CAN2 loop test (class version)");
  // 再起動の原因調査用（1 = power on, 3 = sw, 4 = panic, 5/6/7 = WDT, 9 = brownout）
  ESP_LOGI(TAG, "reset reason: %d", static_cast<int>(esp_reset_reason()));

  DebugLed led3(DebugLedConfig{board::DEBUG_LED3_PIN, true});
  ESP_ERROR_CHECK(led3.Start());

  CanBus can1(MakeCanConfig(board::CAN1_TX_PIN, board::CAN1_RX_PIN));
  CanBus can2(MakeCanConfig(board::CAN2_TX_PIN, board::CAN2_RX_PIN));
  ESP_ERROR_CHECK(can1.Start());
  ESP_ERROR_CHECK(can2.Start());

  // 異常系の確認（期待: Start 2 回目 = ESP_ERR_INVALID_STATE、dlc 9 = ESP_ERR_INVALID_ARG）
  ESP_LOGI(TAG, "check: Start() twice -> %s", esp_err_to_name(can1.Start()));
  CanFrame bad = MakeTestFrame(CAN1_TEST_ID, 0xEE, 0);
  bad.dlc = 9;
  ESP_LOGI(TAG, "check: Send(dlc=9) -> %s", esp_err_to_name(can1.Send(bad, pdMS_TO_TICKS(100))));

  uint32_t count = 0;
  while (true) {
    ++count;
    led3.Toggle();

    if (count == 5) {
      // 受信キューの溢れ確認: CAN2 が drain する前に 20 フレームをまとめて送る
      ESP_LOGI(TAG, "check: burst %lu frames CAN1 -> CAN2", static_cast<unsigned long>(BURST_COUNT));
      for (uint32_t i = 0; i < BURST_COUNT; ++i) {
        can1.Send(MakeTestFrame(CAN1_TEST_ID, 0xB0, i), pdMS_TO_TICKS(100));
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      uint32_t received = 0;
      CanFrame f;
      while (can2.Receive(f, 0)) {
        ++received;
      }
      ESP_LOGI(TAG, "check: burst received=%lu rx_dropped=%lu (expect received=16 dropped=4)",
               static_cast<unsigned long>(received), static_cast<unsigned long>(can2.GetCounters().rx_dropped));
      DrainAndLog(1, can1);
    }

    can1.Send(MakeTestFrame(CAN1_TEST_ID, 0xA1, count), pdMS_TO_TICKS(100));
    can2.Send(MakeTestFrame(CAN2_TEST_ID, 0xB2, count), pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    DrainAndLog(2, can2);
    DrainAndLog(1, can1);

    if (count % 10 == 0) {
      LogCounters(1, can1);
      LogCounters(2, can2);
    }
    vTaskDelay(LOOP_PERIOD);
  }
}
