#ifndef RABCL_ESP32__CAN_BUS_HPP_
#define RABCL_ESP32__CAN_BUS_HPP_

#include <cstdint>

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace rabcl_esp32
{

struct CanFrame
{
  uint32_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8] = {};
  bool ext = false;           // 29 bit ID
  bool rtr = false;           // リモートフレーム
  uint64_t timestamp_us = 0;  // 受信時刻（Start 後の経過時間）
};

struct CanBusConfig
{
  gpio_num_t tx;
  gpio_num_t rx;
  uint32_t bitrate;
  uint32_t tx_queue_depth = 8;
  uint32_t rx_queue_depth = 16;
  int8_t fail_retry_cnt = 3;  // 送信失敗時のハード再送回数（-1 で無限）
  bool auto_recover = true;   // バスオフになったら次の Send / Receive で自動回復する
};

struct CanBusCounters
{
  uint32_t tx_fail = 0;     // ACK が無いなどで送信に失敗した回数
  uint32_t bus_error = 0;   // バスエラーの回数
  uint32_t rx_dropped = 0;  // 受信キューが一杯で捨てたフレーム数
  uint32_t bus_off = 0;     // バスオフに入った回数
  uint32_t recovered = 0;   // バスオフから error_active に戻った回数
};

// TWAI コントローラ 1 つ分。受信は ISR からキューに積み、タスク側で Receive() する。
class CanBus
{
public:
  explicit CanBus(const CanBusConfig & config);
  ~CanBus();
  CanBus(const CanBus &) = delete;
  CanBus & operator=(const CanBus &) = delete;

  esp_err_t Start();  // 2 回目は ESP_ERR_INVALID_STATE
  esp_err_t Stop();
  // dlc > 8 は ESP_ERR_INVALID_ARG。送信完了（または失敗）まで待つ
  esp_err_t Send(const CanFrame & frame, TickType_t timeout);
  // キューから 1 フレーム取り出す。timeout 内に無ければ false
  bool Receive(CanFrame & frame, TickType_t timeout);
  CanBusCounters GetCounters() const;
  esp_err_t GetStatus(twai_node_status_t & status) const;
  esp_err_t Recover();  // バスオフからの手動回復

private:
  static bool OnRxDone(twai_node_handle_t node, const twai_rx_done_event_data_t * edata, void * ctx);
  static bool OnTxDone(twai_node_handle_t node, const twai_tx_done_event_data_t * edata, void * ctx);
  static bool OnError(twai_node_handle_t node, const twai_error_event_data_t * edata, void * ctx);
  static bool OnStateChange(
    twai_node_handle_t node, const twai_state_change_event_data_t * edata, void * ctx);
  void MaybeRecover();

  CanBusConfig config_;
  twai_node_handle_t node_ = nullptr;
  QueueHandle_t rx_queue_ = nullptr;
  bool started_ = false;
  volatile bool recover_requested_ = false;
  // ISR から更新するカウンタ
  volatile uint32_t tx_fail_ = 0;
  volatile uint32_t bus_error_ = 0;
  volatile uint32_t rx_dropped_ = 0;
  volatile uint32_t bus_off_ = 0;
  volatile uint32_t recovered_ = 0;
};

}  // namespace rabcl_esp32

#endif  // RABCL_ESP32__CAN_BUS_HPP_
