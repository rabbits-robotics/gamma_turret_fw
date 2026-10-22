#include "can_bus.hpp"

#include <cstring>

#include <esp_attr.h>
#include <esp_twai_onchip.h>

namespace rabcl_esp32
{

CanBus::CanBus(const CanBusConfig & config) : config_(config) {}

CanBus::~CanBus()
{
  if (started_) {
    Stop();
  }
  if (rx_queue_ != nullptr) {
    vQueueDelete(rx_queue_);
    rx_queue_ = nullptr;
  }
}

esp_err_t CanBus::Start()
{
  if (started_) {
    return ESP_ERR_INVALID_STATE;
  }
  if (rx_queue_ == nullptr) {
    rx_queue_ = xQueueCreate(config_.rx_queue_depth, sizeof(CanFrame));
    if (rx_queue_ == nullptr) {
      return ESP_ERR_NO_MEM;
    }
  }

  twai_onchip_node_config_t cfg = {};
  cfg.io_cfg.tx = config_.tx;
  cfg.io_cfg.rx = config_.rx;
  cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  cfg.bit_timing.bitrate = config_.bitrate;
  cfg.fail_retry_cnt = config_.fail_retry_cnt;
  cfg.tx_queue_depth = config_.tx_queue_depth;
  cfg.timestamp_resolution_hz = 1000000;
  esp_err_t err = twai_new_node_onchip(&cfg, &node_);
  if (err != ESP_OK) {
    return err;
  }

  twai_event_callbacks_t cbs = {};
  cbs.on_rx_done = OnRxDone;
  cbs.on_tx_done = OnTxDone;
  cbs.on_error = OnError;
  cbs.on_state_change = OnStateChange;
  err = twai_node_register_event_callbacks(node_, &cbs, this);
  if (err != ESP_OK) {
    twai_node_delete(node_);
    node_ = nullptr;
    return err;
  }
  err = twai_node_enable(node_);
  if (err != ESP_OK) {
    twai_node_delete(node_);
    node_ = nullptr;
    return err;
  }
  started_ = true;
  return ESP_OK;
}

esp_err_t CanBus::Stop()
{
  if (!started_) {
    return ESP_ERR_INVALID_STATE;
  }
  twai_node_disable(node_);
  esp_err_t err = twai_node_delete(node_);
  node_ = nullptr;
  started_ = false;
  return err;
}

esp_err_t CanBus::Send(const CanFrame & frame, TickType_t timeout)
{
  if (frame.dlc > sizeof(frame.data)) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!started_) {
    return ESP_ERR_INVALID_STATE;
  }
  MaybeRecover();

  uint8_t buf[8];
  std::memcpy(buf, frame.data, sizeof(buf));
  twai_frame_t tx = {};
  tx.header.id = frame.id;
  tx.header.dlc = frame.dlc;
  tx.header.ide = frame.ext ? 1 : 0;
  tx.header.rtr = frame.rtr ? 1 : 0;
  tx.buffer = buf;
  tx.buffer_len = frame.dlc;

  const int timeout_ms = static_cast<int>(pdTICKS_TO_MS(timeout));
  esp_err_t err = twai_node_transmit(node_, &tx, timeout_ms);
  if (err != ESP_OK) {
    return err;
  }
  // buf はスタック上なので、送信が終わるまでここで待つ
  return twai_node_transmit_wait_all_done(node_, timeout_ms);
}

bool CanBus::Receive(CanFrame & frame, TickType_t timeout)
{
  if (!started_) {
    return false;
  }
  MaybeRecover();
  return xQueueReceive(rx_queue_, &frame, timeout) == pdTRUE;
}

CanBusCounters CanBus::GetCounters() const
{
  CanBusCounters c;
  c.tx_fail = tx_fail_;
  c.bus_error = bus_error_;
  c.rx_dropped = rx_dropped_;
  c.bus_off = bus_off_;
  c.recovered = recovered_;
  return c;
}

esp_err_t CanBus::GetStatus(twai_node_status_t & status) const
{
  if (!started_) {
    return ESP_ERR_INVALID_STATE;
  }
  return twai_node_get_info(node_, &status, nullptr);
}

esp_err_t CanBus::Recover()
{
  if (!started_) {
    return ESP_ERR_INVALID_STATE;
  }
  recover_requested_ = false;
  return twai_node_recover(node_);
}

void CanBus::MaybeRecover()
{
  if (recover_requested_ && config_.auto_recover) {
    Recover();
  }
}

// --- ISR コールバック。ブロックする API とログは使わない ---

bool IRAM_ATTR CanBus::OnRxDone(twai_node_handle_t node, const twai_rx_done_event_data_t *, void * ctx)
{
  auto * self = static_cast<CanBus *>(ctx);
  CanFrame frame;
  twai_frame_t rx = {};
  rx.buffer = frame.data;
  rx.buffer_len = sizeof(frame.data);
  if (twai_node_receive_from_isr(node, &rx) != ESP_OK) {
    return false;
  }
  frame.id = rx.header.id;
  frame.dlc = static_cast<uint8_t>(rx.header.dlc);
  frame.ext = rx.header.ide != 0;
  frame.rtr = rx.header.rtr != 0;
  frame.timestamp_us = rx.header.timestamp;
  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(self->rx_queue_, &frame, &woken) != pdTRUE) {
    self->rx_dropped_ = self->rx_dropped_ + 1;
  }
  return woken == pdTRUE;
}

bool IRAM_ATTR CanBus::OnTxDone(twai_node_handle_t, const twai_tx_done_event_data_t * edata, void * ctx)
{
  auto * self = static_cast<CanBus *>(ctx);
  if (!edata->is_tx_success) {
    self->tx_fail_ = self->tx_fail_ + 1;
  }
  return false;
}

bool IRAM_ATTR CanBus::OnError(twai_node_handle_t, const twai_error_event_data_t *, void * ctx)
{
  auto * self = static_cast<CanBus *>(ctx);
  self->bus_error_ = self->bus_error_ + 1;
  return false;
}

bool IRAM_ATTR CanBus::OnStateChange(
  twai_node_handle_t, const twai_state_change_event_data_t * edata, void * ctx)
{
  auto * self = static_cast<CanBus *>(ctx);
  if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
    self->bus_off_ = self->bus_off_ + 1;
    if (self->config_.auto_recover) {
      self->recover_requested_ = true;
    }
  } else if (edata->old_sta == TWAI_ERROR_BUS_OFF && edata->new_sta == TWAI_ERROR_ACTIVE) {
    self->recovered_ = self->recovered_ + 1;
  }
  return false;
}

}  // namespace rabcl_esp32
