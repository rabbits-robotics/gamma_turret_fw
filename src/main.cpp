// gamma_turret_fw: CAN1 ⇔ CAN2 直結試験
// CAN1（G23 / G21）と CAN2（G19 / G17）を両方立ち上げ、1 秒ごとに互いへフレームを送り、
// 受信したフレームをログに出す。CAN1 と CAN2 のコネクタを CANH-CANH、CANL-CANL で直結して使う。
// デバッグ LED D3（G38）はループごとに反転。D4 / D5 は CAN1_TX / CAN2_TX に吸い込み接続なので送信時に光る。

#include <cstring>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace
{

constexpr const char * TAG = "main";

constexpr gpio_num_t DEBUG_LED3_PIN = GPIO_NUM_38;  // D3 黄緑
constexpr TickType_t LOOP_PERIOD = pdMS_TO_TICKS(1000);

// gamma_turret の CAN ピン（docs/回路図作成リファレンス.md）
constexpr gpio_num_t CAN1_TX_PIN = GPIO_NUM_23;
constexpr gpio_num_t CAN1_RX_PIN = GPIO_NUM_21;
constexpr gpio_num_t CAN2_TX_PIN = GPIO_NUM_19;
constexpr gpio_num_t CAN2_RX_PIN = GPIO_NUM_17;
constexpr uint32_t CAN_BITRATE = 1000000;  // 1 Mbps（beta と同じ。DAMIAO / LK モータの既定）
constexpr uint32_t CAN_TX_QUEUE_DEPTH = 8;

constexpr uint32_t CAN1_TEST_ID = 0x101;
constexpr uint32_t CAN2_TEST_ID = 0x202;

// 受信フレームを ISR からメインタスクへ渡すためのキュー
struct RxItem
{
  int bus;  // 1 = CAN1, 2 = CAN2
  twai_frame_header_t header;
  uint8_t data[8];
};
constexpr UBaseType_t RX_QUEUE_LEN = 16;
QueueHandle_t rx_queue = nullptr;

struct CanBus
{
  int index;
  twai_node_handle_t node;
};
CanBus can1 = {1, nullptr};
CanBus can2 = {2, nullptr};

// --- コールバック（ISR から呼ばれるので、ブロックする API は使わない） ---

bool IRAM_ATTR OnRxDone(twai_node_handle_t node, const twai_rx_done_event_data_t *, void * user_ctx)
{
  auto * bus = static_cast<CanBus *>(user_ctx);
  RxItem item = {};
  item.bus = bus->index;
  twai_frame_t frame = {};
  frame.buffer = item.data;
  frame.buffer_len = sizeof(item.data);
  BaseType_t woken = pdFALSE;
  if (twai_node_receive_from_isr(node, &frame) == ESP_OK) {
    item.header = frame.header;
    xQueueSendFromISR(rx_queue, &item, &woken);
  }
  return woken == pdTRUE;
}

bool IRAM_ATTR OnTxDone(twai_node_handle_t, const twai_tx_done_event_data_t * edata, void * user_ctx)
{
  auto * bus = static_cast<CanBus *>(user_ctx);
  if (!edata->is_tx_success) {
    ESP_EARLY_LOGW(TAG, "CAN%d: tx failed id=0x%03X", bus->index, edata->done_tx_frame->header.id);
  }
  return false;
}

bool IRAM_ATTR OnError(twai_node_handle_t, const twai_error_event_data_t * edata, void * user_ctx)
{
  auto * bus = static_cast<CanBus *>(user_ctx);
  ESP_EARLY_LOGW(TAG, "CAN%d: bus error flags=0x%x", bus->index, edata->err_flags.val);
  return false;
}

bool IRAM_ATTR OnStateChange(
  twai_node_handle_t, const twai_state_change_event_data_t * edata, void * user_ctx)
{
  auto * bus = static_cast<CanBus *>(user_ctx);
  static const char * const NAMES[] = {"error_active", "error_warning", "error_passive", "bus_off"};
  ESP_EARLY_LOGI(TAG, "CAN%d: state %s -> %s", bus->index, NAMES[edata->old_sta], NAMES[edata->new_sta]);
  return false;
}

// --- 初期化 ---

void InitDebugLed(gpio_num_t pin)
{
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&cfg));
  gpio_set_level(pin, 1);  // 吸い込みなので High = 消灯
}

void InitCan(CanBus & bus, gpio_num_t tx, gpio_num_t rx)
{
  twai_onchip_node_config_t cfg = {};
  cfg.io_cfg.tx = tx;
  cfg.io_cfg.rx = rx;
  cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  cfg.bit_timing.bitrate = CAN_BITRATE;
  cfg.fail_retry_cnt = 3;
  cfg.tx_queue_depth = CAN_TX_QUEUE_DEPTH;
  cfg.timestamp_resolution_hz = 1000000;
  ESP_ERROR_CHECK(twai_new_node_onchip(&cfg, &bus.node));

  twai_event_callbacks_t cbs = {};
  cbs.on_rx_done = OnRxDone;
  cbs.on_tx_done = OnTxDone;
  cbs.on_error = OnError;
  cbs.on_state_change = OnStateChange;
  ESP_ERROR_CHECK(twai_node_register_event_callbacks(bus.node, &cbs, &bus));
  ESP_ERROR_CHECK(twai_node_enable(bus.node));
  ESP_LOGI(TAG, "CAN%d: enabled (tx=G%d rx=G%d %lu bps)", bus.index, tx, rx, static_cast<unsigned long>(CAN_BITRATE));
}

// 8 バイトのデータフレームを送る（標準 ID）
void Send(CanBus & bus, uint32_t id, const uint8_t (&data)[8])
{
  uint8_t buf[8];
  std::memcpy(buf, data, sizeof(buf));
  twai_frame_t frame = {};
  frame.header.id = id;
  frame.header.dlc = 8;
  frame.buffer = buf;
  frame.buffer_len = sizeof(buf);
  esp_err_t err = twai_node_transmit(bus.node, &frame, 100);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "CAN%d: transmit error %s", bus.index, esp_err_to_name(err));
    return;
  }
  // 送信バッファはスタック上なので、送り終わるまで待つ
  twai_node_transmit_wait_all_done(bus.node, 100);
}

void LogNodeStatus(const CanBus & bus)
{
  twai_node_status_t st = {};
  twai_node_record_t rec = {};
  if (twai_node_get_info(bus.node, &st, &rec) == ESP_OK) {
    ESP_LOGI(TAG, "CAN%d: state=%d tec=%u rec=%u bus_err=%lu", bus.index, st.state,
             st.tx_error_count, st.rx_error_count, static_cast<unsigned long>(rec.bus_err_num));
  }
}

}  // namespace

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "gamma_turret_fw CAN1 <-> CAN2 loop test");
  InitDebugLed(DEBUG_LED3_PIN);

  rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(RxItem));
  assert(rx_queue != nullptr);

  InitCan(can1, CAN1_TX_PIN, CAN1_RX_PIN);
  InitCan(can2, CAN2_TX_PIN, CAN2_RX_PIN);

  uint32_t count = 0;
  bool led_on = false;
  while (true) {
    ++count;
    led_on = !led_on;
    gpio_set_level(DEBUG_LED3_PIN, led_on ? 0 : 1);

    // CAN1 → CAN2、CAN2 → CAN1 の順に 1 フレームずつ送る
    uint8_t payload[8] = {
      0xA1, static_cast<uint8_t>(count), static_cast<uint8_t>(count >> 8), 0, 0, 0, 0, 0};
    Send(can1, CAN1_TEST_ID, payload);
    payload[0] = 0xB2;
    Send(can2, CAN2_TEST_ID, payload);

    // 少し待ってから、届いたフレームを全部ログに出す
    vTaskDelay(pdMS_TO_TICKS(50));
    RxItem item;
    while (xQueueReceive(rx_queue, &item, 0) == pdTRUE) {
      ESP_LOGI(TAG, "CAN%d rx: id=0x%03lX dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
               item.bus, static_cast<unsigned long>(item.header.id), item.header.dlc,
               item.data[0], item.data[1], item.data[2], item.data[3],
               item.data[4], item.data[5], item.data[6], item.data[7]);
    }

    if (count % 10 == 0) {
      LogNodeStatus(can1);
      LogNodeStatus(can2);
    }
    vTaskDelay(LOOP_PERIOD);
  }
}
