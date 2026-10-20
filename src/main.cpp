// gamma_turret_fw: Lチカ
// デバッグ LED D2（黄）は G37、D3（黄緑）は G38 に吸い込み接続（アノード側が 1kΩ 経由で 3.3V）。
// GPIO を Low にすると点灯、High にすると消灯する。
// G37 は ROM の起動ログが出る UART0 TX と兼用のため、起動直後に D2 が一瞬チラつく。

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{

constexpr gpio_num_t DEBUG_LED2_PIN = GPIO_NUM_37;  // D2 黄
constexpr gpio_num_t DEBUG_LED3_PIN = GPIO_NUM_38;  // D3 黄緑
constexpr TickType_t BLINK_PERIOD = pdMS_TO_TICKS(1000);
constexpr const char * TAG = "main";

void InitDebugLed(gpio_num_t pin)
{
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&cfg));
  gpio_set_level(pin, 1);  // 消灯から始める
}

}  // namespace

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "gamma_turret_fw blink start");
  InitDebugLed(DEBUG_LED2_PIN);
  InitDebugLed(DEBUG_LED3_PIN);

  bool on = false;
  uint32_t count = 0;
  while (true) {
    on = !on;
    // 吸い込みなので Low で点灯。D2 と D3 を交互に点ける
    gpio_set_level(DEBUG_LED3_PIN, on ? 0 : 1);
    gpio_set_level(DEBUG_LED2_PIN, on ? 1 : 0);
    if (on) {
      ESP_LOGI(TAG, "blink %lu", static_cast<unsigned long>(++count));
    }
    vTaskDelay(BLINK_PERIOD);
  }
}
