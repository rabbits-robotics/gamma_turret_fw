// gamma_turret_fw: Lチカ
// デバッグ LED D3（黄緑）は G38 に吸い込み接続（アノード側が 1kΩ 経由で 3.3V）。
// GPIO を Low にすると点灯、High にすると消灯する。

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{

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
  InitDebugLed(DEBUG_LED3_PIN);

  bool on = false;
  uint32_t count = 0;
  while (true) {
    on = !on;
    gpio_set_level(DEBUG_LED3_PIN, on ? 0 : 1);  // 吸い込みなので Low で点灯
    if (on) {
      ESP_LOGI(TAG, "blink %lu", static_cast<unsigned long>(++count));
    }
    vTaskDelay(BLINK_PERIOD);
  }
}
