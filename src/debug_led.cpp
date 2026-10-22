#include "debug_led.hpp"

namespace rabcl_esp32
{

DebugLed::DebugLed(const DebugLedConfig & config) : config_(config) {}

esp_err_t DebugLed::Start()
{
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << config_.pin;
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  esp_err_t err = gpio_config(&cfg);
  if (err != ESP_OK) {
    return err;
  }
  Set(false);
  return ESP_OK;
}

void DebugLed::Set(bool on)
{
  on_ = on;
  const bool level_high = config_.active_low ? !on : on;
  gpio_set_level(config_.pin, level_high ? 1 : 0);
}

}  // namespace rabcl_esp32
