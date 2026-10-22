#ifndef RABCL_ESP32__DEBUG_LED_HPP_
#define RABCL_ESP32__DEBUG_LED_HPP_

#include <driver/gpio.h>
#include <esp_err.h>

namespace rabcl_esp32
{

struct DebugLedConfig
{
  gpio_num_t pin;
  bool active_low = true;  // 吸い込み接続（Low で点灯）なら true
};

// GPIO 1 本の LED。コンストラクタはハードを触らず、Start() で出力に設定して消灯する。
class DebugLed
{
public:
  explicit DebugLed(const DebugLedConfig & config);
  esp_err_t Start();
  void Set(bool on);
  void On() { Set(true); }
  void Off() { Set(false); }
  void Toggle() { Set(!on_); }

private:
  DebugLedConfig config_;
  bool on_ = false;
};

}  // namespace rabcl_esp32

#endif  // RABCL_ESP32__DEBUG_LED_HPP_
