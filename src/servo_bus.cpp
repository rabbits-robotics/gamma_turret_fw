#include "servo_bus.hpp"

#include "board.hpp"

namespace
{
rabcl_esp32::HalfDuplexSerialConfig MakeConfig()
{
  rabcl_esp32::HalfDuplexSerialConfig cfg;
  cfg.port = board::SERVO_UART_PORT;
  cfg.tx = board::SERVO_UART_TX_PIN;
  cfg.rx = board::SERVO_UART_RX_PIN;
  cfg.baud = board::SERVO_UART_BAUD;
  cfg.rx_buffer = 256;
  return cfg;
}
}  // namespace

ServoBus::ServoBus()
: port_(MakeConfig())
{
}

esp_err_t ServoBus::Start()
{
  return port_.Start();
}

int ServoBus::Transfer(
  const uint8_t * tx, size_t tx_len, uint8_t * rx, size_t rx_len, uint32_t timeout_ms)
{
  return port_.Transfer(tx, tx_len, rx, rx_len, timeout_ms);
}
