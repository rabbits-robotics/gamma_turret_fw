// ST3215 バス（SRV1、半二重 UART1）。rabcl::St3215Port を rabcl_esp32::HalfDuplexSerial で実装する薄いアダプタ
#ifndef GAMMA_TURRET_FW__SERVO_BUS_HPP_
#define GAMMA_TURRET_FW__SERVO_BUS_HPP_

#include <esp_err.h>
#include <rabcl/component/st3215.hpp>
#include <rabcl_esp32/half_duplex_serial.hpp>

class ServoBus : public rabcl::St3215Port
{
public:
  ServoBus();
  esp_err_t Start();
  int Transfer(
    const uint8_t * tx, size_t tx_len, uint8_t * rx, size_t rx_len, uint32_t timeout_ms) override;

private:
  rabcl_esp32::HalfDuplexSerial port_;
};

#endif  // GAMMA_TURRET_FW__SERVO_BUS_HPP_
