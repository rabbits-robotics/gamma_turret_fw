// gamma_turret 基板のピン割り当てと定数（docs/回路図作成リファレンス.md を正とする）
#ifndef GAMMA_TURRET_FW__BOARD_HPP_
#define GAMMA_TURRET_FW__BOARD_HPP_

#include <cstdint>

#include "driver/gpio.h"

namespace board
{

// デバッグ LED（吸い込み接続。Low で点灯）
constexpr gpio_num_t DEBUG_LED2_PIN = GPIO_NUM_37;  // D2 黄（UART0 TX と兼用。起動時に ROM ログで一瞬点く）
constexpr gpio_num_t DEBUG_LED3_PIN = GPIO_NUM_38;  // D3 黄緑

// CAN（MCP2562 × 2）
constexpr gpio_num_t CAN1_TX_PIN = GPIO_NUM_23;  // CAN1: 砲塔・ジンバル系
constexpr gpio_num_t CAN1_RX_PIN = GPIO_NUM_21;
constexpr gpio_num_t CAN2_TX_PIN = GPIO_NUM_19;  // CAN2: 車体系
constexpr gpio_num_t CAN2_RX_PIN = GPIO_NUM_17;
constexpr uint32_t CAN_BITRATE = 1000000;  // 1 Mbps（beta と同じ。DAMIAO / LK モータの既定）

}  // namespace board

#endif  // GAMMA_TURRET_FW__BOARD_HPP_
