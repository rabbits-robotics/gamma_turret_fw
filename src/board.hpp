// gamma_turret 基板のピン割り当てと定数（docs/回路図作成リファレンス.md を正とする）
#ifndef GAMMA_TURRET_FW__BOARD_HPP_
#define GAMMA_TURRET_FW__BOARD_HPP_

#include <cstdint>

#include <driver/gpio.h>

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

// CAN ID（README の「CAN ID 一覧」が正）
// DAMIAO: CAN_ID は 16 未満（フィードバック先頭バイトの状態コードと重ならないため）。
//         CAN_ID = 指令 ID（MIT モード）、0x200 + CAN_ID = 指令 ID（速度モード）、MST_ID = CAN_ID + 0x10 = フィードバック ID
// LK: 0x140 + モータ ID。送受信とも同じ ID
// 配列の順は FR, FL, BR, BL（前右、前左、後右、後左）
constexpr uint32_t DM_VELOCITY_CMD_BASE = 0x200;

// CAN1: 首
constexpr uint32_t YAW_ID = 0x141;       // MHF5005（LK）
constexpr uint32_t PITCH_CAN_ID = 0x01;  // DM3507、MIT
constexpr uint32_t PITCH_MST_ID = 0x11;
constexpr uint32_t ROLL_CAN_ID = 0x02;   // DM3507、MIT
constexpr uint32_t ROLL_MST_ID = 0x12;

// CAN2: 車体
constexpr uint32_t CHASSIS_CAN_ID[4] = {0x03, 0x04, 0x05, 0x06};  // DM2325、速度モード
constexpr uint32_t CHASSIS_MST_ID[4] = {0x13, 0x14, 0x15, 0x16};
constexpr uint32_t LEG_CAN_ID[4] = {0x07, 0x08, 0x09, 0x0A};  // DM4310、MIT
constexpr uint32_t LEG_MST_ID[4] = {0x17, 0x18, 0x19, 0x1A};

}  // namespace board

#endif  // GAMMA_TURRET_FW__BOARD_HPP_
