// gamma_turret 基板のピン割り当てと定数（docs/回路図作成リファレンス.md を正とする）
#ifndef GAMMA_TURRET_FW__BOARD_HPP_
#define GAMMA_TURRET_FW__BOARD_HPP_

#include <cstdint>

#include <driver/gpio.h>
#include <driver/uart.h>

namespace board
{

// PC（Jetson）との UART。rabcl の共通プロトコルで通信する
constexpr uart_port_t PC_UART_PORT = UART_NUM_2;
constexpr gpio_num_t PC_UART_TX_PIN = GPIO_NUM_26;  // → Jetson RX（10k プルアップ）
constexpr gpio_num_t PC_UART_RX_PIN = GPIO_NUM_27;  // ← Jetson TX
constexpr uint32_t PC_UART_BAUD = 921600;

// IMU（BNO055、基板直載せ、UART モード = J3 接続・J4 接続・J5 除去・J2 開放）。モジュールの送信 → G15、受信 ← G12（10k プルアップ）
// ⚠ J3 と J5 を同時に閉じると VOUT（2.8 V）が GND に短絡して LP5907 が発熱する（2026-11-01 に実測。モジュール 2 台がこれで止まった）
constexpr uart_port_t IMU_UART_PORT = UART_NUM_3;
constexpr gpio_num_t IMU_UART_TX_PIN = GPIO_NUM_12;  // → BNO055 ピン 4（SCL/R）
constexpr gpio_num_t IMU_UART_RX_PIN = GPIO_NUM_15;  // ← BNO055 ピン 3（SDA/T）
constexpr uint32_t IMU_UART_BAUD = 115200;        // BNO055 の UART は固定

// 装填サーボ（ST3215、SRV1）。半二重: TX は 1 kΩ で Data に合流、RX は 1 kΩ で Data 直結（送信はエコーで戻る）
constexpr uart_port_t SERVO_UART_PORT = UART_NUM_1;
constexpr gpio_num_t SERVO_UART_TX_PIN = GPIO_NUM_8;
constexpr gpio_num_t SERVO_UART_RX_PIN = GPIO_NUM_10;
constexpr uint32_t SERVO_UART_BAUD = 1000000;  // ST3215 の出荷時

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
