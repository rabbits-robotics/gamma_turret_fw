// gamma_turret 基板のピン割り当てと定数（docs/回路図作成リファレンス.md を正とする）
#ifndef GAMMA_TURRET_FW__BOARD_HPP_
#define GAMMA_TURRET_FW__BOARD_HPP_

#include <cstdint>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>

namespace board
{

// PC との UART。rabcl の共通プロトコルで通信する
constexpr uart_port_t PC_UART_PORT = UART_NUM_2;
constexpr gpio_num_t PC_UART_TX_PIN = GPIO_NUM_26;  // → PC 側（USB-UART 変換）RX（10k プルアップ）
constexpr gpio_num_t PC_UART_RX_PIN = GPIO_NUM_27;  // ← PC 側（USB-UART 変換）TX
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
// DAMIAO: 首 Pitch と車輪は beta で設定済みの ID（0x42〜0x46）を当面そのまま使う。CAN_ID が 16 以上だと
//         フィードバック先頭バイトの状態コード（上位ニブル）に CAN_ID の bit4〜7 が OR で混ざるので、CanReceiver でその分をマスクする
//         （異常コードの bit2 が消え、9 低電圧と D 通信途絶が区別できない）。メカ組立後の原点リセットと一緒に DMTool で
//         gamma の値（Pitch 0x01 / 0x11、車輪 0x03〜0x06 / 0x13〜0x16）へ書き換え、マスクを外す（2026-11-15 決定）
//         CAN_ID = 指令 ID（MIT モード）、0x200 + CAN_ID = 指令 ID（速度モード）、MST_ID = CAN_ID + 0x10 = フィードバック ID
// LK: 0x140 + モータ ID。送受信とも同じ ID
// 配列の順は FR, FL, BR, BL（前右、前左、後右、後左）
constexpr uint32_t DM_VELOCITY_CMD_BASE = 0x200;

// CAN1: 首
constexpr uint32_t YAW_ID = 0x141;       // MHF5005（LK）
constexpr uint32_t PITCH_CAN_ID = 0x42;  // DM3507、MIT（beta と同じ）
constexpr uint32_t PITCH_MST_ID = 0x52;
constexpr uint32_t ROLL_CAN_ID = 0x02;   // モータは無い（番号だけ予約。送らない）
constexpr uint32_t ROLL_MST_ID = 0x12;

// 射出 ESC × 2（Motive-RC XR-7A、BLHeli_S）。GPIO 直結の 50 Hz サーボパルス。回路図: G0 = FIRE_RIGHT_MOTOR、G1 = FIRE_LEFT_MOTOR
constexpr gpio_num_t ESC_RIGHT_PIN = GPIO_NUM_0;
constexpr gpio_num_t ESC_LEFT_PIN = GPIO_NUM_1;
constexpr uint32_t ESC_PWM_HZ = 50;
constexpr ledc_timer_t ESC_LEDC_TIMER = LEDC_TIMER_0;  // 2 本で共有
constexpr ledc_channel_t ESC_RIGHT_CHANNEL = LEDC_CHANNEL_0;
constexpr ledc_channel_t ESC_LEFT_CHANNEL = LEDC_CHANNEL_1;

// CAN2: 車体
constexpr uint32_t CHASSIS_CAN_ID[4] = {0x43, 0x44, 0x45, 0x46};  // DM2325、速度モード（beta と同じ）
constexpr uint32_t CHASSIS_MST_ID[4] = {0x53, 0x54, 0x55, 0x56};
constexpr uint32_t LEG_CAN_ID[4] = {0x07, 0x08, 0x09, 0x0A};  // DM4310、MIT
constexpr uint32_t LEG_MST_ID[4] = {0x17, 0x18, 0x19, 0x1A};

}  // namespace board

#endif  // GAMMA_TURRET_FW__BOARD_HPP_
