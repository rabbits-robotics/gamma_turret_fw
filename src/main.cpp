// gamma_turret_fw: DM4310 の CAN 疎通試験 + PC との通信（rabcl の共通プロトコル）
// CAN2 につないだ DM4310（前右脚）に 1 秒ごとにイネーブルを送り、受信したフレームをすべてログする。
// MST_ID のフレームは rabcl で復号して位置・速度・トルク・温度を出し、関節状態として Comm に渡す
// （Comm が JOINT_STATE で PC へ送る）。PC から受けた指令は 1 秒ごとにログに出すだけで制御には使わない。
// SEND_MIT_ZERO_CMD を true にすると、イネーブル確認後にゲイン 0 の MIT 指令を 10 Hz で送り、
// フィードバックを連続で受ける（トルクは出ない）。モータは機体に組み込み済みなので段階的に進める。
// 制御・通信のロジックはタスクとして src/tasks/ に足していく。

#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rabcl/interface/can.hpp>
#include <rabcl/protocol/protocol.hpp>
#include <rabcl/utils/type.hpp>
#include <rabcl_esp32/can_bus.hpp>
#include <rabcl_esp32/debug_led.hpp>
#include <rabcl_esp32/serial_port.hpp>

#include "board.hpp"
#include "comm.hpp"
#include "robot.hpp"

namespace
{

constexpr const char * TAG = "main";
constexpr TickType_t LOOP_PERIOD = pdMS_TO_TICKS(100);  // 10 Hz
constexpr uint32_t ENABLE_PERIOD_LOOPS = 10;            // 1 s ごとにイネーブル
constexpr uint32_t COUNTER_PERIOD_LOOPS = 100;          // 10 s ごとにカウンタ
constexpr TickType_t SEND_TIMEOUT = pdMS_TO_TICKS(10);
constexpr bool SEND_MIT_ZERO_CMD = false;  // イネーブル後にゲイン 0 の MIT 指令を送る（段階 2 で true に）
constexpr uint8_t DM_ERR_ENABLED = 1;     // フィードバック D[0] 上位 4 bit

// 試験対象: 前右脚
constexpr robot::Joint TEST_JOINT = robot::LEG_FR;
constexpr uint32_t TEST_CAN_ID = robot::JOINTS[TEST_JOINT].can_id;
constexpr uint32_t TEST_MST_ID = robot::JOINTS[TEST_JOINT].mst_id;

using rabcl_esp32::CanBus;
using rabcl_esp32::CanBusConfig;
using rabcl_esp32::CanFrame;
using rabcl_esp32::DebugLed;
using rabcl_esp32::DebugLedConfig;
using rabcl_esp32::SerialPortConfig;

CanBusConfig MakeCanConfig(gpio_num_t tx, gpio_num_t rx)
{
  CanBusConfig cfg;
  cfg.tx = tx;
  cfg.rx = rx;
  cfg.bitrate = board::CAN_BITRATE;
  return cfg;
}

SerialPortConfig MakePcSerialConfig()
{
  SerialPortConfig cfg;
  cfg.port = board::PC_UART_PORT;
  cfg.tx = board::PC_UART_TX_PIN;
  cfg.rx = board::PC_UART_RX_PIN;
  cfg.baud = board::PC_UART_BAUD;
  return cfg;
}

void LogCounters(int bus, const CanBus & can)
{
  const auto c = can.GetCounters();
  twai_node_status_t st = {};
  can.GetStatus(st);
  ESP_LOGI(TAG, "CAN%d: state=%d tec=%u rec=%u tx_fail=%lu bus_error=%lu rx_dropped=%lu bus_off=%lu recovered=%lu",
           bus, st.state, st.tx_error_count, st.rx_error_count,
           static_cast<unsigned long>(c.tx_fail), static_cast<unsigned long>(c.bus_error),
           static_cast<unsigned long>(c.rx_dropped), static_cast<unsigned long>(c.bus_off),
           static_cast<unsigned long>(c.recovered));
}

void LogFrame(int bus, const CanFrame & f)
{
  ESP_LOGI(TAG, "CAN%d rx: id=0x%03lX dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
           bus, static_cast<unsigned long>(f.id), f.dlc,
           f.data[0], f.data[1], f.data[2], f.data[3], f.data[4], f.data[5], f.data[6], f.data[7]);
}

// DM4310 のフィードバックを復号してログし、関節状態として返す。イネーブル済みなら enabled = true
rabcl::protocol::JointState DecodeDMFeedback(const CanFrame & f, bool & enabled)
{
  rabcl::MotorInfo m = {};
  rabcl::Can::ParseDMMotorFeedback(
    f.data, m, rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, 1.0f);
  const uint8_t err = rabcl::Can::DMFeedbackError(f.data);
  ESP_LOGI(TAG, "DM4310 fb: id=%u err=%u pos=%.3f rad vel=%.3f rad/s tor=%.3f Nm t_mos=%.0f t_rotor=%.0f",
           rabcl::Can::DMFeedbackId(f.data), err,
           static_cast<double>(m.position_), static_cast<double>(m.velocity_),
           static_cast<double>(m.torque_), static_cast<double>(m.temperature_mos_),
           static_cast<double>(m.temperature_));
  enabled = err == DM_ERR_ENABLED;

  rabcl::protocol::JointState js = {};
  js.id = TEST_JOINT;
  js.position = m.position_;
  js.velocity = m.velocity_;
  js.effort = m.torque_;
  js.temperature = m.temperature_;
  js.error = err == DM_ERR_ENABLED ? 0 : err;
  return js;
}

void SendOrLog(CanBus & can, uint32_t id, const uint8_t data[8], const char * what)
{
  CanFrame f;
  f.id = id;
  f.dlc = 8;
  for (int i = 0; i < 8; ++i) {
    f.data[i] = data[i];
  }
  const esp_err_t err = can.Send(f, SEND_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "send %s (id=0x%03lX) failed: %s", what, static_cast<unsigned long>(id), esp_err_to_name(err));
  }
}

void LogPcCommands(const Comm & comm)
{
  rabcl::protocol::ChassisVel vel;
  if (comm.GetChassisVel(vel)) {
    ESP_LOGI(TAG, "PC cmd: chassis vx=%.2f vy=%.2f wz=%.2f",
             static_cast<double>(vel.vx), static_cast<double>(vel.vy), static_cast<double>(vel.wz));
  }
  rabcl::protocol::JointCmd cmd;
  if (comm.GetJointCmd(TEST_JOINT, cmd)) {
    ESP_LOGI(TAG, "PC cmd: leg_fr pos=%.3f vel=%.3f tff=%.3f kp=%.1f kd=%.2f",
             static_cast<double>(cmd.position), static_cast<double>(cmd.velocity),
             static_cast<double>(cmd.effort), static_cast<double>(cmd.kp), static_cast<double>(cmd.kd));
  }
  ESP_LOGI(TAG, "PC link: rx_frames=%lu crc_errors=%lu",
           static_cast<unsigned long>(comm.rx_frames()), static_cast<unsigned long>(comm.crc_errors()));
}

}  // namespace

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "gamma_turret_fw start (reset reason: %d)", static_cast<int>(esp_reset_reason()));

  DebugLed led3(DebugLedConfig{board::DEBUG_LED3_PIN, true});
  ESP_ERROR_CHECK(led3.Start());

  CanBus can1(MakeCanConfig(board::CAN1_TX_PIN, board::CAN1_RX_PIN));
  CanBus can2(MakeCanConfig(board::CAN2_TX_PIN, board::CAN2_RX_PIN));
  ESP_ERROR_CHECK(can1.Start());
  ESP_ERROR_CHECK(can2.Start());
  ESP_LOGI(TAG, "DM4310 test on CAN2: CAN_ID=0x%03lX MST_ID=0x%03lX",
           static_cast<unsigned long>(TEST_CAN_ID), static_cast<unsigned long>(TEST_MST_ID));

  Comm comm(MakePcSerialConfig());
  ESP_ERROR_CHECK(comm.Start());
  ESP_LOGI(TAG, "PC link on UART%d: tx=G%d rx=G%d %lu bps", static_cast<int>(board::PC_UART_PORT),
           static_cast<int>(board::PC_UART_TX_PIN), static_cast<int>(board::PC_UART_RX_PIN),
           static_cast<unsigned long>(board::PC_UART_BAUD));

  uint8_t enable_cmd[8] = {};
  rabcl::Can::PrepareDMMotorEnable(enable_cmd);
  uint8_t mit_zero_cmd[8] = {};
  rabcl::Can::PrepareDMMotorMITCmd(
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, mit_zero_cmd);

  bool enabled = false;
  uint32_t count = 0;
  while (true) {
    ++count;

    // 受信は全部ログする（CAN1 は何もつないでいなくても念のため）
    CanFrame f;
    while (can2.Receive(f, 0)) {
      LogFrame(2, f);
      if (f.id == TEST_MST_ID) {
        comm.SetJointState(DecodeDMFeedback(f, enabled));
      }
    }
    while (can1.Receive(f, 0)) {
      LogFrame(1, f);
    }

    if (count % ENABLE_PERIOD_LOOPS == 0) {
      led3.Toggle();
      SendOrLog(can2, TEST_CAN_ID, enable_cmd, "enable");
      LogPcCommands(comm);
    } else if (SEND_MIT_ZERO_CMD && enabled) {
      SendOrLog(can2, TEST_CAN_ID, mit_zero_cmd, "MIT zero");
    }

    if (count % COUNTER_PERIOD_LOOPS == 0) {
      LogCounters(1, can1);
      LogCounters(2, can2);
    }
    vTaskDelay(LOOP_PERIOD);
  }
}
