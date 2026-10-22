// gamma_turret_fw: 脚 4 本（DM4310、CAN2）の試験 + PC との通信（rabcl の共通プロトコル）
//
// 起動後、4 本の脚に 1 秒ごとにイネーブルを送り、各脚のフィードバック（位置・速度・トルク・温度）を
// 1 秒ごとにまとめてログする（接続確認）。HOLD_POSITION が true なら、4 本すべてがイネーブルできた
// あとで各脚の目標を可動域の中点へなめらかに動かし、小さなゲインで保持する。指令は esp_timer から
// 1 ms ごとに 1 本ずつ回し、各脚 250 Hz（CAN2 の仕様）。トルクが上限を超えた脚はゲイン 0 に落とす。
// フィードバックは関節状態として Comm に渡す（Comm が JOINT_STATE で PC へ送る）。
// 通信速度の掃引試験は commit 8a07f66 の main.cpp にある。
// 制御・通信のロジックはタスクとして src/tasks/ に足していく。

#include <cstring>

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
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
constexpr TickType_t SEND_TIMEOUT = pdMS_TO_TICKS(10);
constexpr uint8_t DM_ERR_ENABLED = 1;  // フィードバック D[0] 上位 4 bit

// 脚 4 本
constexpr size_t LEG_COUNT = 4;
constexpr robot::Joint LEGS[LEG_COUNT] = {robot::LEG_FR, robot::LEG_FL, robot::LEG_BR, robot::LEG_BL};
constexpr const char * LEG_NAMES[LEG_COUNT] = {"FR", "FL", "BR", "BL"};

// 位置保持（可動域の実測 0〜約 1.6 rad の中点）。目標は起動時の位置から HOLD_RAMP でなめらかに動かす
constexpr bool HOLD_POSITION = true;         // false にすると接続確認（イネーブルとログ）だけ
constexpr float HOLD_TARGET = 0.8f;          // [rad]
constexpr float HOLD_RAMP = 0.5f;            // [rad/s]
constexpr float HOLD_KP = 1.0f;              // [Nm/rad]（柔らかめ。4 点接地のため）
constexpr float HOLD_KD = 0.1f;              // [Nm·s/rad]
constexpr float HOLD_TORQUE_LIMIT = 4.0f;    // [Nm] 超えた脚はゲイン 0 に落とす（DM4310 定格 10 Nm）
constexpr uint32_t CMD_SLOT_US = 1000;       // 1 ms ごとに 1 本 → 各脚 250 Hz

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
  cfg.rx_queue_depth = 64;
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

void LogPcCommands(const Comm & comm)
{
  rabcl::protocol::ChassisVel vel;
  if (comm.GetChassisVel(vel)) {
    ESP_LOGI(TAG, "PC cmd: chassis vx=%.2f vy=%.2f wz=%.2f",
             static_cast<double>(vel.vx), static_cast<double>(vel.vy), static_cast<double>(vel.wz));
  }
  ESP_LOGI(TAG, "PC link: rx_frames=%lu crc_errors=%lu",
           static_cast<unsigned long>(comm.rx_frames()), static_cast<unsigned long>(comm.crc_errors()));
}

// --- 脚の共有状態 ---

struct LegState
{
  rabcl::MotorInfo last = {};
  volatile bool enabled = false;
  volatile uint8_t err = 0;
  volatile uint32_t rx = 0;
  uint32_t rx_for_comm = 0;
  float target = 0.0f;
  volatile bool tripped = false;
  volatile uint32_t sent = 0;
  volatile uint32_t send_fail = 0;
};

struct Legs
{
  CanBus * can = nullptr;
  Comm * comm = nullptr;
  uint8_t zero_cmd[8] = {};  // ゲイン 0 の MIT 指令
  LegState leg[LEG_COUNT];
  volatile uint32_t next = 0;  // 次に指令を送る脚
};

int LegIndexForMst(uint32_t id)
{
  for (size_t i = 0; i < LEG_COUNT; ++i) {
    if (robot::JOINTS[LEGS[i]].mst_id == id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// CAN2 の受信を待ち受けるタスク。各脚のフィードバックを復号して共有状態に置く
void RxTask(void * arg)
{
  auto * legs = static_cast<Legs *>(arg);
  CanFrame f;
  while (true) {
    if (!legs->can->Receive(f, portMAX_DELAY)) {
      continue;
    }
    const int i = LegIndexForMst(f.id);
    if (i < 0) {
      ESP_LOGI(TAG, "CAN2 rx (other): id=0x%03lX data=%02X %02X %02X %02X %02X %02X %02X %02X",
               static_cast<unsigned long>(f.id), f.data[0], f.data[1], f.data[2], f.data[3],
               f.data[4], f.data[5], f.data[6], f.data[7]);
      continue;
    }
    LegState & leg = legs->leg[i];
    rabcl::Can::ParseDMMotorFeedback(
      f.data, leg.last, rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, 1.0f);
    leg.err = rabcl::Can::DMFeedbackError(f.data);
    leg.enabled = leg.err == DM_ERR_ENABLED;
    leg.rx = leg.rx + 1;
    if (++leg.rx_for_comm % 10 == 0 || !HOLD_POSITION) {
      rabcl::protocol::JointState js = {};
      js.id = LEGS[i];
      js.position = leg.last.position_;
      js.velocity = leg.last.velocity_;
      js.effort = leg.last.torque_;
      js.temperature = leg.last.temperature_;
      js.error = leg.enabled ? 0 : leg.err;
      legs->comm->SetJointState(js);
    }
  }
}

void SendTo(CanBus & can, robot::Joint joint, const uint8_t data[8], LegState * stat)
{
  CanFrame f;
  f.id = robot::JOINTS[joint].can_id;
  f.dlc = 8;
  std::memcpy(f.data, data, 8);
  const esp_err_t err = can.Send(f, stat == nullptr ? SEND_TIMEOUT : 0);
  if (stat != nullptr) {
    if (err == ESP_OK) {
      stat->sent = stat->sent + 1;
    } else {
      stat->send_fail = stat->send_fail + 1;
    }
  } else if (err != ESP_OK) {
    ESP_LOGW(TAG, "send to id=0x%03lX failed: %s", static_cast<unsigned long>(f.id), esp_err_to_name(err));
  }
}

// esp_timer から 1 ms ごとに呼ばれ、脚を 1 本ずつ回す（各脚 250 Hz）
void HoldTimerCallback(void * arg)
{
  auto * legs = static_cast<Legs *>(arg);
  const uint32_t i = legs->next % LEG_COUNT;
  legs->next = i + 1;
  LegState & leg = legs->leg[i];

  const float torque = leg.last.torque_;
  if (!leg.tripped && (torque > HOLD_TORQUE_LIMIT || torque < -HOLD_TORQUE_LIMIT)) {
    leg.tripped = true;
  }
  constexpr float ramp_step = HOLD_RAMP * CMD_SLOT_US * LEG_COUNT / 1000000.0f;
  if (leg.target < HOLD_TARGET) {
    leg.target = leg.target + ramp_step > HOLD_TARGET ? HOLD_TARGET : leg.target + ramp_step;
  } else if (leg.target > HOLD_TARGET) {
    leg.target = leg.target - ramp_step < HOLD_TARGET ? HOLD_TARGET : leg.target - ramp_step;
  }
  uint8_t cmd[8];
  if (leg.tripped) {
    std::memcpy(cmd, legs->zero_cmd, 8);
  } else {
    rabcl::Can::PrepareDMMotorMITCmd(
      leg.target, 0.0f, HOLD_KP, HOLD_KD, 0.0f,
      rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, cmd);
  }
  SendTo(*legs->can, LEGS[i], cmd, &leg);
}

void LogLegs(const Legs & legs, bool holding)
{
  for (size_t i = 0; i < LEG_COUNT; ++i) {
    const LegState & leg = legs.leg[i];
    if (holding) {
      ESP_LOGI(TAG, "leg %s: pos=%.3f (target %.3f) vel=%.3f tor=%.3f Nm t_mos=%.0f t_rotor=%.0f rx=%lu sent=%lu fail=%lu%s%s",
               LEG_NAMES[i], static_cast<double>(leg.last.position_), static_cast<double>(leg.target),
               static_cast<double>(leg.last.velocity_), static_cast<double>(leg.last.torque_),
               static_cast<double>(leg.last.temperature_mos_), static_cast<double>(leg.last.temperature_),
               static_cast<unsigned long>(leg.rx), static_cast<unsigned long>(leg.sent),
               static_cast<unsigned long>(leg.send_fail), leg.enabled ? "" : " NOT-ENABLED",
               leg.tripped ? " TRIPPED" : "");
    } else {
      ESP_LOGI(TAG, "leg %s: %s pos=%.3f vel=%.3f tor=%.3f Nm t_mos=%.0f t_rotor=%.0f err=%u rx=%lu",
               LEG_NAMES[i], leg.rx == 0 ? "NO RESPONSE" : (leg.enabled ? "enabled" : "disabled"),
               static_cast<double>(leg.last.position_), static_cast<double>(leg.last.velocity_),
               static_cast<double>(leg.last.torque_), static_cast<double>(leg.last.temperature_mos_),
               static_cast<double>(leg.last.temperature_), leg.err, static_cast<unsigned long>(leg.rx));
    }
  }
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

  Comm comm(MakePcSerialConfig());
  ESP_ERROR_CHECK(comm.Start());
  ESP_LOGI(TAG, "PC link on UART%d: tx=G%d rx=G%d %lu bps", static_cast<int>(board::PC_UART_PORT),
           static_cast<int>(board::PC_UART_TX_PIN), static_cast<int>(board::PC_UART_RX_PIN),
           static_cast<unsigned long>(board::PC_UART_BAUD));

  static Legs legs;
  legs.can = &can2;
  legs.comm = &comm;
  rabcl::Can::PrepareDMMotorMITCmd(
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, legs.zero_cmd);
  for (size_t i = 0; i < LEG_COUNT; ++i) {
    ESP_LOGI(TAG, "leg %s: CAN_ID=0x%03lX MST_ID=0x%03lX", LEG_NAMES[i],
             static_cast<unsigned long>(robot::JOINTS[LEGS[i]].can_id),
             static_cast<unsigned long>(robot::JOINTS[LEGS[i]].mst_id));
  }
  xTaskCreate(RxTask, "can2_rx", 4096, &legs, 6, nullptr);

  uint8_t enable_cmd[8] = {};
  rabcl::Can::PrepareDMMotorEnable(enable_cmd);

  // 接続確認: 4 本すべてがイネーブルできるまで 1 秒ごとにイネーブルを送り、状態をログする
  bool all_enabled = false;
  uint32_t count = 0;
  while (!all_enabled || !HOLD_POSITION) {
    ++count;
    led3.Toggle();
    for (size_t i = 0; i < LEG_COUNT; ++i) {
      SendTo(can2, LEGS[i], enable_cmd, nullptr);
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // 応答を待ってからログ
    LogLegs(legs, false);
    all_enabled = true;
    for (size_t i = 0; i < LEG_COUNT; ++i) {
      all_enabled = all_enabled && legs.leg[i].enabled;
    }
    if (count % 10 == 0) {
      LogCounters(2, can2);
      LogPcCommands(comm);
    }
    vTaskDelay(pdMS_TO_TICKS(900));
  }

  // 位置保持: 各脚の今の位置を目標の初期値にして、1 ms ごとに 1 本ずつ MIT 指令
  for (size_t i = 0; i < LEG_COUNT; ++i) {
    legs.leg[i].target = legs.leg[i].last.position_;
  }
  ESP_LOGW(TAG, "HOLD start: to %.3f rad at %.2f rad/s, kp=%.1f kd=%.2f, %lu Hz per leg (torque limit %.1f Nm)",
           static_cast<double>(HOLD_TARGET), static_cast<double>(HOLD_RAMP),
           static_cast<double>(HOLD_KP), static_cast<double>(HOLD_KD),
           static_cast<unsigned long>(1000000 / (CMD_SLOT_US * LEG_COUNT)), static_cast<double>(HOLD_TORQUE_LIMIT));
  esp_timer_handle_t hold_timer = nullptr;
  esp_timer_create_args_t hold_args = {};
  hold_args.callback = HoldTimerCallback;
  hold_args.arg = &legs;
  hold_args.name = "hold";
  ESP_ERROR_CHECK(esp_timer_create(&hold_args, &hold_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(hold_timer, CMD_SLOT_US));

  count = 0;
  while (true) {
    ++count;
    led3.Toggle();
    LogLegs(legs, true);
    if (count % 10 == 0) {
      LogCounters(2, can2);
      LogPcCommands(comm);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
