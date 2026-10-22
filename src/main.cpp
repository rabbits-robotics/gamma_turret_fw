// gamma_turret_fw: DM4310 の CAN 試験 + PC との通信（rabcl の共通プロトコル）
//
// 起動後、CAN2 の DM4310（前右脚）に 1 秒ごとにイネーブルを送り、応答（err=1）を待つ。
// イネーブルできたら通信速度の掃引試験を行う: 指令周期を 100 → 200 → 500 → 1000 → 2000 Hz と変え、
// 各 5 秒で「送った数 / 送信失敗 / 受けた数 / 応答遅延（µs）」をログに出す。指令はゲイン 0 の MIT 指令
// なのでトルクは出ない。掃引後は 10 Hz のゲイン 0 指令に戻る。
// フィードバックは関節状態として Comm に渡す（Comm が JOINT_STATE で PC へ送る）。
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
constexpr TickType_t LOOP_PERIOD = pdMS_TO_TICKS(100);  // 10 Hz
constexpr uint32_t ENABLE_PERIOD_LOOPS = 10;            // 1 s ごとにイネーブル
constexpr uint32_t COUNTER_PERIOD_LOOPS = 100;          // 10 s ごとにカウンタ
constexpr TickType_t SEND_TIMEOUT = pdMS_TO_TICKS(10);
constexpr uint8_t DM_ERR_ENABLED = 1;  // フィードバック D[0] 上位 4 bit

// 速度掃引試験
constexpr bool RATE_SWEEP = true;
constexpr uint32_t SWEEP_PERIODS_US[] = {10000, 5000, 2000, 1000, 500};  // 100, 200, 500, 1000, 2000 Hz
constexpr uint32_t SWEEP_DURATION_MS = 5000;
constexpr size_t SEND_TS_RING = 256;

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
  cfg.rx_queue_depth = 64;  // 掃引試験で 2 kHz のフィードバックを取りこぼさないため
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

rabcl::protocol::JointState ToJointState(const CanFrame & f, rabcl::MotorInfo & m, uint8_t & err)
{
  rabcl::Can::ParseDMMotorFeedback(
    f.data, m, rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, 1.0f);
  err = rabcl::Can::DMFeedbackError(f.data);
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
  std::memcpy(f.data, data, 8);
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

// --- 速度掃引試験の共有状態 ---

struct RateTest
{
  CanBus * can = nullptr;
  Comm * comm = nullptr;
  uint8_t cmd[8] = {};
  volatile bool enabled = false;      // DM4310 が err=1 を返した
  volatile bool measuring = false;    // 掃引中は受信側が遅延を測る
  volatile uint32_t sent = 0;
  volatile uint32_t send_fail = 0;
  volatile uint32_t rx = 0;
  volatile int64_t lat_sum = 0;
  volatile int64_t lat_max = 0;
  volatile int64_t lat_min = 0;
  int64_t send_ts[SEND_TS_RING] = {};
  rabcl::MotorInfo last = {};

  void ResetStats()
  {
    sent = 0;
    send_fail = 0;
    rx = 0;
    lat_sum = 0;
    lat_max = 0;
    lat_min = 0;
  }
};

// esp_timer から周期的に呼ばれる（esp_timer タスク文脈）
void SendTimerCallback(void * arg)
{
  auto * t = static_cast<RateTest *>(arg);
  CanFrame f;
  f.id = TEST_CAN_ID;
  f.dlc = 8;
  std::memcpy(f.data, t->cmd, 8);
  const uint32_t n = t->sent;
  t->send_ts[n % SEND_TS_RING] = esp_timer_get_time();
  if (t->can->Send(f, 0) == ESP_OK) {
    t->sent = n + 1;
  } else {
    t->send_fail = t->send_fail + 1;
  }
}

// CAN2 の受信を待ち受ける高優先度タスク。フィードバックの遅延を送信時刻との差で測る
void RxTask(void * arg)
{
  auto * t = static_cast<RateTest *>(arg);
  CanFrame f;
  uint32_t n_for_comm = 0;
  while (true) {
    if (!t->can->Receive(f, portMAX_DELAY)) {
      continue;
    }
    const int64_t now = esp_timer_get_time();
    if (f.id != TEST_MST_ID) {
      ESP_LOGI(TAG, "CAN2 rx: id=0x%03lX data=%02X %02X %02X %02X %02X %02X %02X %02X",
               static_cast<unsigned long>(f.id), f.data[0], f.data[1], f.data[2], f.data[3],
               f.data[4], f.data[5], f.data[6], f.data[7]);
      continue;
    }
    uint8_t err = 0;
    const auto js = ToJointState(f, t->last, err);
    t->enabled = err == DM_ERR_ENABLED;
    if (t->measuring) {
      const uint32_t k = t->rx;  // k 番目の応答は k 番目の送信に対応するとみなす
      t->rx = k + 1;
      if (k < t->sent) {
        const int64_t lat = now - t->send_ts[k % SEND_TS_RING];
        t->lat_sum = t->lat_sum + lat;
        if (lat > t->lat_max) {t->lat_max = lat;}
        if (t->lat_min == 0 || lat < t->lat_min) {t->lat_min = lat;}
      }
    }
    if (++n_for_comm % 10 == 0) {
      t->comm->SetJointState(js);  // PC へは 100 Hz 相当まで
    }
  }
}

void RunRateSweep(RateTest & t, CanBus & can2)
{
  esp_timer_handle_t timer = nullptr;
  esp_timer_create_args_t args = {};
  args.callback = SendTimerCallback;
  args.arg = &t;
  args.name = "can_rate";
  ESP_ERROR_CHECK(esp_timer_create(&args, &timer));

  for (uint32_t period_us : SWEEP_PERIODS_US) {
    t.ResetStats();
    t.measuring = true;
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, period_us));
    vTaskDelay(pdMS_TO_TICKS(SWEEP_DURATION_MS));
    ESP_ERROR_CHECK(esp_timer_stop(timer));
    vTaskDelay(pdMS_TO_TICKS(100));  // 最後の応答を待つ
    t.measuring = false;

    const uint32_t sent = t.sent;
    const uint32_t rx = t.rx;
    const uint32_t lost = sent > rx ? sent - rx : 0;
    const double avg = rx > 0 ? static_cast<double>(t.lat_sum) / rx : 0.0;
    ESP_LOGI(TAG, "RATE %4lu Hz: sent=%lu send_fail=%lu rx=%lu lost=%lu (%.1f%%) latency us: min=%lld avg=%.0f max=%lld",
             static_cast<unsigned long>(1000000 / period_us), static_cast<unsigned long>(sent),
             static_cast<unsigned long>(t.send_fail), static_cast<unsigned long>(rx),
             static_cast<unsigned long>(lost), sent > 0 ? 100.0 * lost / sent : 0.0,
             static_cast<long long>(t.lat_min), avg, static_cast<long long>(t.lat_max));
    LogCounters(2, can2);
  }
  ESP_ERROR_CHECK(esp_timer_delete(timer));
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

  static RateTest test;
  test.can = &can2;
  test.comm = &comm;
  rabcl::Can::PrepareDMMotorMITCmd(
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    rabcl::Can::DM4310_PMAX, rabcl::Can::DM4310_VMAX, rabcl::Can::DM4310_TMAX, test.cmd);
  xTaskCreate(RxTask, "can2_rx", 4096, &test, 6, nullptr);

  // イネーブルできるまで 1 秒ごとに送る
  while (!test.enabled) {
    led3.Toggle();
    SendOrLog(can2, TEST_CAN_ID, enable_cmd, "enable");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  ESP_LOGI(TAG, "DM4310 enabled: pos=%.3f rad t_rotor=%.0f",
           static_cast<double>(test.last.position_), static_cast<double>(test.last.temperature_));

  if (RATE_SWEEP) {
    RunRateSweep(test, can2);
  }

  // 掃引後: 10 Hz のゲイン 0 指令（段階 2 と同じ）
  uint32_t count = 0;
  while (true) {
    ++count;
    SendOrLog(can2, TEST_CAN_ID, test.cmd, "MIT zero");
    if (count % ENABLE_PERIOD_LOOPS == 0) {
      led3.Toggle();
      ESP_LOGI(TAG, "DM4310: pos=%.3f rad vel=%.3f rad/s tor=%.3f Nm t_mos=%.0f t_rotor=%.0f enabled=%d",
               static_cast<double>(test.last.position_), static_cast<double>(test.last.velocity_),
               static_cast<double>(test.last.torque_), static_cast<double>(test.last.temperature_mos_),
               static_cast<double>(test.last.temperature_), test.enabled ? 1 : 0);
      LogPcCommands(comm);
    }
    if (count % COUNTER_PERIOD_LOOPS == 0) {
      LogCounters(1, can1);
      LogCounters(2, can2);
    }
    vTaskDelay(LOOP_PERIOD);
  }
}
