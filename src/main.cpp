// gamma_turret_fw: 制御本体（設計書 docs/superpowers/specs/2026-10-23-gamma-control-design.md）
//
//   Comm          PC と rabcl の共通プロトコルで通信（指令の受け口、関節状態の送信）
//   ImuStore      IMU の共有状態。ImuTask（BNO055、UART3、100 Hz）が書く。初期化できるまで valid = false
//   Controller    指令・フィードバック・IMU から関節ごとの CAN 指令を作る
//   CanScheduler  esp_timer で等間隔に 1 関節ずつ送る（CAN2: 0.5 ms × 8 = 各 250 Hz、CAN1: 0.67 ms × 3 = 各 500 Hz）
//   CanReceiver   バスごとの受信タスク。フィードバックを Controller と Comm に渡す
//
// main はこれらを生成して Start し、1 秒ごとに状態をログするだけ。
// 脚 4 本の試験版（HOLD_POSITION）は commit dd6b5af の main.cpp、通信速度の掃引試験は 8a07f66 にある。

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rabcl_esp32/can_bus.hpp>
#include <rabcl_esp32/debug_led.hpp>
#include <rabcl_esp32/pwm_out.hpp>
#include <rabcl_esp32/serial_port.hpp>

#include "board.hpp"
#include "can_sched.hpp"
#include "comm.hpp"
#include "control.hpp"
#include "imu.hpp"
#include "imu_task.hpp"
#include "robot.hpp"

namespace
{

constexpr const char * TAG = "main";
constexpr uint32_t CONTROL_TICK_MS = 10;  // Controller::Tick の周期（100 Hz）
constexpr uint32_t CONTROL_TASK_STACK = 4096;
constexpr const char * LEG_NAMES[4] = {"FR", "FL", "BR", "BL"};

using rabcl_esp32::CanBus;
using rabcl_esp32::CanBusConfig;
using rabcl_esp32::DebugLed;
using rabcl_esp32::DebugLedConfig;
using rabcl_esp32::PwmOut;
using rabcl_esp32::PwmOutConfig;
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

struct ControlTaskArgs
{
  Controller * controller;
  PwmOut * esc_right;
  PwmOut * esc_left;
};

void ControlTask(void * arg)
{
  auto * args = static_cast<ControlTaskArgs *>(arg);
  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TICK_MS));
    args->controller->Tick(esp_timer_get_time());
    // 射出 ESC: Tick が決めたパルス幅を 2 本に書く（同じ幅。途絶で待機に戻る）
    const uint32_t fire_us = args->controller->fire_pulse_us();
    args->esc_right->SetPulseUs(fire_us);
    args->esc_left->SetPulseUs(fire_us);
  }
}

void LogBus(const char * name, const CanBus & can, const CanScheduler & sched, const CanReceiver & rx)
{
  const auto c = can.GetCounters();
  const auto s = sched.counters();
  twai_node_status_t st = {};
  can.GetStatus(st);
  ESP_LOGI(TAG, "%s: sent=%lu fail=%lu skipped=%lu enables=%lu | state=%d tec=%u rec=%u bus_error=%lu rx_dropped=%lu bus_off=%lu unknown_rx=%lu",
           name, static_cast<unsigned long>(s.sent), static_cast<unsigned long>(s.send_fail),
           static_cast<unsigned long>(s.skipped), static_cast<unsigned long>(s.enables),
           st.state, st.tx_error_count, st.rx_error_count,
           static_cast<unsigned long>(c.bus_error), static_cast<unsigned long>(c.rx_dropped),
           static_cast<unsigned long>(c.bus_off), static_cast<unsigned long>(rx.unknown_frames()));
}

void LogLegs(const Controller & controller)
{
  for (size_t i = 0; i < 4; ++i) {
    const auto s = controller.GetStatus(static_cast<robot::Joint>(robot::LEG_FR + i));
    ESP_LOGI(TAG, "leg %s: %s pos=%.3f target=%.3f vel=%.3f tor=%.3f Nm t_rotor=%.0f err=%u rx=%lu%s%s",
             LEG_NAMES[i], s.rx == 0 ? "NO RESPONSE" : (s.enabled ? "enabled" : "disabled"),
             static_cast<double>(s.info.position_), static_cast<double>(s.target),
             static_cast<double>(s.info.velocity_), static_cast<double>(s.info.torque_),
             static_cast<double>(s.info.temperature_), s.err, static_cast<unsigned long>(s.rx),
             s.tripped ? " TRIPPED" : "", s.stale ? " NO-FEEDBACK" : "");
  }
}

void LogWheels(const Controller & controller)
{
  for (size_t i = 0; i < 4; ++i) {
    const auto s = controller.GetStatus(static_cast<robot::Joint>(robot::WHEEL_FR + i));
    ESP_LOGI(TAG, "wheel %s: %s vel=%.3f target=%.3f tor=%.3f Nm t_rotor=%.0f err=%u rx=%lu%s",
             LEG_NAMES[i], s.rx == 0 ? "NO RESPONSE" : (s.enabled ? "enabled" : "disabled"),
             static_cast<double>(s.info.velocity_), static_cast<double>(s.target),
             static_cast<double>(s.info.torque_), static_cast<double>(s.info.temperature_),
             s.err, static_cast<unsigned long>(s.rx), s.stale ? " NO-FEEDBACK" : "");
  }
}

void LogYaw(const Controller & controller)
{
  const auto s = controller.GetStatus(robot::YAW);
  ESP_LOGI(TAG, "yaw: %s pos=%.3f target=%.3f vel=%.3f cur=%.2f A rx=%lu",
           s.rx == 0 ? "NO RESPONSE" : "ok", static_cast<double>(s.info.position_),
           static_cast<double>(s.target), static_cast<double>(s.info.velocity_),
           static_cast<double>(s.info.current_), static_cast<unsigned long>(s.rx));
  const auto p = controller.GetStatus(robot::PITCH);
  ESP_LOGI(TAG, "pitch: %s pos=%.3f target=%.3f vel=%.3f tor=%.3f Nm t_rotor=%.0f err=%u rx=%lu%s",
           p.rx == 0 ? "NO RESPONSE" : (p.enabled ? "enabled" : "disabled"),
           static_cast<double>(p.info.position_), static_cast<double>(p.target),
           static_cast<double>(p.info.velocity_), static_cast<double>(p.info.torque_),
           static_cast<double>(p.info.temperature_), p.err, static_cast<unsigned long>(p.rx),
           p.stale ? " NO-FEEDBACK" : "");
}

void LogImu(const ImuTask & task, const ImuStore & store)
{
  const auto c = task.counters();
  const ImuState s = store.Get();
  ESP_LOGI(TAG, "IMU: %s %s samples=%lu errors=%lu reinits=%lu calib=0x%02X | yaw=%.3f pitch=%.3f roll=%.3f | gyro %.2f %.2f %.2f",
           rabcl::BNO055Sequencer::StateName(c.state), s.valid ? "valid" : "INVALID",
           static_cast<unsigned long>(c.samples),
           static_cast<unsigned long>(c.errors), static_cast<unsigned long>(c.reinits), c.calib_stat,
           static_cast<double>(s.attitude.yaw), static_cast<double>(s.attitude.pitch),
           static_cast<double>(s.attitude.roll),
           static_cast<double>(s.gyro[0]), static_cast<double>(s.gyro[1]), static_cast<double>(s.gyro[2]));
}

void LogPc(const Comm & comm, const Controller & controller)
{
  rabcl::protocol::ChassisVel vel;
  rabcl::protocol::BodyPose pose;
  const bool has_vel = comm.GetChassisVel(vel);
  const bool has_pose = comm.GetBodyPose(pose);
  // モード 5 つ（load fire speed chassis stabilize。未受信は -）と首の目標（yaw / pitch の JOINT_CMD。未受信は -）
  char modes[16];
  for (int i = 0; i < robot::MODE_COUNT; ++i) {
    uint8_t v = 0;
    modes[2 * i] = comm.GetMode(static_cast<uint8_t>(i), v) ? static_cast<char>('0' + (v % 10)) : '-';
    modes[2 * i + 1] = ' ';
  }
  modes[2 * robot::MODE_COUNT - 1] = '\0';
  rabcl::protocol::JointCmd yaw_cmd;
  rabcl::protocol::JointCmd pitch_cmd;
  const bool has_yaw = comm.GetJointCmd(robot::YAW, yaw_cmd);
  const bool has_pitch = comm.GetJointCmd(robot::PITCH, pitch_cmd);
  ESP_LOGI(TAG, "PC: rx_frames=%lu crc_errors=%lu motor_cmd_frames=%lu %s error_flags=0x%02lX%s | vel %.2f %.2f %.2f | pose h=%.2f r=%.2f p=%.2f | modes %s | neck yaw=%s%.2f pitch=%s%.2f | fire=%lu us",
           static_cast<unsigned long>(comm.rx_frames()), static_cast<unsigned long>(comm.crc_errors()),
           static_cast<unsigned long>(comm.motor_command_frames()),
           controller.command_lost() ? "LOST" : "ok", static_cast<unsigned long>(controller.error_flags()),
           controller.motor_power_lost() ? " MOTOR-POWER-LOST" : "",
           has_vel ? static_cast<double>(vel.vx) : 0.0, has_vel ? static_cast<double>(vel.vy) : 0.0,
           has_vel ? static_cast<double>(vel.wz) : 0.0,
           has_pose ? static_cast<double>(pose.height) : 0.0, has_pose ? static_cast<double>(pose.roll) : 0.0,
           has_pose ? static_cast<double>(pose.pitch) : 0.0, modes,
           has_yaw ? "" : "-", has_yaw ? static_cast<double>(yaw_cmd.position) : 0.0,
           has_pitch ? "" : "-", has_pitch ? static_cast<double>(pitch_cmd.position) : 0.0,
           static_cast<unsigned long>(controller.fire_pulse_us()));
}

}  // namespace

#ifndef GAMMA_ST3215_TEST  // 試験モード（st3215_test.cpp）のときは通常の app_main を外す
extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "gamma_turret_fw start (reset reason: %d)", static_cast<int>(esp_reset_reason()));

  // 射出 ESC: 起動直後から待機パルスを出す（アーミング。校正シーケンスは出さない。復電後に待たせない）
  static PwmOut esc_right(PwmOutConfig{board::ESC_RIGHT_PIN, board::ESC_LEDC_TIMER, board::ESC_RIGHT_CHANNEL,
      board::ESC_PWM_HZ, LEDC_TIMER_14_BIT, robot::FIRE_IDLE_US});
  static PwmOut esc_left(PwmOutConfig{board::ESC_LEFT_PIN, board::ESC_LEDC_TIMER, board::ESC_LEFT_CHANNEL,
      board::ESC_PWM_HZ, LEDC_TIMER_14_BIT, robot::FIRE_IDLE_US});
  ESP_ERROR_CHECK(esc_right.Start());
  ESP_ERROR_CHECK(esc_left.Start());
  ESP_LOGI(TAG, "ESC PWM on G%d / G%d: %lu Hz, idle %lu us", static_cast<int>(board::ESC_RIGHT_PIN),
           static_cast<int>(board::ESC_LEFT_PIN), static_cast<unsigned long>(board::ESC_PWM_HZ),
           static_cast<unsigned long>(robot::FIRE_IDLE_US));

  static DebugLed led3(DebugLedConfig{board::DEBUG_LED3_PIN, true});
  ESP_ERROR_CHECK(led3.Start());

  static CanBus can1(MakeCanConfig(board::CAN1_TX_PIN, board::CAN1_RX_PIN));
  static CanBus can2(MakeCanConfig(board::CAN2_TX_PIN, board::CAN2_RX_PIN));
  ESP_ERROR_CHECK(can1.Start());
  ESP_ERROR_CHECK(can2.Start());

  static Comm comm(MakePcSerialConfig());
  ESP_ERROR_CHECK(comm.Start());
  ESP_LOGI(TAG, "PC link on UART%d: tx=G%d rx=G%d %lu bps", static_cast<int>(board::PC_UART_PORT),
           static_cast<int>(board::PC_UART_TX_PIN), static_cast<int>(board::PC_UART_RX_PIN),
           static_cast<unsigned long>(board::PC_UART_BAUD));

  static ImuStore imu;
  static Controller controller(comm, imu);
  static ImuTask imu_task(imu, comm, controller);
  if (imu_task.Start() != ESP_OK) {
    ESP_LOGE(TAG, "IMU task failed to start; running without IMU");  // IMU のためにモータの起動を止めない
  }
  ESP_LOGI(TAG, "IMU (BNO055) on UART%d: tx=G%d rx=G%d", static_cast<int>(board::IMU_UART_PORT),
           static_cast<int>(board::IMU_UART_TX_PIN), static_cast<int>(board::IMU_UART_RX_PIN));

  static CanReceiver rx1(can1, robot::Bus::CAN1, controller, comm, "can1_rx");
  static CanReceiver rx2(can2, robot::Bus::CAN2, controller, comm, "can2_rx");
  ESP_ERROR_CHECK(rx1.Start());
  ESP_ERROR_CHECK(rx2.Start());

  static ControlTaskArgs control_args{&controller, &esc_right, &esc_left};
  if (xTaskCreate(ControlTask, "control", CONTROL_TASK_STACK, &control_args, 5, nullptr) != pdPASS) {
    ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
  }

  static CanScheduler sched1(
    can1, controller, robot::CAN1_ORDER, robot::CAN1_JOINT_COUNT, robot::CAN1_SLOT_US, "can1_tx");
  static CanScheduler sched2(
    can2, controller, robot::CAN2_ORDER, robot::CAN2_JOINT_COUNT, robot::CAN2_SLOT_US, "can2_tx");
  ESP_ERROR_CHECK(sched1.Start());
  ESP_ERROR_CHECK(sched2.Start());
  ESP_LOGI(TAG, "schedulers: CAN1 %lu us x %u joints, CAN2 %lu us x %u joints, control tick %lu ms",
           static_cast<unsigned long>(robot::CAN1_SLOT_US), static_cast<unsigned>(robot::CAN1_JOINT_COUNT),
           static_cast<unsigned long>(robot::CAN2_SLOT_US), static_cast<unsigned>(robot::CAN2_JOINT_COUNT),
           static_cast<unsigned long>(CONTROL_TICK_MS));

  while (true) {
    led3.Toggle();
    LogWheels(controller);
    LogLegs(controller);
    LogYaw(controller);
    LogBus("CAN1", can1, sched1, rx1);
    LogBus("CAN2", can2, sched2, rx2);
    LogPc(comm, controller);
    LogImu(imu_task, imu);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
#endif  // GAMMA_ST3215_TEST
