// PC（Jetson）との通信タスク。rabcl の共通プロトコル（rabcl::protocol）を SerialPort で流す。
//   上り: 持っている関節の状態（JOINT_STATE）を 100 Hz、IMU は値があれば 100 Hz、INFO / SYSTEM を 1 Hz
//   下り: CHASSIS_VEL / JOINT_CMD / MODE を受けて共有状態に置く（制御側が Get* で読む）
#ifndef GAMMA_TURRET_FW__COMM_HPP_
#define GAMMA_TURRET_FW__COMM_HPP_

#include <cstdint>

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <rabcl/protocol/parser.hpp>
#include <rabcl/protocol/protocol.hpp>
#include <rabcl_esp32/serial_port.hpp>

#include "robot.hpp"

class Comm
{
public:
  explicit Comm(const rabcl_esp32::SerialPortConfig & config);
  ~Comm();
  Comm(const Comm &) = delete;
  Comm & operator=(const Comm &) = delete;

  esp_err_t Start();  // SerialPort を開き、送信タスクと受信タスクを起動する

  // 上り（制御側が書く）
  void SetJointState(const rabcl::protocol::JointState & state);  // state.id < JOINT_COUNT
  void SetImu(const rabcl::protocol::Imu & imu);

  // 下り（制御側が読む）。起動後に 1 度も受けていなければ false
  bool GetJointCmd(uint8_t id, rabcl::protocol::JointCmd & cmd) const;
  bool GetChassisVel(rabcl::protocol::ChassisVel & vel) const;
  bool GetMode(uint8_t id, uint8_t & value) const;

  uint32_t rx_frames() const {return rx_frames_;}
  uint32_t crc_errors() const {return parser_.crc_errors();}

private:
  static constexpr uint32_t TX_PERIOD_MS = 10;        // 100 Hz
  static constexpr uint32_t INFO_PERIOD_LOOPS = 100;  // 1 Hz
  static constexpr size_t RX_CHUNK = 256;
  static constexpr uint32_t TASK_STACK = 4096;

  static void TxTaskEntry(void * arg);
  static void RxTaskEntry(void * arg);
  void TxTask();
  void RxTask();
  static void OnFrame(
    rabcl::protocol::Type type, const uint8_t * payload, uint8_t len, void * ctx);
  void HandleFrame(rabcl::protocol::Type type, const uint8_t * payload, uint8_t len);
  void SendJointStates();
  void Send(const uint8_t * buf, size_t len);

  rabcl_esp32::SerialPort port_;
  rabcl::protocol::Parser parser_;
  SemaphoreHandle_t mutex_ = nullptr;
  TaskHandle_t tx_task_ = nullptr;
  TaskHandle_t rx_task_ = nullptr;
  bool started_ = false;

  // 上り
  rabcl::protocol::JointState joints_[robot::JOINT_COUNT] = {};
  bool joint_present_[robot::JOINT_COUNT] = {};
  rabcl::protocol::Imu imu_ = {};
  bool imu_present_ = false;

  // 下り
  rabcl::protocol::JointCmd joint_cmd_[robot::JOINT_COUNT] = {};
  bool joint_cmd_present_[robot::JOINT_COUNT] = {};
  rabcl::protocol::ChassisVel chassis_vel_ = {};
  bool chassis_vel_present_ = false;
  uint8_t modes_[robot::MODE_COUNT] = {};
  bool mode_present_[robot::MODE_COUNT] = {};

  volatile uint32_t rx_frames_ = 0;
  uint8_t tx_buf_[rabcl::protocol::MAX_FRAME];
  uint8_t rx_buf_[RX_CHUNK];
};

#endif  // GAMMA_TURRET_FW__COMM_HPP_
