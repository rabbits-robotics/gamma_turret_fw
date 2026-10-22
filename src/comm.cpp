#include "comm.hpp"

#include <cstring>

#include <esp_timer.h>

namespace proto = rabcl::protocol;

namespace
{

class Lock
{
public:
  explicit Lock(SemaphoreHandle_t m) : m_(m) {xSemaphoreTake(m_, portMAX_DELAY);}
  ~Lock() {xSemaphoreGive(m_);}

private:
  SemaphoreHandle_t m_;
};

}  // namespace

Comm::Comm(const rabcl_esp32::SerialPortConfig & config) : port_(config) {}

Comm::~Comm()
{
  if (tx_task_ != nullptr) {
    vTaskDelete(tx_task_);
  }
  if (rx_task_ != nullptr) {
    vTaskDelete(rx_task_);
  }
  if (mutex_ != nullptr) {
    vSemaphoreDelete(mutex_);
  }
}

esp_err_t Comm::Start()
{
  if (started_) {
    return ESP_ERR_INVALID_STATE;
  }
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr) {
    return ESP_ERR_NO_MEM;
  }
  const esp_err_t err = port_.Start();
  if (err != ESP_OK) {
    return err;
  }
  if (xTaskCreate(RxTaskEntry, "comm_rx", TASK_STACK, this, 5, &rx_task_) != pdPASS ||
    xTaskCreate(TxTaskEntry, "comm_tx", TASK_STACK, this, 4, &tx_task_) != pdPASS)
  {
    return ESP_ERR_NO_MEM;
  }
  started_ = true;
  return ESP_OK;
}

// --- 上り ---

void Comm::SetJointState(const proto::JointState & state)
{
  if (state.id >= robot::JOINT_COUNT) {
    return;
  }
  Lock lock(mutex_);
  joints_[state.id] = state;
  joint_present_[state.id] = true;
}

void Comm::SetImu(const proto::Imu & imu)
{
  Lock lock(mutex_);
  imu_ = imu;
  imu_present_ = true;
}

// --- 下り ---

bool Comm::GetJointCmd(uint8_t id, proto::JointCmd & cmd) const
{
  if (id >= robot::JOINT_COUNT) {
    return false;
  }
  Lock lock(mutex_);
  if (!joint_cmd_present_[id]) {
    return false;
  }
  cmd = joint_cmd_[id];
  return true;
}

bool Comm::GetChassisVel(proto::ChassisVel & vel) const
{
  Lock lock(mutex_);
  if (!chassis_vel_present_) {
    return false;
  }
  vel = chassis_vel_;
  return true;
}

bool Comm::GetMode(uint8_t id, uint8_t & value) const
{
  if (id >= robot::MODE_COUNT) {
    return false;
  }
  Lock lock(mutex_);
  if (!mode_present_[id]) {
    return false;
  }
  value = modes_[id];
  return true;
}

// --- 送信タスク ---

void Comm::TxTaskEntry(void * arg)
{
  static_cast<Comm *>(arg)->TxTask();
}

void Comm::TxTask()
{
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t loops = 0;
  while (true) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TX_PERIOD_MS));
    ++loops;

    SendJointStates();

    bool imu_present;
    proto::Imu imu;
    {
      Lock lock(mutex_);
      imu_present = imu_present_;
      imu = imu_;
    }
    if (imu_present) {
      Send(tx_buf_, proto::EncodeImu(imu, tx_buf_, sizeof(tx_buf_)));
    }

    if (loops % INFO_PERIOD_LOOPS == 0) {
      proto::Info info = {};
      info.protocol_version = proto::VERSION;
      info.joint_count = robot::JOINT_COUNT;
      std::memcpy(info.firmware_id, robot::FIRMWARE_ID, sizeof(info.firmware_id));
      Send(tx_buf_, proto::EncodeInfo(info, tx_buf_, sizeof(tx_buf_)));

      proto::System system = {};
      system.bus_voltage = 0.0f;  // v1.0 基板に電圧測定なし
      system.error_flags = 0;
      system.uptime_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
      Send(tx_buf_, proto::EncodeSystem(system, tx_buf_, sizeof(tx_buf_)));
    }
  }
}

void Comm::SendJointStates()
{
  // 持っている関節だけを、1 メッセージ MAX_JOINT_STATES 個ずつに分けて送る
  proto::JointState batch[proto::MAX_JOINT_STATES];
  uint8_t n = 0;
  for (uint8_t id = 0; id < robot::JOINT_COUNT; ++id) {
    {
      Lock lock(mutex_);
      if (!joint_present_[id]) {
        continue;
      }
      batch[n++] = joints_[id];
    }
    if (n == proto::MAX_JOINT_STATES) {
      Send(tx_buf_, proto::EncodeJointState(batch, n, tx_buf_, sizeof(tx_buf_)));
      n = 0;
    }
  }
  if (n > 0) {
    Send(tx_buf_, proto::EncodeJointState(batch, n, tx_buf_, sizeof(tx_buf_)));
  }
}

void Comm::Send(const uint8_t * buf, size_t len)
{
  if (len > 0) {
    port_.Write(buf, len);
  }
}

// --- 受信タスク ---

void Comm::RxTaskEntry(void * arg)
{
  static_cast<Comm *>(arg)->RxTask();
}

void Comm::RxTask()
{
  while (true) {
    const int n = port_.Read(rx_buf_, sizeof(rx_buf_), pdMS_TO_TICKS(20));
    if (n > 0) {
      parser_.Feed(rx_buf_, static_cast<size_t>(n), OnFrame, this);
    }
  }
}

void Comm::OnFrame(proto::Type type, const uint8_t * payload, uint8_t len, void * ctx)
{
  static_cast<Comm *>(ctx)->HandleFrame(type, payload, len);
}

void Comm::HandleFrame(proto::Type type, const uint8_t * payload, uint8_t len)
{
  rx_frames_ = rx_frames_ + 1;
  switch (type) {
    case proto::Type::CHASSIS_VEL: {
        proto::ChassisVel vel;
        if (proto::DecodeChassisVel(payload, len, vel)) {
          Lock lock(mutex_);
          chassis_vel_ = vel;
          chassis_vel_present_ = true;
        }
        break;
      }
    case proto::Type::JOINT_CMD: {
        proto::JointCmd cmds[proto::MAX_JOINT_CMDS];
        uint8_t count = 0;
        if (proto::DecodeJointCmd(payload, len, cmds, proto::MAX_JOINT_CMDS, count)) {
          Lock lock(mutex_);
          for (uint8_t i = 0; i < count; ++i) {
            if (cmds[i].id < robot::JOINT_COUNT) {
              joint_cmd_[cmds[i].id] = cmds[i];
              joint_cmd_present_[cmds[i].id] = true;
            }
          }
        }
        break;
      }
    case proto::Type::MODE: {
        proto::ModeCmd modes[robot::MODE_COUNT * 2];
        uint8_t count = 0;
        if (proto::DecodeMode(payload, len, modes, sizeof(modes) / sizeof(modes[0]), count)) {
          Lock lock(mutex_);
          for (uint8_t i = 0; i < count; ++i) {
            if (modes[i].id < robot::MODE_COUNT) {
              modes_[modes[i].id] = modes[i].value;
              mode_present_[modes[i].id] = true;
            }
          }
        }
        break;
      }
    default:
      // 上りの種別が来たら（ループバック試験）受信数だけ数える
      break;
  }
}
