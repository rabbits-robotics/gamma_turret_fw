// IMU の共有状態。BNO055 ドライバ（別設計）が Set し、Controller が Get する。
// 姿勢は REP-103（yaw 左が正、pitch 前が下がると正、roll 右が下がると正）、IMU はカメラと同じリンクに付く
#ifndef GAMMA_TURRET_FW__IMU_HPP_
#define GAMMA_TURRET_FW__IMU_HPP_

#include <cstdint>

#include <freertos/FreeRTOS.h>

#include <rabcl/controller/body_attitude.hpp>

struct ImuState
{
  rabcl::Attitude attitude = {0.0f, 0.0f, 0.0f};  // [rad] 地面に対する向き
  float gyro[3] = {0.0f, 0.0f, 0.0f};             // [rad/s] x, y, z
  bool valid = false;                             // センサが生きていて値が使えるか
  int64_t time_us = 0;                            // 計測時刻（esp_timer_get_time）
  float neck_yaw = 0.0f;                          // 計測時の首 Yaw の関節角 [rad]（IMU − 関節角 で胴体の向きにするため）
};

class ImuStore
{
public:
  void Set(const ImuState & state)
  {
    taskENTER_CRITICAL(&lock_);
    state_ = state;
    taskEXIT_CRITICAL(&lock_);
  }

  ImuState Get() const
  {
    taskENTER_CRITICAL(&lock_);
    const ImuState copy = state_;
    taskEXIT_CRITICAL(&lock_);
    return copy;
  }

private:
  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  ImuState state_;
};

#endif  // GAMMA_TURRET_FW__IMU_HPP_
