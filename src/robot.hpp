// gamma の機体定義（関節番号、モード番号、関節とモータの対応）。
// 関節番号は rabbits_bridge の config/gamma.yaml の joints の並びと同じにする。
// ピンや CAN ID の値そのものは board.hpp に置く。
#ifndef GAMMA_TURRET_FW__ROBOT_HPP_
#define GAMMA_TURRET_FW__ROBOT_HPP_

#include <cstdint>

#include "board.hpp"

namespace robot
{

constexpr char FIRMWARE_ID[8] = "gamma";

// 関節番号（rabcl::protocol の JointState / JointCmd の id）
enum Joint : uint8_t
{
  YAW = 0,   // yaw_joint（MHF5005、CAN1）
  PITCH,     // pitch_joint（DM3507、CAN1）
  ROLL,      // roll_joint（DM3507、CAN1）
  WHEEL_FR,  // right_front_wheel_joint（DM2325、CAN2）
  WHEEL_FL,  // left_front_wheel_joint
  WHEEL_BR,  // right_back_wheel_joint
  WHEEL_BL,  // left_back_wheel_joint
  LEG_FR,    // right_front_leg_joint（DM4310、CAN2）
  LEG_FL,    // left_front_leg_joint
  LEG_BR,    // right_back_leg_joint
  LEG_BL,    // left_back_leg_joint
  JOINT_COUNT
};

// モード番号（rabcl::protocol の ModeCmd の id）。config/gamma.yaml の modes の並び
enum Mode : uint8_t
{
  LOAD = 0,  // 0: 停止, 1: 正転, 2: 逆転
  FIRE,      // 0: 停止, 1: 射出
  SPEED,     // 0: 低速, 1: 高速
  CHASSIS,   // 0: 通常, 1: 無限回転
  MODE_COUNT
};

enum class MotorType : uint8_t
{
  LK_MHF5005,
  DM3507,
  DM2325,
  DM4310,
};

enum class Bus : uint8_t
{
  CAN1,
  CAN2,
};

struct JointSpec
{
  Bus bus;
  MotorType motor;
  uint32_t can_id;  // 指令 ID（DAMIAO は CAN_ID、LK は 0x140 + ID）
  uint32_t mst_id;  // フィードバック ID（DAMIAO は MST_ID、LK は指令と同じ）
};

constexpr JointSpec JOINTS[JOINT_COUNT] = {
  {Bus::CAN1, MotorType::LK_MHF5005, board::YAW_ID, board::YAW_ID},
  {Bus::CAN1, MotorType::DM3507, board::PITCH_CAN_ID, board::PITCH_MST_ID},
  {Bus::CAN1, MotorType::DM3507, board::ROLL_CAN_ID, board::ROLL_MST_ID},
  {Bus::CAN2, MotorType::DM2325, board::CHASSIS_CAN_ID[0], board::CHASSIS_MST_ID[0]},
  {Bus::CAN2, MotorType::DM2325, board::CHASSIS_CAN_ID[1], board::CHASSIS_MST_ID[1]},
  {Bus::CAN2, MotorType::DM2325, board::CHASSIS_CAN_ID[2], board::CHASSIS_MST_ID[2]},
  {Bus::CAN2, MotorType::DM2325, board::CHASSIS_CAN_ID[3], board::CHASSIS_MST_ID[3]},
  {Bus::CAN2, MotorType::DM4310, board::LEG_CAN_ID[0], board::LEG_MST_ID[0]},
  {Bus::CAN2, MotorType::DM4310, board::LEG_CAN_ID[1], board::LEG_MST_ID[1]},
  {Bus::CAN2, MotorType::DM4310, board::LEG_CAN_ID[2], board::LEG_MST_ID[2]},
  {Bus::CAN2, MotorType::DM4310, board::LEG_CAN_ID[3], board::LEG_MST_ID[3]},
};

}  // namespace robot

#endif  // GAMMA_TURRET_FW__ROBOT_HPP_
