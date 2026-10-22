#ifndef TRAJECTORY_FOLLOW_HPP
#define TRAJECTORY_FOLLOW_HPP

#include <cstdint>
#include "trajectory_task.hpp"
#include "can.h"

namespace trajectory_follow
{
// 所有角度/角速度都是"关节坐标"：已完成零点、符号与减速比换算。
// 关节角是连续多圈角，不做 ±pi 折算。
struct AxisFeedback
{
  float angle_rad;
  float speed_rad_s;
  uint32_t stamp_ms;
  bool valid;
};

struct Feedback
{
  AxisFeedback yaw;
  AxisFeedback pitch;
};

struct AxisResult
{
  float target_angle_rad;          // 轨迹原始参考角
  float target_speed_rad_s;        // 参考角速度（速度前馈用）
  float target_acceleration_rad_s2;  // 参考角加速度（力矩前馈用）
  float limited_angle_rad;         // 经机械行程限幅后的参考角
  float actual_angle_rad;
  float actual_speed_rad_s;
  float angle_error_rad;
  float speed_feedforward_rad_s;
  float speed_demand_rad_s;        // 位置环输出（斜坡前）
  float speed_command_rad_s;       // 斜坡后，速度环的目标转速
  float accel_feedforward_nm;      // J·α 力矩前馈
  float gravity_feedforward_nm;    // K_g·cos(θ) 重力前馈
  float friction_feedforward_nm;   // 库仑摩擦前馈
  float breakaway_feedforward_nm;  // 静摩擦脱困辅助力矩
  float control_output;            // 力矩指令，N*m
  bool angle_limited;
  bool speed_limited;
  bool output_limited;
};

// 故障码（用于绘图观察，非锁存位）
constexpr uint8_t FAULT_NONE = 0;
constexpr uint8_t FAULT_TRACKING_ERROR = 1;
constexpr uint8_t FAULT_OVERSPEED = 2;
constexpr uint8_t FAULT_FEEDBACK = 3;
constexpr uint8_t FAULT_TIMING = 4;
constexpr uint8_t FAULT_CONFIG = 5;
constexpr uint8_t FAULT_IMU = 6;
constexpr uint8_t FAULT_PRELIFT = 7;
constexpr uint8_t FAULT_BOUNDARY = 8;
enum class PitchSequencePhase : uint8_t { Waiting=0, Prelift=1, Tracking=2, Fault=3 };

struct Result
{
  AxisResult yaw;
  AxisResult pitch;
  uint32_t stamp_ms;
  uint32_t trajectory_elapsed_ms;
  bool configuration_ready;
  bool feedback_fresh;
  bool active;
  bool calibrated;
  bool rc_stop;
  uint8_t fault;  // FAULT_*，停机原因，便于在曲线上定位
  bool pitch_only;
  PitchSequencePhase pitch_sequence_phase;
  bool pitch_reference_started;
  bool pitch_fault_holding;
  bool pitch_boundary_braking;
};

// 任务上下文接口。反馈由 CAN1 适配器每周期刷新，set_feedback 保留兼容。
void set_feedback(const Feedback & feedback);
Result read();  // 给绘图用的一致性快照
// Same slow reference for control and grading; raw trajectory files remain untouched.
TrajectoryPoint sample_reference(uint32_t elapsed_ms);
void set_enabled(bool enabled);
// 在停机状态、物理摆到期望初始姿态时调用；有新鲜反馈后异步生效。
bool request_initial_pose_capture();

// 只由现有的 CAN1 分发器 / 底盘发送器调用，本模块不初始化 CAN、不定义 HAL 回调。
bool receive_can1(const CAN_RxHeaderTypeDef & header, uint8_t * bytes, uint32_t stamp_ms);
void transmit_can1(uint32_t now_ms, bool permitted);

struct TransportStatus
{
  uint32_t yaw_frames;
  uint32_t pitch_frames;
  uint32_t tx_frames;
  uint32_t tx_busy;
  uint32_t tx_errors;
};
TransportStatus read_transport();
}  // namespace trajectory_follow

extern "C" void trajectory_follow_task(void const * argument);

#endif  // TRAJECTORY_FOLLOW_HPP
