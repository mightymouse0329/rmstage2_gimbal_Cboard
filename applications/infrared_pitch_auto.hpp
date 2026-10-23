#ifndef INFRARED_PITCH_AUTO_HPP
#define INFRARED_PITCH_AUTO_HPP
#include <cstdint>
#include "can.h"

namespace infrared_pitch_auto {
enum class Stage : uint8_t {
  WaitImu=0, WaitFeedback=1, SeekPitch=2, MoveDefault=3, WaitInfrared=4,
  Descent=5, DescentHold=6, FoldBack=7, FoldHold=8, PitchStart=9,
  Tracking=10, Holding=11, Fault=12
};
enum class Fault : uint8_t {
  None=0, Feedback=1, Timing=2, SeekTimeout=3, NoSeekMotion=4, SeekTravel=5,
  Overspeed=6, MoveTimeout=7, Preparation=8, Following=9, Imu=10
};
struct Status {
  Stage stage;
  Fault fault;
  uint32_t stamp_ms, infrared_elapsed_ms, tracking_elapsed_ms;
  float m2006_relative_rad, m2006_target_rad, m2006_torque_nm, m2006_feedback_nm;
  float pitch_torque_nm, yaw_torque_nm;
  float distance_cm, calibration_progress;
  bool infrared_triggered, fold_stable, handed_over;
  // Frozen before overspeed zeroes the outputs. Do not confuse stopped speed with fault speed.
  float fault_pitch_speed_rad_s, fault_m2006_speed_rad_s;
  uint8_t fault_stage;
};
struct HoldOutput {
  float yaw_target_rad, pitch_target_rad, yaw_torque_nm, pitch_torque_nm;
};
bool selected();
bool preparation_active();
void receive_can1(const CAN_RxHeaderTypeDef &, uint8_t *, uint32_t now_ms);
void snapshot(Status & out);
void run();  // Runs in the EXISTING homing task; never creates an RTOS task.
// Called by the existing IMU follower only during actual trajectory tracking.
bool follow_stop_due(uint32_t now_ms);
HoldOutput hold_follow_pose(float yaw, float pitch, float yaw_speed, float pitch_speed);
void report_follow_fault();
}
#endif
