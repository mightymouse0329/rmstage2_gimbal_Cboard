#ifndef PITCH_START_HPP
#define PITCH_START_HPP
#include <cstdint>
#include "can.h"
namespace pitch_start {
enum class Stage : uint8_t { Idle=0, WaitFeedback=1, SeekPitch=2, MovePitch=3, Done=4, Fault=5 };
enum class Fault : uint8_t { None=0, Config=1, Feedback=2, Timing=3, SeekTimeout=4,
  SeekTravel=5, MoveTimeout=6, PitchOverspeed=7, MoveOvershoot=8,
  NoSeekMotion=9, M2006Overspeed=10 };
struct Status {
  Stage stage;
  Fault fault;
  float pitch_angle_rad;
  float m2006_relative_rad;
  float pitch_return_relative_rad;
  float pitch_return_target_relative_rad;
  float pitch_torque_nm;
  float m2006_torque_nm;
  float pitch_speed_rad_s;
  float m2006_speed_rad_s;
  float fault_pitch_speed_rad_s;  // Latched speeds at the fault; inspect in debugger.
  float fault_m2006_speed_rad_s;
  float pitch_feedback_torque_nm;
  float m2006_feedback_torque_nm;
  uint32_t stall_elapsed_ms;
  float pitch_zero_rad;  // GM6020 final 60-degree target; RAM only.
  float m2006_zero_rad;
  float pitch_up_imu_direction;  // Raw IMU sign observed while GM6020 moves positively.
  bool homed;
  bool m2006_locked;  // 1 = M2006 hold engaged
  bool feedback_fresh;
  uint32_t stamp_ms;
  uint32_t pitch_rx_frames, m2006_rx_frames, tx_errors;
};
bool selected();
// Task-context commands; no automatic retry after a fault.
void start();
void cancel();
Status snapshot();
// Accepted only after successful preparation. Keeps M2006 hold and transfers only GM6020.
bool submit_pitch_follow_torque(float torque_nm, uint32_t now_ms);
bool receive_can1(const CAN_RxHeaderTypeDef &, uint8_t *, uint32_t now_ms);
void transmit_can1(uint32_t now_ms);
}
extern "C" void pitch_start_task(void const * argument);
#endif
