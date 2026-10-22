#ifndef GIMBAL_IMU_HPP
#define GIMBAL_IMU_HPP

#include <cstdint>

namespace gimbal_imu {
enum class Phase : uint8_t { Starting=0, Warmup=1, Calibrating=2, Ready=3, Fault=4 };
enum class Fault : uint8_t { None=0, Init=1, Spi=2, Timing=3, Range=4 };
struct State {
  float yaw_rad;                 // Aligned trajectory coordinates, radians.
  float pitch_rad;
  float raw_yaw_rad;             // Fused body attitude, NOT encoder angles.
  float raw_pitch_rad;
  float bias_rad_s[3];
  float acceleration_norm_mps2;
  float max_raw_gyro_rad_s;
  float calibration_std_rad_s;
  uint32_t calibration_elapsed_ms;
  uint8_t calibration_restart_reason;  // 1 motion, 2 gravity, 3 noise/mean drift, 4 gap.
  uint32_t stamp_ms;
  uint32_t samples;
  uint32_t calibration_samples;
  uint32_t calibration_restarts;
  uint32_t max_interval_ms;
  int init_error;
  Phase phase;
  Fault fault;
  bool ready;
  bool aligned;
  bool valid;
};

// Only the existing grading task owns begin/update and the SPI1 sensor.
void begin();
void update();
State read();
bool ready();
// Called once by trajectory_follow at the SAME initial-pose capture as encoders.
bool capture_reference(float initial_yaw_rad, float initial_pitch_rad);
bool capture_reference(float initial_yaw_rad, float initial_pitch_rad, float pitch_direction);
}  // namespace gimbal_imu

#endif
