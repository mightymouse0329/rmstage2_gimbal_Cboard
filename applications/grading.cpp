#include "grading.hpp"

#include <cmath>
#include <limits>
#include "FreeRTOS.h"
#include "task.h"
#include "control_constants.hpp"
#include "homing.hpp"
#include "pitch_start.hpp"
#include "gimbal_imu.hpp"
#include "trajectory_follow.hpp"

void yaw_detect(float gimbal_angle, float plan_angle, float* fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }
  constexpr float alpha = 0.001f;
  constexpr float threshold = 0.5f / 57.3f;
  static uint8_t fire = 0;
  fire = (std::fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;
  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

void pitch_detect(float gimbal_angle, float plan_angle, float* fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }
  constexpr float alpha = 0.001f;
  constexpr float threshold = 1.5f / 57.3f;
  static uint8_t fire = 0;
  fire = (std::fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;
  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

namespace grading {
namespace {
Result latest = {};
}

Result read()
{
  taskENTER_CRITICAL();
  const Result value = latest;
  taskEXIT_CRITICAL();
  return value;
}

void run()
{
  static_assert(configTICK_RATE_HZ == 1000, "Grading requires 1ms RTOS ticks");
  constexpr TickType_t period = pdMS_TO_TICKS(1);
  gimbal_imu::begin();  // Sensor setup runs here; no new FreeRTOS task.
  TickType_t wake = xTaskGetTickCount();
  Result score = {};  // Both ratios start at zero, as required by the filter.

  for (;;) {
    vTaskDelayUntil(&wake, period);
    gimbal_imu::update();
    const auto imu = gimbal_imu::read();
    const auto state = trajectory_follow::read();
    const uint32_t now = xTaskGetTickCount();
    // Only the actual angles come from the IMU; the plan comes from the
    // controller reference. Zeros were aligned before any motor motion.
    score.imu_ready = imu.ready;
    score.follow_fault = state.fault;
    score.target_yaw_rad = state.yaw.target_angle_rad;
    score.target_pitch_rad = state.pitch.target_angle_rad;
    const float missing = std::numeric_limits<float>::quiet_NaN();
    score.actual_yaw_rad = imu.valid ? imu.yaw_rad : missing;
    score.actual_pitch_rad = imu.valid ? imu.pitch_rad : missing;
    const int32_t sample_offset = static_cast<int32_t>(imu.stamp_ms-state.stamp_ms);
    const int32_t max_offset = control_constants::FOLLOW_MAX_STEP_MS;
    const bool owner_allows = state.prepared_sequence ?
      state.pitch_sequence_phase == trajectory_follow::PitchSequencePhase::Tracking :
      (!homing::selected() && !pitch_start::selected());
    const bool tracking = imu.valid && owner_allows &&
      state.active && state.calibrated &&
      state.feedback_fresh && state.fault == trajectory_follow::FAULT_NONE &&
      now - state.stamp_ms <= control_constants::FOLLOW_MAX_STEP_MS &&
      !control_constants::FOLLOW_CONSTANT_TORQUE_TEST &&
      sample_offset >= -max_offset && sample_offset <= max_offset;
    // The plan remains visible after a latched motion fault. Only scoring is
    // gated by tracking; do not replace the reference with a constant on stop.
    if (tracking || (state.prepared_sequence && state.pitch_reference_started)) {
      // Evaluate the SAME plan at the IMU timestamp, avoiding a one-tick
      // phase error between the separate control and grading tasks.
      constexpr uint32_t scale = control_constants::FOLLOW_TRAJECTORY_TIME_SCALE;
      constexpr int32_t period_ms = TRAJECTORY_PERIOD_MS * scale;
      const int32_t aligned_offset = sample_offset >= -max_offset && sample_offset <= max_offset ?
        sample_offset : 0;
      int32_t phase = static_cast<int32_t>(state.trajectory_elapsed_ms) + aligned_offset;
      if (phase < 0) phase += period_ms;
      const auto reference = state.prepared_sequence ?
        trajectory_follow::sample_reference(static_cast<uint32_t>(phase % period_ms)) :
        trajectory_sample_at_ms((phase % period_ms) / scale);
      score.target_yaw_rad = reference.yaw;
      score.target_pitch_rad = reference.pitch;
    }
    score.yaw_active = tracking && !state.pitch_only && std::isfinite(imu.yaw_rad) &&
      std::isfinite(state.yaw.target_angle_rad);
    score.pitch_active = tracking && (state.prepared_sequence || !control_constants::FOLLOW_SINGLE_YAW) &&
      std::isfinite(imu.pitch_rad) && std::isfinite(state.pitch.target_angle_rad);

    if (score.yaw_active) {
      yaw_detect(imu.yaw_rad, score.target_yaw_rad, &score.yaw_ratio);
      ++score.yaw_samples;
    }
    if (score.pitch_active) {
      pitch_detect(imu.pitch_rad, score.target_pitch_rad, &score.pitch_ratio);
      ++score.pitch_samples;
    }
    score.stamp_ms = now;
    taskENTER_CRITICAL();
    latest = score;
    taskEXIT_CRITICAL();

    // Do not replay missed ticks as several samples of the same instant.
    const TickType_t finished_at = xTaskGetTickCount();
    if (finished_at - wake >= period) wake = finished_at;
  }
}
}  // namespace grading

extern "C" void grading_task(void const* argument)
{
  (void)argument;
  grading::run();
}
