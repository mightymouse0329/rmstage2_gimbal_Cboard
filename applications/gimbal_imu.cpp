#include "gimbal_imu.hpp"

#include <cmath>
#include "FreeRTOS.h"
#include "task.h"
#include "bmi088_simple.h"
#include "control_constants.hpp"
#include "imu_attitude.hpp"
#include "imu_calibration_noise.hpp"
#include "spi.h"

namespace gimbal_imu {
namespace {
namespace c=control_constants;
State latest={}, work={};
ImuAttitude attitude;
uint32_t warmup_start=0, calibration_start=0;
float gyro_sum[3]={}, acc_sum[3]={};
ImuCalibrationNoise calibration_noise;
bool aligned=false;
float aligned_pitch_direction=c::IMU_PITCH_DIRECTION;
float yaw_origin=0.0f, pitch_origin=0.0f, yaw_reference=0.0f, pitch_reference=0.0f;
constexpr float rad_per_deg=0.0174532925199f;

void publish() {
  taskENTER_CRITICAL();
  latest=work;
  taskEXIT_CRITICAL();
}
void fail(Fault fault) {
  work.phase=Phase::Fault; work.fault=fault;
  work.ready=false; work.valid=false;
  publish();
}
void reset_calibration() {
  if (work.calibration_samples) ++work.calibration_restarts;
  work.calibration_samples=0;
  work.calibration_elapsed_ms=0;
  work.calibration_std_rad_s=0.0f;
  calibration_noise.reset();
  for (unsigned i=0;i<3;++i) gyro_sum[i]=acc_sum[i]=0.0f;
}
void transform(const float in[3], float out[3]) {
  for (unsigned i=0;i<3;++i) {
    out[i]=0.0f;
    for (unsigned j=0;j<3;++j) out[i]+=c::IMU_SENSOR_TO_GIMBAL[i][j]*in[j];
  }
}
}  // namespace

void begin() {
  static_assert(configTICK_RATE_HZ == 1000, "IMU requires 1ms ticks");
  work=State{};
  reset_calibration();  // Also clear statistics and sums if begin() is called again.
  calibration_start=0;
  aligned=false;
  publish();
  // APB2 is 84 MHz. /16 gives 5.25 MHz, below BMI088's 10 MHz SPI maximum.
  hspi1.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;
  if (HAL_SPI_Init(&hspi1)!=HAL_OK) {work.init_error=6;fail(Fault::Init);return;}
  work.init_error=bmi088_init();
  if (work.init_error) {fail(Fault::Init);return;}
  warmup_start=xTaskGetTickCount();
  work.phase=Phase::Warmup;
  publish();
}

void update() {
  if (work.phase==Phase::Fault || work.phase==Phase::Starting) return;
  float sensor_acc[3], sensor_deg_s[3], a[3], w[3];
  if (!bmi088_read(sensor_acc,sensor_deg_s)) {fail(Fault::Spi);return;}
  const uint32_t now=xTaskGetTickCount();
  const uint32_t interval=work.samples ? now-work.stamp_ms : 0;
  if (interval==0 && work.samples) return;
  work.stamp_ms=now; ++work.samples;
  if (interval>work.max_interval_ms) work.max_interval_ms=interval;
  transform(sensor_acc,a);
  transform(sensor_deg_s,w);
  bool still=true;
  work.max_raw_gyro_rad_s=0.0f;
  for (unsigned i=0;i<3;++i) {
    w[i]*=rad_per_deg;
    if (!std::isfinite(a[i]) || !std::isfinite(w[i]) ||
        std::fabs(w[i])>33.0f) {fail(Fault::Range);return;}
    if (std::fabs(w[i])>work.max_raw_gyro_rad_s) work.max_raw_gyro_rad_s=std::fabs(w[i]);
    if (std::fabs(w[i])>c::IMU_CALIBRATION_MAX_RATE_RAD_S) still=false;
  }
  const float norm=std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
  work.acceleration_norm_mps2=norm;
  if (interval>c::IMU_MAX_GAP_MS) {
    if (work.phase==Phase::Ready) {fail(Fault::Timing);return;}
    work.calibration_restart_reason=4;
    reset_calibration();
  }
  if (work.phase==Phase::Warmup) {
    if (now-warmup_start<c::IMU_WARMUP_MS) {publish();return;}
    work.phase=Phase::Calibrating;
  }
  if (work.phase==Phase::Calibrating) {
    if (!still || std::fabs(norm-9.80665f)>1.0f) {
      work.calibration_restart_reason=still ? 2 : 1;
      reset_calibration();publish();return;
    }
    if (!work.calibration_samples) {
      calibration_start=now;
    }
    if (!calibration_noise.add(w,now,c::IMU_CALIBRATION_BLOCK_MS,
                               c::IMU_CALIBRATION_MEAN_SPREAD_RAD_S)) {
      work.calibration_rejected_std_rad_s=calibration_noise.max_std_rad_s();
      work.calibration_restart_reason=5;
      reset_calibration();publish();return;
    }
    work.calibration_std_rad_s=calibration_noise.max_std_rad_s();
    for (unsigned i=0;i<3;++i) {
      gyro_sum[i]+=w[i]; acc_sum[i]+=a[i];
    }
    ++work.calibration_samples;
    work.calibration_elapsed_ms=now-calibration_start;
    if (work.calibration_samples>=c::IMU_CALIBRATION_MIN_SAMPLES &&
        work.calibration_elapsed_ms>=c::IMU_CALIBRATION_MS) {
      if (work.calibration_std_rad_s>c::IMU_CALIBRATION_STD_RAD_S) {
        work.calibration_rejected_std_rad_s=work.calibration_std_rad_s;
        work.calibration_restart_reason=3;
        reset_calibration();publish();return;
      }
      for (unsigned i=0;i<3;++i) {
        work.bias_rad_s[i]=gyro_sum[i]/work.calibration_samples;
        a[i]=acc_sum[i]/work.calibration_samples;
      }
      if (!attitude.begin(a,c::FOLLOW_BOTH_ENABLE && c::IMU_GIMBAL_AXIS_PROJECTION)) {
        fail(Fault::Range);return;
      }
      work.raw_yaw_rad=attitude.yaw; work.raw_pitch_rad=attitude.pitch;
      work.phase=Phase::Ready; work.ready=true;
    }
    publish();return;
  }
  for (unsigned i=0;i<3;++i) w[i]-=work.bias_rad_s[i];
  if (!attitude.update(a,w,interval*0.001f,c::IMU_ATTITUDE_KP)) {
    fail(Fault::Range);return;
  }
  work.raw_yaw_rad=attitude.yaw; work.raw_pitch_rad=attitude.pitch;
  work.world_yaw_rate_rad_s=attitude.world_vertical_rate(w);
  if (!std::isfinite(work.world_yaw_rate_rad_s)) {fail(Fault::Range);return;}
  publish();
}

State read() {
  State result;
  read(result);
  return result;
}
void read(State & result) {
  taskENTER_CRITICAL();
  result=latest;
  const bool referenced=aligned;
  const float yo=yaw_origin, po=pitch_origin, yr=yaw_reference, pr=pitch_reference;
  const float pitch_direction=aligned_pitch_direction;
  taskEXIT_CRITICAL();
  result.ready=result.phase==Phase::Ready &&
    xTaskGetTickCount()-result.stamp_ms<=c::IMU_MAX_GAP_MS;
  result.aligned=referenced;
  result.valid=result.ready && referenced;
  if (referenced) {
    result.yaw_rad=yr+c::IMU_YAW_DIRECTION*(result.raw_yaw_rad-yo);
    result.pitch_rad=pr+pitch_direction*(result.raw_pitch_rad-po);
  }
}
bool ready() {return read().ready;}
bool capture_reference(float yaw_rad,float pitch_rad) {
  return capture_reference(yaw_rad,pitch_rad,c::IMU_PITCH_DIRECTION);
}
bool capture_reference(float yaw_rad,float pitch_rad,float pitch_direction) {
  if ((pitch_direction!=1.0f && pitch_direction!=-1.0f) ||
      !std::isfinite(yaw_rad) || !std::isfinite(pitch_rad)) return false;
  taskENTER_CRITICAL();
  const bool ok=latest.ready && latest.phase==Phase::Ready &&
    xTaskGetTickCount()-latest.stamp_ms<=c::IMU_MAX_GAP_MS;
  if (ok) {
    yaw_origin=latest.raw_yaw_rad; pitch_origin=latest.raw_pitch_rad;
    yaw_reference=yaw_rad; pitch_reference=pitch_rad;
    aligned_pitch_direction=pitch_direction;
    aligned=true;
  }
  taskEXIT_CRITICAL();
  return ok;
}
}  // namespace gimbal_imu
