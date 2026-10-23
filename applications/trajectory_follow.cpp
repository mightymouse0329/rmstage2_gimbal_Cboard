#include "trajectory_follow.hpp"
#include "homing.hpp"
#include "pitch_start.hpp"
#include "gimbal_can.hpp"
#include "gimbal_imu.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include "FreeRTOS.h"
#include "task.h"
#include "control_constants.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "shared_data.hpp"

// 云台轨迹跟随（裸电机阶段）
//
// 控制结构（每个控制周期执行一次）：
//   参考角 + 速度前馈 ──> 位置环 P(+可选 I/D) ──> 目标转速限幅 ──> 目标转速斜坡
//        ──> 速度环 PI ──> 力矩前馈(J·α + K_g·cosθ) ──> 力矩限幅 ──> CAN 0x1FE
//
// 关键约束：电机只在收到新电流帧时更新力矩，所以 FOLLOW_CAN_TX_INTERVAL_MS
// 就是"执行器的采样周期"。闭环时间常数 τ = J/(Kt·Kp) 必须 ≥ 5~10 倍该周期，
// 否则采样环自激，表现为满量程力矩正负翻转的"嗡嗡"振动。

namespace trajectory_follow
{
namespace
{
using control_constants::FollowAxisConfig;
using control_constants::FollowPidConfig;

static_assert(configTICK_RATE_HZ == 1000, "Trajectory follow requires 1 ms RTOS ticks");
static_assert(sizeof(TickType_t) == sizeof(uint32_t), "32-bit ticks required");
static_assert(control_constants::FOLLOW_PERIOD_MS > 0, "Invalid control period");

struct PidState
{
  float integral = 0.0f;
  float previous_feedback = 0.0f;
  float filtered_derivative = 0.0f;
  bool initialized = false;
};

struct AxisState
{
  PidState angle;
  PidState speed;
  float speed_command = 0.0f;  // 斜坡后的目标转速
  float ff_direction = 0.0f;   // 低通后的速度指令，仅用于摩擦前馈取方向
  bool breakaway = false;      // 静摩擦脱困是否正在生效
};

// ---- 电机对象（CAN1，GM6020 电流模式，由 chassis_task 负责总线初始化）----
sp::RM_Motor yaw_motor(
  control_constants::FOLLOW_YAW_MOTOR_ID, sp::RM_Motors::GM6020,
  control_constants::FOLLOW_YAW_RATIO);
sp::RM_Motor pitch_motor(
  control_constants::FOLLOW_PITCH_MOTOR_ID, sp::RM_Motors::GM6020,
  control_constants::FOLLOW_PITCH_RATIO);

// ---- 共享状态 ----
Feedback feedback = {};
Result latest = {};
SequenceDiagnostics sequence_diagnostics = {};
TransportStatus transport = {};
bool enabled = control_constants::FOLLOW_START_ENABLED || control_constants::FOLLOW_SINGLE_YAW;
bool capture_requested = false;
bool initial_pose_captured = false;
bool startup_capture_allowed = control_constants::FOLLOW_AUTO_CAPTURE_INITIAL_POSE ||
  control_constants::FOLLOW_SINGLE_YAW;
float yaw_origin = 0.0f;
float pitch_origin = 0.0f;
uint32_t yaw_stamp = 0;
uint32_t pitch_stamp = 0;
float requested_yaw_torque = 0.0f;
float requested_pitch_torque = 0.0f;
uint32_t command_stamp = 0;
bool command_active = false;
uint32_t last_tx_ms = 0;
bool sent_once = false;
uint8_t fault_code = FAULT_NONE;
AxisState yaw_state;
AxisState pitch_state;

float clamp(float value, float low, float high)
{
  return value < low ? low : (value > high ? high : value);
}

bool gains_valid(const FollowPidConfig & pid)
{
  return std::isfinite(pid.kp) && pid.kp >= 0.0f && std::isfinite(pid.ki) && pid.ki >= 0.0f &&
         std::isfinite(pid.kd) && pid.kd >= 0.0f && std::isfinite(pid.integral_limit) &&
         pid.integral_limit >= 0.0f;
}

bool configured(const FollowAxisConfig & c)
{
  return c.configured && gains_valid(c.angle_pid) && gains_valid(c.speed_pid) &&
         (!control_constants::FOLLOW_ENFORCE_ANGLE_LIMITS ||
          (std::isfinite(c.angle_min_rad) && std::isfinite(c.angle_max_rad) &&
           c.angle_min_rad < c.angle_max_rad)) &&
         std::isfinite(c.max_speed_rad_s) && c.max_speed_rad_s > 0.0f &&
         std::isfinite(c.max_accel_rad_s2) && c.max_accel_rad_s2 > 0.0f &&
         std::isfinite(c.max_output) && c.max_output > 0.0f && c.max_output <= sp::GM6020_MAX_TORQUE &&
         std::isfinite(c.velocity_feedforward_gain) && c.velocity_feedforward_gain >= 0.0f &&
         std::isfinite(c.accel_feedforward_j) && c.accel_feedforward_j >= 0.0f &&
         std::isfinite(c.gravity_coefficient_nm) && c.gravity_coefficient_nm >= 0.0f &&
         (c.angle_pid.kp + c.angle_pid.ki + c.angle_pid.kd) > 0.0f &&
         (c.speed_pid.kp + c.speed_pid.ki + c.speed_pid.kd) > 0.0f;
}

bool fresh(const AxisFeedback & f, uint32_t now)
{
  return f.valid && std::isfinite(f.angle_rad) && std::isfinite(f.speed_rad_s) &&
         (now - f.stamp_ms) < control_constants::FOLLOW_FEEDBACK_TIMEOUT_MS;
}

// 默认微分先行；位置环可传入解析目标速度，形成同一滤波器下的误差微分。
// target 与 actual 用同一坐标；饱和时停止积分避免积分饱和。
float pid(
  PidState & state, const FollowPidConfig & config, float target, float actual, float dt,
  float feedforward, float low, float high, bool & saturated,
  float derivative_reference = 0.0f,
  float derivative_filter_s = control_constants::FOLLOW_D_FILTER_S)
{
  const float error = target - actual;
  const float feedback_derivative =
    state.initialized ? -(actual - state.previous_feedback) / dt : 0.0f;
  const float derivative = derivative_reference + feedback_derivative;
  const float alpha = dt / (derivative_filter_s + dt);
  state.filtered_derivative += alpha * (derivative - state.filtered_derivative);
  state.previous_feedback = actual;
  state.initialized = true;
  const float increment = config.ki * error * dt;
  const float candidate_i =
    clamp(state.integral + increment, -config.integral_limit, config.integral_limit);
  const float base = feedforward + config.kp * error + config.kd * state.filtered_derivative;
  const float candidate = base + candidate_i;
  if (!((candidate > high && increment > 0.0f) || (candidate < low && increment < 0.0f)))
    state.integral = candidate_i;
  const float raw = base + state.integral;
  const float output = clamp(raw, low, high);
  saturated = raw < low || raw > high;
  return output;
}

// 单轴：位置环 → 目标转速限幅 → 斜坡 → 速度环（含力矩前馈）→ 力矩限幅
AxisResult update_axis(
  AxisState & state, const FollowAxisConfig & c, const AxisFeedback & f, float ref_angle,
  float ref_rate, float ref_acc, float dt, bool physical_bounds = false,
  bool track_reference_derivative = false)
{
  AxisResult r = {};
  r.target_angle_rad = ref_angle;
  r.target_speed_rad_s = ref_rate;
  r.target_acceleration_rad_s2 = ref_acc;
  r.actual_angle_rad = f.angle_rad;
  r.actual_speed_rad_s = f.speed_rad_s;

  const bool bounded = physical_bounds || control_constants::FOLLOW_ENFORCE_ANGLE_LIMITS;
  const float target =
    bounded ? clamp(ref_angle, c.angle_min_rad, c.angle_max_rad) : ref_angle;
  r.limited_angle_rad = target;
  r.angle_limited = target != ref_angle;
  r.angle_error_rad = target - f.angle_rad;

  // 速度前馈：把参考角速度直接加到目标转速上，消除 P 环的固有跟踪滞后。
  float speed_ff = c.velocity_feedforward_gain * ref_rate;
  if (r.angle_limited) speed_ff = 0.0f;  // 被限位截断时不再前馈
  r.speed_feedforward_rad_s = speed_ff;

  float min_speed = -c.max_speed_rad_s;
  float max_speed = c.max_speed_rad_s;
  if (bounded && f.angle_rad <= c.angle_min_rad) min_speed = 0.0f;  // 到限位不许继续往外
  if (bounded && f.angle_rad >= c.angle_max_rad) max_speed = 0.0f;

  // 双轴测试的 yaw：Kd * LPF(目标速度 - 反馈角度差分速度)。
  // 两项共用 yaw 位置环微分滤波器，保留反馈阻尼，避免抵消速度前馈。
  // 目标角被限幅时，其目标速度为零；pitch 和原单轴分支保持微分先行。
  const float derivative_reference =
    track_reference_derivative && !r.angle_limited ? ref_rate : 0.0f;
  const float angle_derivative_filter_s = track_reference_derivative ?
    control_constants::FOLLOW_BOTH_YAW_ANGLE_D_FILTER_S : control_constants::FOLLOW_D_FILTER_S;
  r.speed_demand_rad_s =
    pid(state.angle, c.angle_pid, target, f.angle_rad, dt, speed_ff, min_speed, max_speed,
        r.speed_limited, derivative_reference, angle_derivative_filter_s);

  // 目标转速斜坡：限制加速度冲击。必须 ≥ 轨迹峰值加速度，否则会压住曲线。
  const float max_change = c.max_accel_rad_s2 * dt;
  state.speed_command +=
    clamp(r.speed_demand_rad_s - state.speed_command, -max_change, max_change);
  state.speed_command = clamp(state.speed_command, min_speed, max_speed);
  r.speed_command_rad_s = state.speed_command;
  r.speed_limited = r.speed_limited || (r.speed_demand_rad_s != state.speed_command);

  // 力矩前馈：加速度前馈 J·α（补惯性）+ 重力前馈 K_g·cosθ（补偏置质量）
  r.accel_feedforward_nm = c.accel_feedforward_j * ref_acc;
  r.gravity_feedforward_nm = c.gravity_coefficient_nm * std::cos(f.angle_rad);
  // 摩擦前馈：沿"速度指令方向"补库仑摩擦。
  // 注意这一版的取舍（实测对比过）：
  //   用 speed_command（含位置环误差修正）→ 低速段跟得更紧，但它在闭环内，
  //   小信号等效增益 = friction_nm / blend = 0.07/0.02 = 3.5 N·m/(rad/s)，
  //   而速度环 Kp 只有 0.015，相当于把内环增益放大约 230 倍 →
  //   平台段过零时力矩在 ±friction_nm 之间翻转，裸电机会震动。
  //   用 ref_rate（参考角速度）→ 不震，但平台段参考速度≈0 时前馈也归零，
  //   跟踪会松 0.5~1 deg。这里按实测结果选用 speed_command。
  if (c.friction_nm > 0.0f) {
    const float blend = control_constants::FOLLOW_FRICTION_BLEND_RAD_S;
    // 方向信号先低通：轨迹每周期有 6 个停顿反向点，速度指令会在 ±小值之间逐拍抖动，
    // 直接用它的符号会让力矩每拍翻转 ±friction_nm，裸电机就会高频震动。
    const float filter = control_constants::FOLLOW_FRICTION_FILTER_S;
    if (filter > 0.0f) {
      const float alpha = dt / (filter + dt);
      state.ff_direction += alpha * (state.speed_command - state.ff_direction);
    } else {
      state.ff_direction = state.speed_command;
    }
    r.friction_feedforward_nm =
      c.friction_nm * (state.ff_direction / (std::fabs(state.ff_direction) + blend));
  }
  // 静摩擦脱困：平台段参考速度≈0，前馈也归零，电机会被静摩擦粘住。
  // 误差超过 ON 且几乎不动 → 沿误差方向补一个摩擦力矩；误差回到 OFF 以下或已动起来 → 撤销。
  // 滞环 + 速度判据保证不会在阈值附近抖动，也不会干扰高速段。
  const float kick_on = control_constants::FOLLOW_BREAKAWAY_ON_RAD;
  if (kick_on > 0.0f) {
    const float speed_quiet = control_constants::FOLLOW_BREAKAWAY_SPEED_RAD_S;
    const float error = std::fabs(r.angle_error_rad);
    const float speed = std::fabs(f.speed_rad_s);
    if (!state.breakaway && error > kick_on && speed < speed_quiet)
      state.breakaway = true;
    else if (state.breakaway &&
             (error < control_constants::FOLLOW_BREAKAWAY_OFF_RAD || speed > 2.0f * speed_quiet))
      state.breakaway = false;
    if (state.breakaway)
      r.breakaway_feedforward_nm = r.angle_error_rad > 0.0f ? c.friction_nm : -c.friction_nm;
  }

  float min_out = -c.max_output;
  float max_out = c.max_output;
  if (bounded && f.angle_rad <= c.angle_min_rad) min_out = 0.0f;
  if (bounded && f.angle_rad >= c.angle_max_rad) max_out = 0.0f;
  r.control_output = pid(
    state.speed, c.speed_pid, state.speed_command, f.speed_rad_s, dt,
    r.accel_feedforward_nm + r.gravity_feedforward_nm + r.friction_feedforward_nm +
      r.breakaway_feedforward_nm,
    min_out, max_out, r.output_limited);
  return r;
}

bool finite_output(const AxisResult & r)
{
  return std::isfinite(r.control_output) && std::isfinite(r.speed_command_rad_s) &&
         std::isfinite(r.angle_error_rad);
}

void reset_axis(AxisState & state)
{
  state = AxisState{};
}

void latch_fault(uint8_t code)
{
  taskENTER_CRITICAL();
  enabled = false;
  command_active = false;
  requested_yaw_torque = 0.0f;
  requested_pitch_torque = 0.0f;
  fault_code = code;
  taskEXIT_CRITICAL();
}

void step(
  const TrajectoryPoint & target, const Feedback & f, bool run, uint32_t now, float dt,
  uint32_t phase_ms, bool rc_stop)
{
  Result result = {};
  result.stamp_ms = now;
  result.calibrated = initial_pose_captured;
  result.rc_stop = rc_stop;
  result.trajectory_elapsed_ms = phase_ms;
  result.fault = fault_code;
  result.configuration_ready = configured(control_constants::FOLLOW_YAW) &&
    (control_constants::FOLLOW_SINGLE_YAW || configured(control_constants::FOLLOW_PITCH));
  result.feedback_fresh = fresh(f.yaw, now) &&
    (control_constants::FOLLOW_SINGLE_YAW || fresh(f.pitch, now));
  const bool target_valid = std::isfinite(target.yaw) && std::isfinite(target.yaw_vel) &&
                            std::isfinite(target.yaw_acc) && std::isfinite(target.pitch) &&
                            std::isfinite(target.pitch_vel) && std::isfinite(target.pitch_acc);

  // 停机时也把参考/反馈填进去，方便在曲线上对照"是没进闭环还是跟不好"。
  result.yaw.target_angle_rad = target.yaw;
  result.yaw.target_speed_rad_s = target.yaw_vel;
  result.yaw.target_acceleration_rad_s2 = target.yaw_acc;
  result.yaw.actual_angle_rad = f.yaw.angle_rad;
  result.yaw.actual_speed_rad_s = f.yaw.speed_rad_s;
  result.yaw.angle_error_rad = target.yaw - f.yaw.angle_rad;
  result.pitch.target_angle_rad = target.pitch;
  result.pitch.target_speed_rad_s = target.pitch_vel;
  result.pitch.target_acceleration_rad_s2 = target.pitch_acc;
  result.pitch.actual_angle_rad = f.pitch.angle_rad;
  result.pitch.actual_speed_rad_s = f.pitch.speed_rad_s;
  result.pitch.angle_error_rad = target.pitch - f.pitch.angle_rad;

  if (run && result.configuration_ready && result.feedback_fresh && target_valid &&
      std::isfinite(dt) && dt > 0.0f &&
      dt <= control_constants::FOLLOW_MAX_STEP_MS * 0.001f) {
    if (control_constants::FOLLOW_CONSTANT_TORQUE_TEST) {
      // 恒力矩台架测试：不走任何闭环，只给固定力矩（用于测静摩擦和确认转向）
      reset_axis(yaw_state);
      reset_axis(pitch_state);
      result.yaw.control_output =
        clamp(control_constants::FOLLOW_TEST_TORQUE_NM, -control_constants::FOLLOW_YAW.max_output,
              control_constants::FOLLOW_YAW.max_output);
      result.yaw.speed_command_rad_s = 0.0f;
      result.active = true;
    } else {
      result.yaw =
        update_axis(yaw_state, control_constants::FOLLOW_YAW, f.yaw, target.yaw, target.yaw_vel,
                    target.yaw_acc, dt);
      if (!control_constants::FOLLOW_SINGLE_YAW)
        result.pitch = update_axis(
          pitch_state, control_constants::FOLLOW_PITCH, f.pitch, target.pitch, target.pitch_vel,
          target.pitch_acc, dt);
      result.active = finite_output(result.yaw) &&
        (control_constants::FOLLOW_SINGLE_YAW || finite_output(result.pitch));
    }
  }
  if (!result.active) {
    reset_axis(yaw_state);
    reset_axis(pitch_state);
    result.yaw.control_output = 0.0f;
    result.pitch.control_output = 0.0f;
    result.yaw.speed_command_rad_s = 0.0f;
    result.pitch.speed_command_rad_s = 0.0f;
  }
  taskENTER_CRITICAL();
  latest = result;
  taskEXIT_CRITICAL();
}
}  // namespace

void set_feedback(const Feedback & value)
{
  taskENTER_CRITICAL();
  feedback = value;
  taskEXIT_CRITICAL();
}

Result read()
{
  taskENTER_CRITICAL();
  const Result value = latest;
  taskEXIT_CRITICAL();
  return value;
}
void read_sequence_diagnostics(SequenceDiagnostics & out)
{
  taskENTER_CRITICAL();
  out = sequence_diagnostics;
  taskEXIT_CRITICAL();
}

void set_enabled(bool value)
{
  taskENTER_CRITICAL();
  enabled = value;
  if (value) fault_code = FAULT_NONE;
  if (!value) {
    command_active = false;
    requested_yaw_torque = 0.0f;
    requested_pitch_torque = 0.0f;
  }
  taskEXIT_CRITICAL();
}

bool request_initial_pose_capture()
{
  taskENTER_CRITICAL();
  const bool accepted = !enabled;
  if (accepted) capture_requested = true;
  taskEXIT_CRITICAL();
  return accepted;
}

bool receive_can1(const CAN_RxHeaderTypeDef & header, uint8_t * bytes, uint32_t stamp_ms)
{
  if (!bytes || header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8)
    return false;
  if (header.StdId == yaw_motor.rx_id) {
    yaw_motor.read(bytes, stamp_ms);
    yaw_stamp = stamp_ms;
    ++transport.yaw_frames;
    return true;
  }
  if (header.StdId == pitch_motor.rx_id) {
    pitch_motor.read(bytes, stamp_ms);
    pitch_stamp = stamp_ms;
    ++transport.pitch_frames;
    return true;
  }
  return false;
}

TransportStatus read_transport()
{
  taskENTER_CRITICAL();
  const TransportStatus value = transport;
  taskEXIT_CRITICAL();
  return value;
}

void transmit_can1(uint32_t now_ms, bool permitted)
{
  if (::homing::selected() || ::pitch_start::selected()) return;  // Exclusive test owns the motors.

  // 电流帧周期 = 执行器采样周期，见文件头注释。这里是"必要不充分"的频率控制：
  // 只有真正发出新帧，电机才会更新力矩。
  if (sent_once && (now_ms - last_tx_ms) < control_constants::FOLLOW_CAN_TX_INTERVAL_MS) return;
  float yaw_torque;
  float pitch_torque;
  taskENTER_CRITICAL();
  const bool live = permitted && command_active &&
    (now_ms - command_stamp) < control_constants::FOLLOW_COMMAND_TIMEOUT_MS;
  yaw_torque = live ? requested_yaw_torque : 0.0f;
  pitch_torque = live ? requested_pitch_torque : 0.0f;
  taskEXIT_CRITICAL();
  uint8_t bytes[8] = {};  // 0x1FE 的 4 个 16 位槽位：yaw 占 1-2、pitch 占 3-4，其余补 0
  yaw_motor.cmd(yaw_torque);
  pitch_motor.cmd(pitch_torque);
  yaw_motor.write(bytes);
  pitch_motor.write(bytes);
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
    ++transport.tx_busy;
    return;  // 邮箱满就下次再发，绝不阻塞底盘控制
  }
  CAN_TxHeaderTypeDef header = {};
  header.StdId = yaw_motor.tx_id;  // 两台电机同在 0x1FE 电流帧
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  uint32_t mailbox = 0;
  if (HAL_CAN_AddTxMessage(&hcan1, &header, bytes, &mailbox) == HAL_OK) {
    last_tx_ms = now_ms;
    sent_once = true;
    ++transport.tx_frames;
  } else {
    ++transport.tx_errors;
  }
}

constexpr TrajectoryPoint stretched_reference(TrajectoryPoint point, float pitch_gain, uint32_t stretch)
{
  const float time2 = static_cast<float>(stretch) * stretch;
  return {point.yaw, point.yaw_vel / stretch, point.yaw_acc / time2,
          point.pitch * pitch_gain, point.pitch_vel * pitch_gain / stretch,
          point.pitch_acc * pitch_gain / time2};
}

constexpr TrajectoryPoint prelift_reference(TrajectoryPoint point, float u, float seconds, float travel)
{
  const float u2 = u*u, u3 = u2*u, u4 = u3*u, u5 = u4*u;
  point.pitch = point.pitch - travel + travel * (10.0f*u3 - 15.0f*u4 + 6.0f*u5);
  point.pitch_vel = travel * 30.0f*u2*(1.0f-u)*(1.0f-u) / seconds;
  point.pitch_acc = travel * (60.0f*u - 180.0f*u2 + 120.0f*u3) / (seconds*seconds);
  return point;
}

TrajectoryPoint sample_reference(uint32_t elapsed_ms)
{
  constexpr uint32_t stretch = control_constants::FOLLOW_TRAJECTORY_TIME_SCALE;
  TrajectoryPoint point = trajectory_sample_at_ms((elapsed_ms / stretch) % TRAJECTORY_PERIOD_MS);
  // Undo configured amplitude scaling HERE, never edit the reference data/generator.
  const float pitch_gain = control_constants::FOLLOW_PITCH_SEQUENCE_ENABLED ?
    60.0f / TRAJECTORY_PITCH_RANGE_DEG : 1.0f;
  return stretched_reference(point, pitch_gain, stretch);
}

struct PitchBoundaryResult {
  float torque;
  bool lower_braking;
  bool upper_braking;
};

// Measured speed determines braking, not the planned direction. The latency
// term includes IMU filtering/CAN response. This is a software braking envelope,
// not a claim that unknown mechanics can always stop at the assumed deceleration.
constexpr PitchBoundaryResult pitch_boundary_torque(
  float angle, float speed, float torque, float lower, float upper)
{
  namespace c = control_constants;
  const float magnitude = speed < 0.0f ? -speed : speed;
  const float stopping = magnitude * magnitude / (2.0f * c::FOLLOW_PITCH_BRAKE_DECEL_RAD_S2) +
    magnitude * c::FOLLOW_PITCH_BRAKE_LATENCY_S;
  const bool down = speed < 0.0f && angle - lower <= stopping;
  const bool up = speed > 0.0f && upper - angle <= stopping;
  const float raw_brake = c::FOLLOW_PITCH_BRAKE_GAIN_NM_PER_RAD_S * magnitude;
  const float brake = raw_brake < c::FOLLOW_PITCH_MAX_TORQUE_NM ?
    raw_brake : c::FOLLOW_PITCH_MAX_TORQUE_NM;
  if (down && torque < brake) torque = brake;
  if (up && torque > -brake) torque = -brake;
  if (angle <= lower && torque < 0.0f) torque = 0.0f;
  if (angle >= upper && torque > 0.0f) torque = 0.0f;
  return {torque, down, up};
}

constexpr bool pitch_support_fault(uint8_t fault)
{
  return fault == FAULT_TRACKING_ERROR || fault == FAULT_OVERSPEED ||
    fault == FAULT_PRELIFT || fault == FAULT_BOUNDARY || fault == FAULT_YAW_REFERENCE;
}

constexpr float dual_yaw_authority(uint32_t elapsed_ms) {
  return elapsed_ms >= control_constants::FOLLOW_BOTH_YAW_STARTUP_MS ? 1.0f :
    static_cast<float>(elapsed_ms) / control_constants::FOLLOW_BOTH_YAW_STARTUP_MS;
}
constexpr bool dual_yaw_step_valid(float delta, float dt) {
  namespace c = control_constants;
  const float magnitude = delta < 0.0f ? -delta : delta;
  return dt > 0.0f && magnitude <=
    c::FOLLOW_YAW.max_speed_rad_s * c::FOLLOW_OVERSPEED_FACTOR * dt + c::FOLLOW_BOTH_YAW_JUMP_SLACK_RAD;
}
constexpr bool dual_yaw_wrong_way(float plan_delta, float actual_delta, float speed) {
  namespace c = control_constants;
  return (plan_delta > c::FOLLOW_BOTH_YAW_WRONG_WAY_RAD &&
      actual_delta < -c::FOLLOW_BOTH_YAW_WRONG_WAY_RAD && speed < -c::FOLLOW_BOTH_YAW_WRONG_WAY_RAD_S) ||
    (plan_delta < -c::FOLLOW_BOTH_YAW_WRONG_WAY_RAD &&
      actual_delta > c::FOLLOW_BOTH_YAW_WRONG_WAY_RAD && speed > c::FOLLOW_BOTH_YAW_WRONG_WAY_RAD_S);
}

// Optional one/two-axis sequence uses the existing follow task and PID structure. The tested
// pitch_start owns the shared CAN sender and preserves M2006 position holding.
void run_pitch_sequence()
{
  namespace c = control_constants;
  constexpr bool both = c::FOLLOW_BOTH_ENABLE;
  static Result result;  // Keep the existing 256-word task stack; no large reset temporary.
  static pitch_start::Status prepared;
  static gimbal_imu::State imu;
  static SequenceDiagnostics trace = {};
  static pitch_start::CanDiagnostics fault_can = {};
  struct YawMotorSample {float angle, speed, torque;};
  static YawMotorSample motor_sample = {};
  float yaw_motor_start = 0.0f;
  static FollowAxisConfig pitch_config = c::FOLLOW_PITCH;
  static FollowAxisConfig hold_config = c::FOLLOW_PITCH;
  static FollowAxisConfig yaw_config = c::FOLLOW_YAW;
  if (both) {
    // IMU feedback has a different response from the legacy motor-RPM loop.
    // Reduce only this inner loop; preserve position gains, limits and timing.
    yaw_config.speed_pid = {c::FOLLOW_BOTH_YAW_SPEED_KP, c::FOLLOW_BOTH_YAW_SPEED_KI,
      c::FOLLOW_BOTH_YAW_SPEED_KD, c::FOLLOW_YAW_SPEED_INTEGRAL_LIMIT};
  }
  const TrajectoryPoint initial = sample_reference(0);
  const float physical_lower = initial.pitch - c::FOLLOW_PITCH_PRELIFT_RAD +
    c::FOLLOW_PITCH_LOWER_MARGIN_RAD;
  const float tracking_lower = initial.pitch - c::FOLLOW_PITCH_TRACK_FLOOR_TOL_RAD;
  pitch_config.angle_min_rad = initial.pitch - c::FOLLOW_PITCH_PRELIFT_RAD +
    c::FOLLOW_PITCH_LOWER_MARGIN_RAD;
  pitch_config.angle_max_rad = initial.pitch - c::FOLLOW_PITCH_PRELIFT_RAD +
    c::FOLLOW_PITCH_USABLE_TRAVEL_RAD - c::FOLLOW_PITCH_UPPER_MARGIN_RAD;
  hold_config = pitch_config;
  hold_config.angle_pid = {c::FOLLOW_PITCH_FAULT_HOLD_KP, 0.0f, 0.0f, 0.0f};
  hold_config.max_speed_rad_s = c::FOLLOW_PITCH_FAULT_HOLD_MAX_SPEED_RAD_S;
  hold_config.max_accel_rad_s2 = c::FOLLOW_PITCH_PRELIFT_MAX_ACCEL_RAD_S2;
  hold_config.accel_feedforward_j = 0.0f;
  hold_config.friction_nm = 0.0f;
  const bool config_ok = configured(pitch_config) &&
    (!both || configured(yaw_config)) &&
    pitch_config.angle_min_rad < initial.pitch &&
    pitch_config.angle_max_rad > -initial.pitch;
  PitchSequencePhase phase = PitchSequencePhase::Waiting;
  uint8_t failure = config_ok ? FAULT_NONE : FAULT_CONFIG;
  FaultCause cause = config_ok ? FaultCause::None : FaultCause::InvalidOutput;
  uint32_t prelift_start = 0, trajectory_start = 0, settle_since = 0, error_ms = 0;
  uint32_t yaw_error_ms = 0;
  uint32_t yaw_wrong_way_ms = 0;
  uint32_t speed_stamp = 0;
  float previous_raw_pitch = 0.0f, filtered_raw_speed = 0.0f, up_sign = 1.0f;
  float previous_raw_yaw = 0.0f, filtered_raw_yaw_speed = 0.0f;
  bool have_speed = false;
  bool reference_started = false, fault_hold_captured = false;
  float fault_hold_target = 0.0f;
  bool yaw_feedback_fault = false;
  TickType_t wake = xTaskGetTickCount(), previous = wake;
  constexpr uint32_t period = TRAJECTORY_PERIOD_MS * c::FOLLOW_TRAJECTORY_TIME_SCALE;
  const float missing = std::numeric_limits<float>::quiet_NaN();

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(c::FOLLOW_PERIOD_MS));
    const uint32_t now = xTaskGetTickCount();
    const uint32_t elapsed = now - previous;
    if (!elapsed) continue;
    previous = now;
    const float dt = elapsed * 0.001f;
    pitch_start::snapshot(prepared);
    gimbal_imu::read(imu);
    if (phase == PitchSequencePhase::Tracking && elapsed > trace.max_step_ms)
      trace.max_step_ms = elapsed;
    taskENTER_CRITICAL();
    const uint32_t feedback_now = xTaskGetTickCount();
    motor_sample = {yaw_motor.angle, yaw_motor.speed, yaw_motor.torque};
    const bool yaw_live = !both || (yaw_motor.is_alive(feedback_now) &&
      feedback_now - yaw_stamp < c::FOLLOW_FEEDBACK_TIMEOUT_MS);
    taskEXIT_CRITICAL();
    std::memset(&result, 0, sizeof(result));
    result.pitch_only = !both;
    result.prepared_sequence = true;
    result.stamp_ms = now;
    result.yaw.target_angle_rad = initial.yaw;
    result.pitch.target_angle_rad = initial.pitch;
    if (reference_started) {
      result.trajectory_elapsed_ms = (now - trajectory_start) % period;
      const TrajectoryPoint planned = sample_reference(result.trajectory_elapsed_ms);
      result.pitch.target_angle_rad = planned.pitch;
      result.yaw.target_angle_rad = planned.yaw;
    }
    result.pitch.actual_angle_rad = imu.valid ? imu.pitch_rad : missing;
    result.yaw.actual_angle_rad = imu.valid ? imu.yaw_rad : missing;
    result.configuration_ready = config_ok;
    result.calibrated = imu.valid && phase != PitchSequencePhase::Waiting;
    const bool pitch_feedback_fresh = imu.ready && prepared.feedback_fresh &&
      feedback_now - prepared.stamp_ms < c::HOME_COMMAND_TIMEOUT_MS;
    result.feedback_fresh = pitch_feedback_fresh && (!both || !reference_started || yaw_live);
    const bool rc_stop = c::FOLLOW_USE_RC_STOP && g_data.sw_r == 0;
    result.rc_stop = rc_stop;

    if (!config_ok) phase = PitchSequencePhase::Fault;
    if (prepared.stage == pitch_start::Stage::Fault) {
      yaw_feedback_fault = false;
      failure = FAULT_FEEDBACK;
      cause = FaultCause::Feedback;
      phase = PitchSequencePhase::Fault;
    }
    if (phase == PitchSequencePhase::Waiting && prepared.stage == pitch_start::Stage::Done &&
        prepared.homed && prepared.m2006_locked && result.feedback_fresh) {
      up_sign = prepared.pitch_up_imu_direction;
      if ((up_sign != 1.0f && up_sign != -1.0f) ||
          !gimbal_imu::capture_reference(initial.yaw,
            initial.pitch - c::FOLLOW_PITCH_PRELIFT_RAD, up_sign)) {
        failure = FAULT_IMU;
        cause = FaultCause::Imu;
        phase = PitchSequencePhase::Fault;
      } else {
        gimbal_imu::read(imu);
        previous_raw_pitch = imu.raw_pitch_rad;
        previous_raw_yaw = imu.raw_yaw_rad;
        speed_stamp = imu.stamp_ms;
        filtered_raw_speed = 0.0f;
        filtered_raw_yaw_speed = 0.0f;
        have_speed = true;
        prelift_start = now;
        phase = PitchSequencePhase::Prelift;
        reset_axis(pitch_state);
        set_enabled(true);
      }
    }

    if (phase == PitchSequencePhase::Prelift || phase == PitchSequencePhase::Tracking) {
      if (!imu.valid || !std::isfinite(imu.pitch_rad) || !std::isfinite(imu.raw_pitch_rad) ||
          (both && (!std::isfinite(imu.yaw_rad) || !std::isfinite(imu.raw_yaw_rad)))) {
        failure = FAULT_IMU;
        cause = FaultCause::Imu;
        phase = PitchSequencePhase::Fault;
      } else if (!result.feedback_fresh || !prepared.homed || !prepared.m2006_locked) {
        yaw_feedback_fault = both && !yaw_live && pitch_feedback_fresh;
        failure = FAULT_FEEDBACK;
        cause = FaultCause::Feedback;
        phase = PitchSequencePhase::Fault;
      } else if (elapsed > c::FOLLOW_MAX_STEP_MS) {
        failure = FAULT_TIMING;
        cause = FaultCause::Timing;
        phase = PitchSequencePhase::Fault;
      } else if (!enabled || rc_stop) {
        failure = FAULT_NONE;
        cause = FaultCause::Stop;
        phase = PitchSequencePhase::Fault;  // Latched stop; no automatic restart.
      }
    }

    // Speed is also from the IMU, never the GM6020 encoder/RPM feedback. Update
    // only at a NEW sensor timestamp, using that sample's actual interval.
    if (imu.ready && (!have_speed || imu.stamp_ms != speed_stamp)) {
      const uint32_t sensor_ms = imu.stamp_ms - speed_stamp;
      if (have_speed && sensor_ms > 0U && sensor_ms <= c::IMU_MAX_GAP_MS) {
        const float sensor_dt = sensor_ms * 0.001f;
        const float measured = (imu.raw_pitch_rad - previous_raw_pitch) / sensor_dt;
        filtered_raw_speed += sensor_dt / (c::FOLLOW_IMU_SPEED_FILTER_S + sensor_dt) *
          (measured - filtered_raw_speed);
        if (both) {
          const float yaw_delta = imu.raw_yaw_rad - previous_raw_yaw;
          if (!std::isfinite(imu.world_yaw_rate_rad_s) || !dual_yaw_step_valid(yaw_delta,sensor_dt)) {
            if (phase == PitchSequencePhase::Prelift || phase == PitchSequencePhase::Tracking) {
              failure = FAULT_YAW_REFERENCE;
              cause = FaultCause::YawJump;
              phase = PitchSequencePhase::Fault;
            }
            filtered_raw_yaw_speed = 0.0f;
          } else {
            // Gyro projected into world vertical: no derivative kick from an
            // angle-estimator correction and no body-Z/pitch cross coupling.
            filtered_raw_yaw_speed += sensor_dt / (c::FOLLOW_IMU_SPEED_FILTER_S + sensor_dt) *
              (imu.world_yaw_rate_rad_s - filtered_raw_yaw_speed);
          }
        }
      } else {
        filtered_raw_speed = 0.0f;
        filtered_raw_yaw_speed = 0.0f;
      }
      previous_raw_pitch = imu.raw_pitch_rad;
      previous_raw_yaw = imu.raw_yaw_rad;
      speed_stamp = imu.stamp_ms;
      have_speed = true;
    }
    const float speed = up_sign * filtered_raw_speed;
    float yaw_speed = c::IMU_YAW_DIRECTION * filtered_raw_yaw_speed;
    const float speed_limit = phase == PitchSequencePhase::Tracking ?
      c::FOLLOW_PITCH.max_speed_rad_s : c::FOLLOW_PITCH_PRELIFT_MAX_SPEED_RAD_S;
    if ((phase == PitchSequencePhase::Prelift || phase == PitchSequencePhase::Tracking) &&
        std::fabs(speed) > speed_limit * c::FOLLOW_OVERSPEED_FACTOR) {
      failure = FAULT_OVERSPEED;
      cause = FaultCause::PitchOverspeed;
      phase = PitchSequencePhase::Fault;
    }
    if (both && phase == PitchSequencePhase::Tracking &&
        std::fabs(yaw_speed) > c::FOLLOW_YAW.max_speed_rad_s * c::FOLLOW_OVERSPEED_FACTOR) {
      failure = FAULT_OVERSPEED;
      cause = FaultCause::YawOverspeed;
      phase = PitchSequencePhase::Fault;
    }

    if (phase == PitchSequencePhase::Prelift || phase == PitchSequencePhase::Tracking) {
      TrajectoryPoint target = initial;
      if (phase == PitchSequencePhase::Prelift) {
        const float seconds = c::FOLLOW_PITCH_PRELIFT_TIME_MS * 0.001f;
        const float u = clamp((now - prelift_start) * 0.001f / seconds, 0.0f, 1.0f);
        target = prelift_reference(initial, u, seconds, c::FOLLOW_PITCH_PRELIFT_RAD);
        const bool settled = u >= 1.0f &&
          std::fabs(imu.pitch_rad - initial.pitch) <= c::FOLLOW_PITCH_PRELIFT_TOL_RAD &&
          std::fabs(speed) <= c::FOLLOW_PITCH_PRELIFT_QUIET_RAD_S &&
          (!both || (yaw_live && std::fabs(yaw_speed) <= c::FOLLOW_BOTH_YAW_START_QUIET_RAD_S));
        if (!settled) settle_since = 0;
        else if (!settle_since) settle_since = now;
        if (settle_since && now - settle_since >= c::FOLLOW_PITCH_PRELIFT_SETTLE_MS) {
          // Reference the actual settled start to raw trajectory point zero.
          // Preserve the velocity-loop load integral through this transition.
          if (!gimbal_imu::capture_reference(initial.yaw, initial.pitch, up_sign)) {
            failure = FAULT_IMU;
            cause = FaultCause::Imu;
            phase = PitchSequencePhase::Fault;
          } else {
            gimbal_imu::read(imu);
            pitch_state.angle = PidState{};
            pitch_state.speed.previous_feedback = speed;
            pitch_state.speed.filtered_derivative = 0.0f;
            pitch_state.speed.initialized = true;
            pitch_state.speed_command = 0.0f;
            pitch_state.ff_direction = 0.0f;
            if (both) {
              speed_stamp = imu.stamp_ms;
              previous_raw_pitch = imu.raw_pitch_rad;
              previous_raw_yaw = imu.raw_yaw_rad;
              filtered_raw_yaw_speed = yaw_speed = 0.0f;
              yaw_motor_start = motor_sample.angle;
              reset_axis(yaw_state);
              yaw_state.angle.previous_feedback = imu.yaw_rad;
              yaw_state.angle.initialized = true;
              yaw_state.speed.previous_feedback = 0.0f;
              yaw_state.speed.initialized = true;
            }
            trajectory_start = now;
            reference_started = true;
            error_ms = 0;
            yaw_error_ms = 0;
            yaw_wrong_way_ms = 0;
            trace.max_step_ms = elapsed;
            phase = PitchSequencePhase::Tracking;
            target = initial;
          }
        } else if (now - prelift_start >= c::FOLLOW_PITCH_PRELIFT_TIMEOUT_MS) {
          yaw_feedback_fault = both && !yaw_live;
          failure = yaw_feedback_fault ? FAULT_FEEDBACK : FAULT_PRELIFT;
          cause = yaw_feedback_fault ? FaultCause::Feedback : FaultCause::Prelift;
          phase = PitchSequencePhase::Fault;
        }
      }
      if (phase == PitchSequencePhase::Tracking) {
        result.trajectory_elapsed_ms = (now - trajectory_start) % period;
        target = sample_reference(result.trajectory_elapsed_ms);
      }
      if (phase != PitchSequencePhase::Fault) {
        // After prelift, preserve clearance below the trajectory minimum rather
        // than using a soft bound only one degree above the physical stop.
        pitch_config.angle_min_rad = phase == PitchSequencePhase::Tracking ?
          tracking_lower : physical_lower;
        pitch_config.max_speed_rad_s = phase == PitchSequencePhase::Tracking ?
          c::FOLLOW_PITCH.max_speed_rad_s : c::FOLLOW_PITCH_PRELIFT_MAX_SPEED_RAD_S;
        pitch_config.max_accel_rad_s2 = phase == PitchSequencePhase::Tracking ?
          c::FOLLOW_PITCH.max_accel_rad_s2 : c::FOLLOW_PITCH_PRELIFT_MAX_ACCEL_RAD_S2;
        AxisFeedback actual = {imu.pitch_rad, speed, imu.stamp_ms, true};
        result.pitch = update_axis(pitch_state, pitch_config, actual,
          target.pitch, target.pitch_vel, target.pitch_acc, dt, true);
        if (both && phase == PitchSequencePhase::Tracking) {
          actual = {imu.yaw_rad, yaw_speed, imu.stamp_ms, yaw_live};
          // Limiting inside the PID preserves anti-windup while the original
          // full yaw torque authority ramps in smoothly.
          yaw_config.max_output = c::FOLLOW_YAW.max_output * dual_yaw_authority(now - trajectory_start);
          result.yaw = update_axis(yaw_state, yaw_config, actual,
            target.yaw, target.yaw_vel, target.yaw_acc, dt, false, true);
        }
        const auto boundary = pitch_boundary_torque(imu.pitch_rad, speed,
          result.pitch.control_output, pitch_config.angle_min_rad, pitch_config.angle_max_rad);
        result.pitch.control_output = boundary.torque;
        result.pitch_boundary_braking = boundary.lower_braking || boundary.upper_braking;
        if (boundary.lower_braking && pitch_state.speed.integral < 0.0f)
          pitch_state.speed.integral = 0.0f;
        if (boundary.upper_braking && pitch_state.speed.integral > 0.0f)
          pitch_state.speed.integral = 0.0f;
        result.calibrated = true;
        result.active = finite_output(result.pitch) &&
          (!both || phase != PitchSequencePhase::Tracking || finite_output(result.yaw));
        if (phase == PitchSequencePhase::Tracking) {
          error_ms = std::fabs(result.pitch.angle_error_rad) > c::FOLLOW_MAX_TRACKING_ERROR_RAD ?
            error_ms + elapsed : 0;
          yaw_error_ms = both && std::fabs(result.yaw.angle_error_rad) > c::FOLLOW_MAX_TRACKING_ERROR_RAD ?
            yaw_error_ms + elapsed : 0;
          if (both && now - trajectory_start < c::FOLLOW_BOTH_YAW_DIRECTION_GUARD_MS &&
              dual_yaw_wrong_way(target.yaw - initial.yaw, imu.yaw_rad - initial.yaw, yaw_speed))
            yaw_wrong_way_ms += elapsed;
          else yaw_wrong_way_ms = 0;
          if (error_ms >= c::FOLLOW_TRACKING_ERROR_MS || yaw_error_ms >= c::FOLLOW_TRACKING_ERROR_MS) {
            failure = FAULT_TRACKING_ERROR;
            cause = yaw_error_ms >= c::FOLLOW_TRACKING_ERROR_MS ?
              FaultCause::YawTracking : FaultCause::PitchTracking;
            phase = PitchSequencePhase::Fault;
          }
          if (imu.pitch_rad < tracking_lower || imu.pitch_rad > pitch_config.angle_max_rad) {
            failure = FAULT_BOUNDARY;
            cause = FaultCause::Boundary;
            phase = PitchSequencePhase::Fault;
          }
          if (yaw_wrong_way_ms >= c::FOLLOW_BOTH_YAW_WRONG_WAY_MS) {
            failure = FAULT_YAW_REFERENCE;
            cause = FaultCause::YawWrongWay;
            phase = PitchSequencePhase::Fault;
          }
        }
        if (!result.active) {
          failure = FAULT_CONFIG;
          cause = FaultCause::InvalidOutput;
          phase = PitchSequencePhase::Fault;
        } else if (phase != PitchSequencePhase::Fault &&
                   !pitch_start::submit_follow_torques(
                     both && phase == PitchSequencePhase::Tracking ?
                       result.yaw.control_output * c::FOLLOW_YAW_DIRECTION : 0.0f,
                     result.pitch.control_output * c::HOME_PITCH_DIRECTION, now)) {
          failure = FAULT_FEEDBACK;
          cause = FaultCause::Feedback;
          phase = PitchSequencePhase::Fault;
        }
      }
    }
    if (!trace.latched) {
      trace.stamp_ms = now;
      trace.fault = failure;
      trace.cause = cause;
      trace.yaw_error_rad = result.yaw.target_angle_rad - imu.yaw_rad;
      trace.yaw_speed_rad_s = c::IMU_YAW_DIRECTION * imu.world_yaw_rate_rad_s;
      trace.yaw_torque_nm = result.yaw.control_output * c::FOLLOW_YAW_DIRECTION;
      trace.yaw_motor_travel_rad = reference_started ? motor_sample.angle - yaw_motor_start : 0.0f;
      trace.yaw_motor_speed_rad_s = motor_sample.speed;
      trace.yaw_feedback_torque_nm = motor_sample.torque;
      trace.yaw_imu_travel_rad = reference_started ? imu.yaw_rad - initial.yaw : 0.0f;
      if (phase == PitchSequencePhase::Fault) {
        // Freeze BEFORE the fault handler zeros yaw; the 50Hz plot otherwise
        // misses the offending 1ms event and shows only a released motor.
        pitch_start::read_can_diagnostics(fault_can);
        trace.yaw_torque_nm = fault_can.last_yaw_torque_nm;
        trace.latched = true;
      }
    }
    trace.phase = static_cast<uint8_t>(phase);
    if (phase == PitchSequencePhase::Fault) {
      result.active = false;
      result.yaw.control_output = 0.0f;
      if (both) reset_axis(yaw_state);
      result.pitch.control_output = 0.0f;
      // A following error is not an instruction to let a gravity-loaded joint
      // fall. With trustworthy IMU/motor feedback, stop the plan and hold a
      // bounded position. Sensor/bus/timing faults and explicit stop still zero.
      const bool can_hold = (pitch_support_fault(failure) ||
          (yaw_feedback_fault && failure == FAULT_FEEDBACK)) && have_speed && imu.valid &&
        std::isfinite(imu.pitch_rad) && std::isfinite(speed) && pitch_feedback_fresh &&
        prepared.homed && prepared.m2006_locked && enabled && !rc_stop &&
        elapsed <= c::FOLLOW_MAX_STEP_MS;
      if (can_hold) {
        hold_config.angle_min_rad = reference_started ? tracking_lower : physical_lower;
        if (!fault_hold_captured) {
          fault_hold_target = clamp(imu.pitch_rad, hold_config.angle_min_rad, hold_config.angle_max_rad);
          pitch_state.angle = PidState{};
          pitch_state.speed_command = 0.0f;
          pitch_state.ff_direction = 0.0f;
          pitch_state.breakaway = false;
          // Retain learned upward support, discard an accumulated downward drive.
          if (pitch_state.speed.integral < 0.0f) pitch_state.speed.integral = 0.0f;
          fault_hold_captured = true;
        }
        const AxisFeedback actual = {imu.pitch_rad, speed, imu.stamp_ms, true};
        const float planned = result.pitch.target_angle_rad;
        result.pitch = update_axis(pitch_state, hold_config, actual,
          fault_hold_target, 0.0f, 0.0f, dt, true);
        const auto boundary = pitch_boundary_torque(imu.pitch_rad, speed,
          result.pitch.control_output, hold_config.angle_min_rad, hold_config.angle_max_rad);
        result.pitch.control_output = boundary.torque;
        result.pitch.target_angle_rad = planned;  // Display plan; limited_angle identifies hold target.
        result.pitch_boundary_braking = boundary.lower_braking || boundary.upper_braking;
        result.pitch_fault_holding = finite_output(result.pitch) &&
          pitch_start::submit_pitch_follow_torque(result.pitch.control_output * c::HOME_PITCH_DIRECTION, now);
      }
      if (!result.pitch_fault_holding) {
        if (fault_hold_captured) failure = FAULT_FEEDBACK;  // Never resume holding after validity is lost.
        result.pitch.control_output = 0.0f;
        reset_axis(pitch_state);
        (void)pitch_start::submit_pitch_follow_torque(0.0f, now);
      }
    }
    result.pitch_reference_started = reference_started;
    result.fault = failure;
    result.pitch_sequence_phase = phase;
    taskENTER_CRITICAL();
    latest = result;
    sequence_diagnostics = trace;
    taskEXIT_CRITICAL();
    if (elapsed > c::FOLLOW_MAX_STEP_MS) wake = now;
  }
}


void run()
{
  TickType_t wake = xTaskGetTickCount();
  TickType_t previous = wake;
  uint32_t trajectory_start_ms = 0;
  bool tracking = false;
  bool overspeed_latched = false;
  uint32_t yaw_error_ms = 0;
  uint32_t pitch_error_ms = 0;
  const TrajectoryPoint initial = trajectory_sample_at_ms(0);
  constexpr uint32_t stretch = control_constants::FOLLOW_TRAJECTORY_TIME_SCALE;
  const uint32_t scaled_period_ms = TRAJECTORY_PERIOD_MS * stretch;

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(control_constants::FOLLOW_PERIOD_MS));

    // CAN 中断在更新电机状态，这里取一个短临界区的快照
    taskENTER_CRITICAL();
    const uint32_t now = xTaskGetTickCount();
    const float raw_yaw = yaw_motor.angle;
    const float raw_pitch = pitch_motor.angle;
    const float yaw_speed = yaw_motor.speed;
    const float pitch_speed = pitch_motor.speed;
    const uint32_t ys = yaw_stamp;
    const uint32_t ps = pitch_stamp;
    const bool motors_live = yaw_motor.is_alive(now) &&
      (control_constants::FOLLOW_SINGLE_YAW || pitch_motor.is_alive(now));
    bool run_requested = enabled;
    const bool recapture = capture_requested && !run_requested;
    const bool rc_stop = control_constants::FOLLOW_USE_RC_STOP && g_data.sw_r == 0;
    taskEXIT_CRITICAL();

    const uint32_t delta = now - previous;
    if (delta == 0U) {  // 同一 tick 内被再次唤醒：没有新的控制步进，保持上一条指令
      wake = now;
      continue;
    }
    previous = now;
    const float dt = static_cast<float>(delta) * 0.001f;

    // Wait for static IMU calibration before motor motion or zero capture.
    const bool imu_ready = gimbal_imu::ready();
    if (!imu_ready && tracking) {
      set_enabled(false);
      latch_fault(FAULT_IMU);
    }
    // Capture the IMU reference and encoder reference at the same start.
    if (motors_live && imu_ready &&
        (recapture || (!initial_pose_captured && startup_capture_allowed)) &&
        gimbal_imu::capture_reference(initial.yaw, initial.pitch)) {
      yaw_origin = raw_yaw;
      pitch_origin = raw_pitch;
      initial_pose_captured = true;
      startup_capture_allowed = false;
      tracking = false;
      reset_axis(yaw_state);
      reset_axis(pitch_state);
      taskENTER_CRITICAL();
      capture_requested = false;
      taskEXIT_CRITICAL();
    }
    // 反馈丢失：关节参考失效，必须重新解锁并重新对齐
    if (!motors_live && initial_pose_captured) {
      initial_pose_captured = false;
      set_enabled(false);
      latch_fault(FAULT_FEEDBACK);
    }
    if (rc_stop && tracking) {
      set_enabled(false);
      latch_fault(FAULT_NONE);
    }
    const bool timing_ok = delta <= control_constants::FOLLOW_MAX_STEP_MS;
    if (!timing_ok && tracking) {
      set_enabled(false);
      latch_fault(FAULT_TIMING);
    }
    // 恒力矩台架测试的超速保护
    if (control_constants::FOLLOW_CONSTANT_TORQUE_TEST && motors_live &&
        std::fabs(yaw_speed) >= control_constants::FOLLOW_TEST_TRIP_SPEED_RAD_S)
      overspeed_latched = true;
    if (overspeed_latched) {
      set_enabled(false);
      latch_fault(FAULT_OVERSPEED);
    }

    const bool configuration_ready = configured(control_constants::FOLLOW_YAW) &&
      (control_constants::FOLLOW_SINGLE_YAW || configured(control_constants::FOLLOW_PITCH));
    if (!configuration_ready) latch_fault(FAULT_CONFIG);
    const bool run_now = enabled && imu_ready && initial_pose_captured && motors_live && configuration_ready &&
                         !rc_stop && timing_ok && !overspeed_latched;
    if (run_now && !tracking) {
      trajectory_start_ms = now;
      reset_axis(yaw_state);
      reset_axis(pitch_state);
      yaw_error_ms = 0;
      pitch_error_ms = 0;
    }
    tracking = run_now;

    // 轨迹相位：按时间缩放缓放
    const uint32_t phase = tracking ? (now - trajectory_start_ms) % scaled_period_ms
                                    : now % scaled_period_ms;
    TrajectoryPoint target = trajectory_sample_at_ms(phase / stretch);
    target.yaw_vel /= stretch;
    target.pitch_vel /= stretch;
    target.yaw_acc /= static_cast<float>(stretch) * stretch;
    target.pitch_acc /= static_cast<float>(stretch) * stretch;

    // 反馈换到"轨迹坐标系"：参考第一点 = 解锁时的实际姿态
    const Feedback current = {
      {initial.yaw + control_constants::FOLLOW_YAW_DIRECTION * (raw_yaw - yaw_origin),
       control_constants::FOLLOW_YAW_DIRECTION * yaw_speed, ys,
       motors_live && initial_pose_captured},
      {initial.pitch + control_constants::FOLLOW_PITCH_DIRECTION * (raw_pitch - pitch_origin),
       control_constants::FOLLOW_PITCH_DIRECTION * pitch_speed, ps,
       motors_live && initial_pose_captured}};
    set_feedback(current);
    step(target, current, tracking, now, dt, phase, rc_stop);
    // Send our own 0x1FE frame: chassis_task.cpp used to do this.
    transmit_can1(now, !rc_stop);

    // 跟踪误差保护：参考与实际偏差过大且持续，就停机锁存，避免越追越飞
    if (tracking) {
      // Only copy the two errors here; avoid a full Result on this task stack.
      taskENTER_CRITICAL();
      const float yaw_error = latest.yaw.angle_error_rad;
      const float pitch_error = latest.pitch.angle_error_rad;
      taskEXIT_CRITICAL();
      yaw_error_ms = std::fabs(yaw_error) >
                         control_constants::FOLLOW_MAX_TRACKING_ERROR_RAD
                       ? yaw_error_ms + delta
                       : 0U;
      pitch_error_ms = (!control_constants::FOLLOW_SINGLE_YAW &&
                        std::fabs(pitch_error) >
                          control_constants::FOLLOW_MAX_TRACKING_ERROR_RAD)
                         ? pitch_error_ms + delta
                         : 0U;
      if (yaw_error_ms >= control_constants::FOLLOW_TRACKING_ERROR_MS ||
          pitch_error_ms >= control_constants::FOLLOW_TRACKING_ERROR_MS) {
        set_enabled(false);
        latch_fault(FAULT_TRACKING_ERROR);
        tracking = false;
      }
    }

    taskENTER_CRITICAL();
    const bool allowed = latest.active && enabled;
    requested_yaw_torque =
      allowed ? latest.yaw.control_output * control_constants::FOLLOW_YAW_DIRECTION : 0.0f;
    requested_pitch_torque = allowed && !control_constants::FOLLOW_SINGLE_YAW
                               ? latest.pitch.control_output *
                                   control_constants::FOLLOW_PITCH_DIRECTION
                               : 0.0f;
    command_stamp = now;
    command_active = allowed;
    taskEXIT_CRITICAL();

    if (!timing_ok) wake = now;
  }
}
}  // namespace trajectory_follow

extern "C" void trajectory_follow_task(void const * argument)
{
  (void)argument;
  // Tests remain peers. Only the explicit pitch sequence below hands control
  // from pitch_start to tracking; other selected tests keep exclusive ownership.
  if (control_constants::FOLLOW_PITCH_SEQUENCE_ENABLED) {
    trajectory_follow::run_pitch_sequence();
    return;
  }
  while (::homing::selected() || ::pitch_start::selected())
    vTaskDelay(pdMS_TO_TICKS(10));
  while (!gimbal_can::start()) vTaskDelay(pdMS_TO_TICKS(100));
  trajectory_follow::run();
}
