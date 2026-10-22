#include "pitch_start.hpp"

#include <cmath>

#include "FreeRTOS.h"
#include "control_constants.hpp"
#include "gimbal_can.hpp"
#include "gimbal_imu.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"
#include "trajectory_follow.hpp"

extern "C" void buzzer_signal_homing_done();

namespace pitch_start
{
namespace
{
namespace c = control_constants;
static_assert(configTICK_RATE_HZ == 1000, "Homing requires 1ms ticks");
sp::RM_Motor pitch(c::HOME_PITCH_ID, sp::RM_Motors::GM6020, c::HOME_PITCH_RATIO);
sp::RM_Motor m2006(c::HOME_M2006_ID, sp::RM_Motors::M2006, c::HOME_M2006_RATIO);
Status latest = {};
// Completion notifies once; the endpoint controller keeps running afterwards.
bool finished = false;
uint32_t pitch_stamp = 0, m_stamp = 0, pitch_rx = 0, m_rx = 0, tx_errors = 0;
float pitch_request = 0.0f, m_request = 0.0f;
uint32_t command_stamp = 0, last_tx = 0;
bool follow_pitch_claimed = false;
float follow_pitch_torque = 0.0f;
uint32_t follow_pitch_stamp = 0, last_m_tx = 0;
bool m_sent_once = false;
bool sent_once = false, start_requested = c::PITCH_START_ENABLED, stop_requested = false;
float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
struct PI
{
  float integral = 0.0f;
  float step(float target, float actual, float kp, float ki, float limit, float dt)
  {
    const float error = target - actual;
    const float inc = ki * error * dt;
    const float candidate = clamp(integral + inc, -limit, limit);
    const float raw = kp * error + candidate;
    if (!((raw > limit && inc > 0.0f) || (raw < -limit && inc < 0.0f))) integral = candidate;
    return clamp(kp * error + integral, -limit, limit);
  }
};
// Physical mechanism travel is measured by the IMU attached to the moving arm.
// One M2006-driven lift to the selected IMU travel, then persistent endpoint hold.
constexpr float PITCH_START_BODY_TRAVEL_RAD = 1.2f;
constexpr uint32_t PITCH_START_IMU_READY_TIMEOUT_MS = 20000U;
constexpr float PITCH_START_MIN_IMU_SEEK_RAD = 0.01745329252f;  // 1 degree to learn direction.

// Dedicated pitch_start endpoint hold. These angles are M2006 output-shaft rad.
// Manufacturer continuous torque is 1 Nm; do not use the driver's electrical
// saturation (1.8 Nm in its conversion) as a continuous holding specification.
constexpr float END_HOLD_MAX_TORQUE_NM = 1.0f;
constexpr float END_HOLD_KP = 60.0f;  // Nm/rad: displacement resistance.
constexpr float END_HOLD_KI = 15.0f;  // Nm/(rad*s): slow load learning avoids stiction hunting.
constexpr float END_HOLD_KD = 0.4f;   // Nm/(rad/s): motion damping.
constexpr float END_HOLD_SPEED_FILTER_S = 0.002f;
// MOTOR OUTPUT-SHAFT tolerance, not the IMU body angle (~0.086 degree).
constexpr float END_HOLD_POSITION_TOL_RAD = 0.0015f;
constexpr float END_HOLD_INTEGRATE_MAX_SPEED_RAD_S = 0.05f;
static_assert(
  END_HOLD_MAX_TORQUE_NM <= sp::M2006_P36_MAX_TORQUE,
  "Endpoint current command exceeds the motor driver's limit");

struct EndpointHold
{
  float integral = 0.0f, filtered_speed = 0.0f;
  bool initialized = false;

  float step(float error, float speed, float dt)
  {
    if (!initialized) {
      // Motion current is not a gravity measurement. At a passively balanced
      // endpoint, inheriting it as I torque pushes the mechanism away at zero error.
      integral = 0.0f;
      filtered_speed = speed;
      initialized = true;
    }
    filtered_speed += dt / (END_HOLD_SPEED_FILTER_S + dt) * (speed - filtered_speed);
    // Continuous dead zone: no P jump at the tolerance boundary. Outside it,
    // position resistance and the full 1Nm authority remain available.
    const float position_error =
      error > END_HOLD_POSITION_TOL_RAD
        ? error - END_HOLD_POSITION_TOL_RAD
        : (error < -END_HOLD_POSITION_TOL_RAD ? error + END_HOLD_POSITION_TOL_RAD : 0.0f);
    const float base = END_HOLD_KP * position_error - END_HOLD_KD * filtered_speed;
    // Do not chase capture transients/backlash with I. Freeze (never erase) the
    // learned load torque inside the tolerance or while moving appreciably.
    const bool integrate = std::fabs(filtered_speed) <= END_HOLD_INTEGRATE_MAX_SPEED_RAD_S;
    const float increment = integrate ? END_HOLD_KI * position_error * dt : 0.0f;
    const float candidate =
      clamp(integral + increment, -END_HOLD_MAX_TORQUE_NM, END_HOLD_MAX_TORQUE_NM);
    const float raw = base + candidate;
    // Conditional integration: hand loading at saturation must not wind up the I term.
    if (!((raw > END_HOLD_MAX_TORQUE_NM && increment > 0.0f) ||
          (raw < -END_HOLD_MAX_TORQUE_NM && increment < 0.0f)))
      integral = candidate;
    // A balanced stationary endpoint needs no artificial torque. If gravity
    // requires a load torque, retain the I term learned after a real deviation.
    return clamp(base + integral, -END_HOLD_MAX_TORQUE_NM, END_HOLD_MAX_TORQUE_NM);
  }
};

struct Feedback
{
  float pa, ps, pt, ma, ms, mt;
  bool fresh;
  float body_pitch_rad;
  bool imu_fresh;
  bool external_pitch_control;
};
struct Controller
{
  Status out = {};
  PI pitch_pi, m_pi;
  EndpointHold endpoint_hold;
  bool endpoint_captured = false;
  float endpoint_pitch_angle = 0.0f;
  float imu_seek_origin = 0.0f, imu_return_origin = 0.0f, imu_return_direction = 0.0f;
  uint32_t entered = 0, stall_since = 0;
  uint32_t pitch_os_since = 0, m_os_since = 0;
  bool m2006_home_done = false;
  bool m2006_hold_locked = false;
  uint32_t m2006_hold_since = 0;
  float m2006_hold_angle = 0.0f;
  float phaseb_torque = 0.0f;  // Slewed GM6020 support only after reaching the endpoint.
  bool stall_active = false, settle_active = false, seek_motion_seen = false;
  float pitch_start = 0.0f, stall_angle = 0.0f, m_origin = 0.0f, pitch_return_origin = 0.0f;
  float pitch_speed_cmd = 0.0f, m_speed_cmd = 0.0f;

  void fault(Fault f)
  {
    out.fault_pitch_speed_rad_s = out.pitch_speed_rad_s;
    out.fault_m2006_speed_rad_s = out.m2006_speed_rad_s;
    out.stage = Stage::Fault;
    out.fault = f;
    out.homed = false;
    out.pitch_torque_nm = out.m2006_torque_nm = 0.0f;
    pitch_pi = {};
    m_pi = {};
  }
  void begin(uint32_t now)
  {
    *this = Controller{};
    out.stage = Stage::WaitFeedback;
    entered = now;
    const bool config =
      c::HOME_PITCH_SPEED_KP > 0.0f && c::HOME_PITCH_SPEED_KI >= 0.0f &&
      c::HOME_M2006_POSITION_KP > 0.0f && c::HOME_M2006_SPEED_KP > 0.0f &&
      c::HOME_M2006_SPEED_KI >= 0.0f && c::HOME_TARGET_ACCEL_RAD_S2 > 0.0f &&
      c::HOME_PITCH_MAX_TORQUE_NM <= sp::GM6020_MAX_TORQUE && c::HOME_M2006_MAX_TORQUE_NM > 0.0f &&
      c::HOME_M2006_MAX_TORQUE_NM <= sp::M2006_P36_MAX_TORQUE &&
      c::HOME_PITCH_MAX_TRAVEL_RAD > 0.0f && c::HOME_PITCH_RETURN_MOVE_RAD > 0.0f &&
      c::HOME_PITCH_SEEK_SPEED_RAD_S > 0.0f && c::HOME_M2006_MAX_SPEED_RAD_S > 0.0f &&
      c::HOME_PITCH_RETURN_MAX_TORQUE_NM <= sp::GM6020_MAX_TORQUE &&
      c::HOME_PITCH_HOLD_MAX_TORQUE_NM <= sp::GM6020_MAX_TORQUE;
    if (!config) fault(Fault::Config);
  }
  void hold_endpoint(const Feedback & f, uint32_t now, float dt)
  {
    if (!endpoint_captured) {
      endpoint_captured = true;
      endpoint_pitch_angle = f.pa;
      // m2006_hold_angle was latched by the ORIGINAL endpoint trigger.
      // Neither target is recaptured after sagging or an external disturbance.
    }
    out.m2006_zero_rad = m2006_hold_angle;
    out.pitch_zero_rad = endpoint_pitch_angle;
    out.m2006_locked = true;
    out.m2006_torque_nm = endpoint_hold.step(m2006_hold_angle - f.ma, f.ms, dt);

    // Once the IMU follower owns pitch, the previous lower-stop hold is released.
    // M2006 above keeps its independently captured shaft position throughout.
    if (f.external_pitch_control) {
      phaseb_torque = 0.0f;
      out.pitch_torque_nm = 0.0f;
    }
    else {
      const float pitch_error = endpoint_pitch_angle - f.pa;
      const float want = clamp(
        c::HOME_PITCH_PHASEB_STIFF_NM_PER_RAD * pitch_error -
          c::HOME_PITCH_PHASEB_DAMP_NM_PER_RAD_S * f.ps,
        -c::HOME_PITCH_PHASEB_MAX_TORQUE_NM, c::HOME_PITCH_PHASEB_MAX_TORQUE_NM);
      const float max_step = 3.0f * dt;
      phaseb_torque += clamp(want - phaseb_torque, -max_step, max_step);
      out.pitch_torque_nm = phaseb_torque;
    }

    // Preserve the previous 5s completion indication, with NO output/target reset.
    if (now - m2006_hold_since >= 5000U) {
      out.stage = Stage::Done;
      out.homed = true;
      m2006_home_done = true;
    }
  }

  void step(const Feedback & f, uint32_t now, float dt)
  {
    out.stamp_ms = now;
    out.feedback_fresh = f.fresh;
    out.pitch_angle_rad = f.pa;
    out.m2006_relative_rad = f.ma - m_origin;
    // Ch3/4 are the requested/measured BODY pitch travel, never M2006 shaft travel.
    out.pitch_return_target_relative_rad = PITCH_START_BODY_TRAVEL_RAD;
    if (f.imu_fresh && std::isfinite(f.body_pitch_rad) && imu_return_direction != 0.0f)
      out.pitch_return_relative_rad = imu_return_direction * (f.body_pitch_rad - imu_return_origin);
    out.pitch_feedback_torque_nm = f.pt;
    out.m2006_feedback_torque_nm = f.mt;
    out.pitch_speed_rad_s = f.ps;
    out.m2006_speed_rad_s = f.ms;
    out.stall_elapsed_ms = 0;
    out.pitch_torque_nm = out.m2006_torque_nm = 0.0f;
    if (out.stage == Stage::Idle || out.stage == Stage::Fault) return;
    if (out.stage == Stage::WaitFeedback) {
      // Motor motion must not prevent the existing static IMU calibration.
      // ready/raw_pitch are usable without trajectory capture_reference/alignment.
      if (!f.fresh || !f.imu_fresh || !std::isfinite(f.body_pitch_rad)) {
        if (now - entered >= PITCH_START_IMU_READY_TIMEOUT_MS) fault(Fault::Feedback);
        return;
      }
      pitch_start = f.pa;
      m_origin = f.ma;
      pitch_return_origin = f.pa;
      entered = now;
      imu_seek_origin = f.body_pitch_rad;
      out.pitch_return_relative_rad = 0.0f;
      out.stage = Stage::SeekPitch;
    }
    if (!f.fresh) {
      fault(Fault::Feedback);
      return;
    }
    if (
      !std::isfinite(f.pa) || !std::isfinite(f.ps) || !std::isfinite(f.pt) ||
      !std::isfinite(f.ma) || !std::isfinite(f.ms) || !std::isfinite(f.mt)) {
      fault(Fault::Feedback);
      return;
    }
    if (dt <= 0.0f || dt > c::HOME_MAX_STEP_MS * 0.001f) {
      fault(Fault::Timing);
      return;
    }
    // 超速判据加 50 ms 去抖：硬挡块撞击或连杆耦合会产生单帧差分尖峰，
    // 单帧即停机会把"正常撞击"误判成超速。
    constexpr uint32_t kOverspeedDebounceMs = 50U;
    if (std::fabs(f.ps) > c::HOME_OVERSPEED_RAD_S) {
      if (!pitch_os_since) pitch_os_since = now;
    }
    else
      pitch_os_since = 0;
    if (std::fabs(f.ms) > c::HOME_M2006_OVERSPEED_RAD_S) {
      if (!m_os_since) m_os_since = now;
    }
    else
      m_os_since = 0;
    if (pitch_os_since && now - pitch_os_since >= kOverspeedDebounceMs) {
      fault(Fault::PitchOverspeed);
      return;
    }
    if (m_os_since && now - m_os_since >= kOverspeedDebounceMs) {
      fault(Fault::M2006Overspeed);
      return;
    }
    // Feedback, timing and overspeed guards above still apply while holding.
    // This path owns the endpoint in BOTH MovePitch and Done, indefinitely.
    if (m2006_hold_locked) {
      hold_endpoint(f, now, dt);
      return;
    }
    // During measurement/motion a stale IMU cannot authorize a physical endpoint.
    // Once locked, independent motor-encoder holding above survives IMU loss.
    if (!f.imu_fresh || !std::isfinite(f.body_pitch_rad)) {
      fault(Fault::Feedback);
      return;
    }
    if (out.stage == Stage::SeekPitch) {
      const float progress = c::HOME_PITCH_DIRECTION * (f.pa - pitch_start);
      if (progress >= c::HOME_PITCH_MIN_SEEK_TRAVEL_RAD) seek_motion_seen = true;
      if (now - entered >= c::HOME_PITCH_TIMEOUT_MS) {
        if (!seek_motion_seen)
          fault(Fault::NoSeekMotion);
        else
          fault(Fault::SeekTimeout);
        return;
      }
      if (std::fabs(f.pa - pitch_start) > c::HOME_PITCH_MAX_TRAVEL_RAD) {
        fault(Fault::SeekTravel);
        return;
      }
      const float target = c::HOME_PITCH_DIRECTION * c::HOME_PITCH_SEEK_SPEED_RAD_S;
      const float dv = c::HOME_TARGET_ACCEL_RAD_S2 * dt;
      pitch_speed_cmd += clamp(target - pitch_speed_cmd, -dv, dv);
      out.pitch_torque_nm = pitch_pi.step(
        pitch_speed_cmd, f.ps, c::HOME_PITCH_SPEED_KP, c::HOME_PITCH_SPEED_KI,
        c::HOME_PITCH_MAX_TORQUE_NM, dt);
      // 判定"顶到挡块"只用：已产生过位移 + 速度≈0 + 指令力矩够大。
      // 不再依赖反馈力矩 f.pt：6020 在堵转时上报的电流/力矩受采样和限幅影响，
      // 出现"实际顶住了但 f.pt 达不到阈值"时会导致 stall 永远不成立、一直顶到超时。
      // 判定"顶到挡块"只靠：已产生过位移 + 指令力矩够大。
      // 不能用速度做条件：顶住挡块嗡嗡震动时编码器速度会来回跳，导致 stall 计时
      // 每帧被清零、永远凑不满 STALL_MS。真正的"没在动"由下面的位移判据保证。
      const bool candidate =
        seek_motion_seen && now - entered >= c::HOME_PITCH_STARTUP_BLANK_MS &&
        c::HOME_PITCH_DIRECTION * out.pitch_torque_nm >= c::HOME_PITCH_STALL_TORQUE_NM;
      if (!candidate)
        stall_active = false;
      else {
        if (!stall_active || std::fabs(f.pa - stall_angle) > c::HOME_PITCH_STALL_TRAVEL_RAD) {
          stall_active = true;
          stall_since = now;
          stall_angle = f.pa;
        }
        out.stall_elapsed_ms = now - stall_since;
        if (out.stall_elapsed_ms >= c::HOME_PITCH_STALL_MS) {
          // Learn the physical pitch sign from rearward motion, independently of
          // motor/gearbox polarity. A nonresponsive/wrong IMU axis must not drive on.
          const float body_seek = f.body_pitch_rad - imu_seek_origin;
          if (std::fabs(body_seek) < PITCH_START_MIN_IMU_SEEK_RAD) {
            fault(Fault::Config);
            return;
          }
          out.pitch_up_imu_direction = body_seek > 0.0f ? 1.0f : -1.0f;
          imu_return_direction = -out.pitch_up_imu_direction;
          imu_return_origin = f.body_pitch_rad;  // Seated rear stop is physical travel zero.
          out.pitch_torque_nm = 0.0f;
          pitch_pi = {};
          pitch_speed_cmd = 0.0f;
          pitch_return_origin = f.pa;
          m_origin = f.ma;
          out.pitch_return_relative_rad = 0.0f;
          out.m2006_relative_rad = 0.0f;
          out.pitch_return_target_relative_rad = PITCH_START_BODY_TRAVEL_RAD;
          out.stage = Stage::MovePitch;
          entered = now;
          m_pi = {};
          m_speed_cmd = 0.0f;
        }
      }
      return;  // M2006 remains zero for this entire phase.
    }
    if (out.stage == Stage::MovePitch) {
      const float body_travel = imu_return_direction * (f.body_pitch_rad - imu_return_origin);
      out.pitch_return_relative_rad = body_travel;
      out.pitch_return_target_relative_rad = PITCH_START_BODY_TRAVEL_RAD;
      if (body_travel >= PITCH_START_BODY_TRAVEL_RAD) {
        // Lock on this tick at the selected travel from the seated rear-stop IMU pose.
        // There is no intermediate handoff or additional second-stage travel.
        m2006_hold_locked = true;
        m2006_hold_since = now;
        m2006_hold_angle = f.ma;
        hold_endpoint(f, now, dt);  // Replace drive current with hold on THIS tick.
        return;
      }
      // Preserve the existing M2006 breakaway/cruise drive and its direction.
      // GM6020 stays released throughout the single lift; M2006 drives it all.
      // Both encoder hold targets are captured only at the selected endpoint.
      const float dir = -c::HOME_PITCH_DIRECTION;
      const float tq = (std::fabs(f.ms) < c::HOME_M2006_BREAKAWAY_SPEED_RAD_S)
                         ? c::HOME_M2006_BREAKAWAY_TORQUE_NM
                         : c::HOME_M2006_CRUISE_TORQUE_NM;
      out.pitch_torque_nm = 0.0f;
      out.m2006_torque_nm =
        dir * c::HOME_M2006_DRIVE_DIR * tq - c::HOME_M2006_DAMPING_NM_PER_RAD_S * f.ms;
      if (now - entered >= c::HOME_PITCH_RETURN_TIMEOUT_MS) fault(Fault::MoveTimeout);
      return;
    }
    // Done is reached only through the endpoint path above. An inconsistent
    // state must stop rather than execute the old homing/final-rotation branch.
    if (out.stage == Stage::Done) fault(Fault::Config);
  }
};
bool submit(uint32_t id, uint8_t * data)
{
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
    ++tx_errors;
    return false;
  }
  CAN_TxHeaderTypeDef h = {};
  h.StdId = id;
  h.IDE = CAN_ID_STD;
  h.RTR = CAN_RTR_DATA;
  h.DLC = 8;
  uint32_t mailbox = 0;
  if (HAL_CAN_AddTxMessage(&hcan1, &h, data, &mailbox) != HAL_OK) {
    ++tx_errors;
    return false;
  }
  return true;
}
}  // namespace
// Standalone selection; neither enabled nor started by homing.
bool selected() { return c::PITCH_START_ENABLED; }
void start()
{
  taskENTER_CRITICAL();
  start_requested = true;
  stop_requested = false;
  follow_pitch_claimed = false;
  follow_pitch_torque = 0.0f;
  taskEXIT_CRITICAL();
}
void cancel()
{
  taskENTER_CRITICAL();
  finished = false;
  follow_pitch_claimed = false;
  follow_pitch_torque = 0.0f;
  stop_requested = true;
  start_requested = false;
  pitch_request = m_request = 0.0f;
  taskEXIT_CRITICAL();
}
Status snapshot()
{
  taskENTER_CRITICAL();
  Status s = latest;
  s.pitch_rx_frames = pitch_rx;
  s.m2006_rx_frames = m_rx;
  s.tx_errors = tx_errors;
  taskEXIT_CRITICAL();
  return s;
}
bool submit_pitch_follow_torque(float torque_nm, uint32_t now_ms)
{
  if (!c::FOLLOW_PITCH_SEQUENCE_ENABLED || !selected() || !std::isfinite(torque_nm)) return false;
  taskENTER_CRITICAL();
  const bool ready = latest.stage == Stage::Done && latest.homed && latest.m2006_locked &&
                     latest.feedback_fresh && latest.fault == Fault::None && !stop_requested &&
                     now_ms - latest.stamp_ms < c::HOME_COMMAND_TIMEOUT_MS;
  if (ready) {
    follow_pitch_torque =
      clamp(torque_nm, -c::FOLLOW_PITCH_MAX_TORQUE_NM, c::FOLLOW_PITCH_MAX_TORQUE_NM);
    follow_pitch_stamp = now_ms;
    follow_pitch_claimed = true;
  }
  taskEXIT_CRITICAL();
  return ready;
}
bool receive_can1(const CAN_RxHeaderTypeDef & h, uint8_t * data, uint32_t now)
{
  if (!selected() || !data || h.IDE != CAN_ID_STD || h.RTR != CAN_RTR_DATA || h.DLC != 8)
    return false;
  if (h.StdId == pitch.rx_id) {
    pitch.read(data, now);
    pitch_stamp = now;
    ++pitch_rx;
    return true;
  }
  if (h.StdId == m2006.rx_id) {
    m2006.read(data, now);
    m_stamp = now;
    ++m_rx;
    return true;
  }
  return false;
}
void transmit_can1(uint32_t now)
{
  if (!selected()) return;
  taskENTER_CRITICAL();
  const bool fast_pitch = c::FOLLOW_PITCH_SEQUENCE_ENABLED && follow_pitch_claimed;
  const bool live = now - command_stamp < c::HOME_COMMAND_TIMEOUT_MS && !stop_requested;
  const float pt = live ? pitch_request : 0.0f, mt = live ? m_request : 0.0f;
  taskEXIT_CRITICAL();
  const uint32_t pitch_period = fast_pitch ? c::FOLLOW_CAN_TX_INTERVAL_MS : c::HOME_CAN_INTERVAL_MS;
  // Exactly one owner of 0x1FE, with all yaw slots zero in this pitch-only mode.
  if (!sent_once || now - last_tx >= pitch_period) {
    uint8_t data[8] = {};
    pitch.cmd(pt);
    pitch.write(data);
    if (submit(pitch.tx_id, data)) {
      last_tx = now;
      sent_once = true;
    }
  }
  // Keep the proven M2006 5ms holding cadence; faster pitch does not flood this group.
  if (!m_sent_once || now - last_m_tx >= c::HOME_CAN_INTERVAL_MS) {
    uint8_t data[8] = {};
    m2006.cmd(mt);
    m2006.write(data);
    if (submit(m2006.tx_id, data)) {
      last_m_tx = now;
      m_sent_once = true;
    }
  }
}

void run()
{
  Controller ctrl;
  TickType_t wake = xTaskGetTickCount(), previous = wake;
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
    const uint32_t now = xTaskGetTickCount();
    const uint32_t elapsed = now - previous;
    if (!elapsed) {
      wake = now;
      continue;
    }
    previous = now;
    const auto imu = gimbal_imu::read();  // Snapshot only; grading task owns SPI/update.
    taskENTER_CRITICAL();
    const Feedback f = {
      pitch.angle,
      pitch.speed,
      pitch.torque,
      m2006.angle,
      m2006.speed,
      m2006.torque,
      pitch.is_open() && m2006.is_open() && now - pitch_stamp < c::HOME_FEEDBACK_TIMEOUT_MS &&
        now - m_stamp < c::HOME_FEEDBACK_TIMEOUT_MS,
      imu.raw_pitch_rad,
      imu.ready,
      c::FOLLOW_PITCH_SEQUENCE_ENABLED && follow_pitch_claimed};
    const bool external_pitch = c::FOLLOW_PITCH_SEQUENCE_ENABLED && follow_pitch_claimed;
    const bool external_fresh = now - follow_pitch_stamp < c::FOLLOW_COMMAND_TIMEOUT_MS;
    const float external_torque = follow_pitch_torque;
    const bool go = start_requested, stop = stop_requested;
    start_requested = false;
    stop_requested = false;
    taskEXIT_CRITICAL();
    if (stop)
      ctrl = Controller{};
    else if (go)
      ctrl.begin(now);
    ctrl.step(f, now, elapsed * 0.001f);
    if (external_pitch && ctrl.out.stage == Stage::Done && ctrl.out.fault == Fault::None)
      ctrl.out.pitch_torque_nm = external_fresh ? external_torque : 0.0f;
    const bool completed = ctrl.out.homed && ctrl.m2006_home_done;
    taskENTER_CRITICAL();
    latest = ctrl.out;
    pitch_request = ctrl.out.pitch_torque_nm;
    m_request = ctrl.out.m2006_torque_nm;
    command_stamp = now;
    // Notify completion only; remain in this program and preserve its stop/hold logic.
    if (completed && !finished) {
      finished = true;
      // Signal ONCE: doing this every cycle re-triggered the buzzer forever.
      buzzer_signal_homing_done();
    }
    taskEXIT_CRITICAL();

    // Only this selected program may transmit its motor commands.
    transmit_can1(now);
    if (elapsed > c::HOME_MAX_STEP_MS) wake = now;
  }
}
}  // namespace pitch_start
extern "C" void pitch_start_task(void const * argument)
{
  (void)argument;
  if (pitch_start::selected()) {
    while (!gimbal_can::start()) vTaskDelay(pdMS_TO_TICKS(100));
    pitch_start::run();
  }
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
