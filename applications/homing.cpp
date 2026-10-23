#include "homing.hpp"
#include "infrared_pitch_auto.hpp"
#include "gimbal_can.hpp"
#include <cmath>
#include "FreeRTOS.h"
#include "task.h"
#include "control_constants.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern "C" void buzzer_signal_homing_done();


namespace homing {
namespace {
namespace c = control_constants;
static_assert(configTICK_RATE_HZ == 1000, "Homing requires 1ms ticks");
sp::RM_Motor pitch(c::HOME_PITCH_ID, sp::RM_Motors::GM6020, c::HOME_PITCH_RATIO);
sp::RM_Motor m2006(c::HOME_M2006_ID, sp::RM_Motors::M2006, c::HOME_M2006_RATIO);
Status latest = {};
// Set only when BOTH stages are done (6020 zero AND M2006 final rotation).
bool finished = false;
uint32_t pitch_stamp=0, m_stamp=0, pitch_rx=0, m_rx=0, tx_errors=0;
float pitch_request=0.0f, m_request=0.0f;
uint32_t command_stamp=0, last_tx=0;
bool sent_once=false, start_requested=c::HOMING_ENABLED, stop_requested=false;
float clamp(float v,float lo,float hi) { return v<lo ? lo : (v>hi ? hi : v); }
struct PI {
  float integral=0.0f;
  float step(float target,float actual,float kp,float ki,float limit,float dt) {
    const float error=target-actual;
    const float inc=ki*error*dt;
    const float candidate=clamp(integral+inc,-limit,limit);
    const float raw=kp*error+candidate;
    if (!((raw>limit && inc>0.0f)||(raw< -limit && inc<0.0f))) integral=candidate;
    return clamp(kp*error+integral,-limit,limit);
  }
};
struct Feedback { float pa,ps,pt,ma,ms,mt; bool fresh; };
struct Controller {
  Status out={};
  PI pitch_pi, m_pi;
  uint32_t entered=0, stall_since=0, settled_since=0;
  uint32_t pitch_os_since=0, m_os_since=0;  // 超速去抖计时
  uint32_t done_since=0;  // Done 阶段计时，用于 500ms 后锁定
  bool locked=false;
  float m2006_home_target=0.0f;      // M2006 最终转动目标
  bool m2006_home_armed=false, m2006_home_done=false;
  uint32_t m2006_home_since=0;
  uint32_t m2006_settle_since=0;
  bool m2006_pullback=false;  // reached pullback angle -> reverse drive
  bool pullback_wait=false;   // reached the bottom -> hold 5 s before reversing
  bool phase_a_hold=false;    // phase A done -> lock the M2006 for 5 s
  uint32_t phase_a_hold_since=0;
  float phase_a_hold_angle=0.0f;
  uint32_t pullback_wait_since=0;
  bool m2006_hold_locked=false;   // reached ch10 target -> hold engaged
  uint32_t m2006_hold_since=0;
  float m2006_hold_angle=0.0f;
  bool pitch_locked=false;      // 6020 locked after the 15 deg M2006 backoff
  float pitch_lock_angle=0.0f;
  float phaseb_torque=0.0f;   // ramped phase-B lock torque (avoids the handoff jerk)
  float m2006_phase_start=0.0f;  // arrival -> 0.5 s hold -> permanent lock
  bool stall_active=false, settle_active=false, seek_motion_seen=false;
  float pitch_start=0.0f, stall_angle=0.0f, m_origin=0.0f, pitch_return_origin=0.0f;
  float pitch_speed_cmd=0.0f, m_speed_cmd=0.0f;

  void fault(Fault f) {
    out.fault_pitch_speed_rad_s=out.pitch_speed_rad_s;
    out.fault_m2006_speed_rad_s=out.m2006_speed_rad_s;
    out.stage=Stage::Fault;out.fault=f;out.homed=false;
    out.pitch_torque_nm=out.m2006_torque_nm=0.0f;
    pitch_pi={};m_pi={};
  }
  void begin(uint32_t now) {
    *this=Controller{};out.stage=Stage::WaitFeedback;entered=now;
    const bool config=c::HOME_PITCH_SPEED_KP>0.0f && c::HOME_PITCH_SPEED_KI>=0.0f &&
      c::HOME_M2006_POSITION_KP>0.0f && c::HOME_M2006_SPEED_KP>0.0f &&
      c::HOME_M2006_SPEED_KI>=0.0f && c::HOME_TARGET_ACCEL_RAD_S2>0.0f &&
      c::HOME_PITCH_MAX_TORQUE_NM<=sp::GM6020_MAX_TORQUE &&
      c::HOME_M2006_MAX_TORQUE_NM>0.0f &&
      c::HOME_M2006_MAX_TORQUE_NM<=sp::M2006_P36_MAX_TORQUE &&
      c::HOME_PITCH_MAX_TRAVEL_RAD>0.0f && c::HOME_PITCH_RETURN_MOVE_RAD>0.0f &&
      c::HOME_PITCH_SEEK_SPEED_RAD_S>0.0f && c::HOME_M2006_MAX_SPEED_RAD_S>0.0f &&
      c::HOME_PITCH_RETURN_MAX_TORQUE_NM<=sp::GM6020_MAX_TORQUE &&
      c::HOME_PITCH_HOLD_MAX_TORQUE_NM<=sp::GM6020_MAX_TORQUE;
    if (!config) fault(Fault::Config);
  }
  void step(const Feedback & f,uint32_t now,float dt) {
    out.stamp_ms=now;out.feedback_fresh=f.fresh;
    out.pitch_angle_rad=f.pa;out.m2006_relative_rad=f.ma-m_origin;
    out.pitch_return_relative_rad=f.pa-pitch_return_origin;
    out.pitch_feedback_torque_nm=f.pt;out.m2006_feedback_torque_nm=f.mt;
    out.pitch_speed_rad_s=f.ps;out.m2006_speed_rad_s=f.ms;
    out.stall_elapsed_ms=0;
    out.pitch_torque_nm=out.m2006_torque_nm=0.0f;
    if (out.stage==Stage::Idle || out.stage==Stage::Fault) return;
    if (out.stage==Stage::WaitFeedback) {
      if (!f.fresh) { if (now-entered>=c::HOME_READY_TIMEOUT_MS) fault(Fault::Feedback); return; }
      pitch_start=f.pa;m_origin=f.ma;pitch_return_origin=f.pa;entered=now;
      out.pitch_return_relative_rad=0.0f;
      out.stage=Stage::SeekPitch;
    }
    if (!f.fresh) { fault(Fault::Feedback);return; }
    if (!std::isfinite(f.pa)||!std::isfinite(f.ps)||!std::isfinite(f.pt)||
        !std::isfinite(f.ma)||!std::isfinite(f.ms)||!std::isfinite(f.mt)) { fault(Fault::Feedback);return; }
    if (dt<=0.0f || dt>c::HOME_MAX_STEP_MS*0.001f) { fault(Fault::Timing);return; }
    // 超速判据加 50 ms 去抖：硬挡块撞击或连杆耦合会产生单帧差分尖峰，
    // 单帧即停机会把"正常撞击"误判成超速。
    constexpr uint32_t kOverspeedDebounceMs=50U;
    if (std::fabs(f.ps)>c::HOME_OVERSPEED_RAD_S) { if (!pitch_os_since) pitch_os_since=now; }
    else pitch_os_since=0;
    if (std::fabs(f.ms)>c::HOME_M2006_OVERSPEED_RAD_S) { if (!m_os_since) m_os_since=now; }
    else m_os_since=0;
    if (pitch_os_since && now-pitch_os_since>=kOverspeedDebounceMs) {
      fault(Fault::PitchOverspeed);return;
    }
    if (m_os_since && now-m_os_since>=kOverspeedDebounceMs) {
      fault(Fault::M2006Overspeed);return;
    }
    if (out.stage==Stage::SeekPitch) {
      const float progress=c::HOME_PITCH_DIRECTION*(f.pa-pitch_start);
      if (progress>=c::HOME_PITCH_MIN_SEEK_TRAVEL_RAD) seek_motion_seen=true;
      if (now-entered>=c::HOME_PITCH_TIMEOUT_MS) {
        if (!seek_motion_seen) fault(Fault::NoSeekMotion);
        else fault(Fault::SeekTimeout);
        return;
      }
      if (std::fabs(f.pa-pitch_start)>c::HOME_PITCH_MAX_TRAVEL_RAD) {fault(Fault::SeekTravel);return;}
      const float target=c::HOME_PITCH_DIRECTION*c::HOME_PITCH_SEEK_SPEED_RAD_S;
      const float dv=c::HOME_TARGET_ACCEL_RAD_S2*dt;
      pitch_speed_cmd+=clamp(target-pitch_speed_cmd,-dv,dv);
      out.pitch_torque_nm=pitch_pi.step(pitch_speed_cmd,f.ps,c::HOME_PITCH_SPEED_KP,
        c::HOME_PITCH_SPEED_KI,c::HOME_PITCH_MAX_TORQUE_NM,dt);
      // 判定"顶到挡块"只用：已产生过位移 + 速度≈0 + 指令力矩够大。
      // 不再依赖反馈力矩 f.pt：6020 在堵转时上报的电流/力矩受采样和限幅影响，
      // 出现"实际顶住了但 f.pt 达不到阈值"时会导致 stall 永远不成立、一直顶到超时。
      // 判定"顶到挡块"只靠：已产生过位移 + 指令力矩够大。
      // 不能用速度做条件：顶住挡块嗡嗡震动时编码器速度会来回跳，导致 stall 计时
      // 每帧被清零、永远凑不满 STALL_MS。真正的"没在动"由下面的位移判据保证。
      const bool candidate=seek_motion_seen && now-entered>=c::HOME_PITCH_STARTUP_BLANK_MS &&
        c::HOME_PITCH_DIRECTION*out.pitch_torque_nm>=c::HOME_PITCH_STALL_TORQUE_NM;
      if (!candidate) stall_active=false;
      else {
        if (!stall_active || std::fabs(f.pa-stall_angle)>c::HOME_PITCH_STALL_TRAVEL_RAD) {
          stall_active=true;stall_since=now;stall_angle=f.pa;
        }
        out.stall_elapsed_ms=now-stall_since;
        if (out.stall_elapsed_ms>=c::HOME_PITCH_STALL_MS) {
          // Remove seek current, then reverse THIS GM6020; M2006 stays released.
          out.pitch_torque_nm=0.0f;pitch_pi={};pitch_speed_cmd=0.0f;
          pitch_return_origin=f.pa;m_origin=f.ma;
          out.pitch_return_relative_rad=0.0f;out.m2006_relative_rad=0.0f;
          out.pitch_return_target_relative_rad=-c::HOME_PITCH_DIRECTION*c::HOME_PITCH_RETURN_MOVE_RAD;
          out.stage=Stage::MovePitch;entered=now;m_pi={};m_speed_cmd=0.0f;
        }
      }
      return; // M2006 remains zero for this entire phase.
    }
    if (out.stage==Stage::MovePitch) {
      // New scheme: the M2006 does all the motion.
      //   Phase A: 6020 released, M2006 drives the axis back 15 deg.
      //   Phase B: 6020 locks at that angle, M2006 continues another 45 deg.
      const float dir=-c::HOME_PITCH_DIRECTION;
      // Two-level drive: breakaway torque while (nearly) stationary, small torque to cruise.
      // A single constant torque either cannot start (0.15) or runs away (0.80).
      const float tq=(std::fabs(f.ms)<c::HOME_M2006_BREAKAWAY_SPEED_RAD_S) ?
        c::HOME_M2006_BREAKAWAY_TORQUE_NM : c::HOME_M2006_CRUISE_TORQUE_NM;
      // Damping must oppose the actual motion, whatever the drive direction is.
      out.m2006_torque_nm=dir*c::HOME_M2006_DRIVE_DIR*tq-c::HOME_M2006_DAMPING_NM_PER_RAD_S*f.ms;
      if (!pitch_locked) {
        out.pitch_torque_nm=0.0f;                       // 6020 free, back-driven
        const float travel=dir*(f.pa-pitch_return_origin);
        out.pitch_return_relative_rad=travel;
        out.pitch_return_target_relative_rad=c::HOME_PITCH_RETURN_MOVE_RAD;
        if (travel>=c::HOME_PITCH_RETURN_MOVE_RAD) {
          pitch_locked=true;
          pitch_lock_angle=f.pa;
          m2006_phase_start=f.ma;
          // Phase A finished: lock the M2006 here for 5 s before phase B.
          // DISABLED for isolation test (set to true to re-enable).
          phase_a_hold=false;
          phase_a_hold_since=now;
          phase_a_hold_angle=f.ma;
        }
      }
      else {
        // 6020 stiff hold at the recorded angle
        const float err_lock=pitch_lock_angle-f.pa;
        // Weak lock here so the M2006 translation can still move the axis.
        const float e_b=(std::fabs(err_lock)>c::HOME_PITCH_HOLD_DEADBAND_RAD) ? err_lock : 0.0f;
        const float want=clamp(c::HOME_PITCH_PHASEB_STIFF_NM_PER_RAD*e_b
          -c::HOME_PITCH_PHASEB_DAMP_NM_PER_RAD_S*f.ps,
          -c::HOME_PITCH_PHASEB_MAX_TORQUE_NM,c::HOME_PITCH_PHASEB_MAX_TORQUE_NM);
        // Ramp the lock torque in (<= 3 Nm/s) so the A->B handoff has no step.
        const float max_step=3.0f*dt;
        phaseb_torque+=clamp(want-phaseb_torque,-max_step,max_step);
        // The 6020 MUST keep its full weak lock: with it released the pitch falls and
        // the M2006 can no longer translate the pivot at all. The resulting rock during
        // the pull-back is a mechanical coupling limit, not a gain problem.
        out.pitch_torque_nm=phaseb_torque;
        // Use the PITCH angle travel, not the M2006 shaft angle: the M2006 drives a
        // lead screw, so its shaft angle is not comparable to the arm angle.
        // Trigger: M2006 relative shaft angle (channel 10) reaches 2.5 rad.
        // One-shot latch; small variation afterwards is allowed.
        if (!m2006_hold_locked) {
          // 5 s lock at the end of phase A (position hold on the M2006).
          if (phase_a_hold) {
            const float e_a=phase_a_hold_angle-f.ma;
            out.m2006_torque_nm=clamp(3.0f*e_a-0.30f*f.ms,
              -c::HOME_M2006_BREAKAWAY_TORQUE_NM,c::HOME_M2006_BREAKAWAY_TORQUE_NM);
            if (now-phase_a_hold_since>=5000U) phase_a_hold=false;
            return;
          }
          // Reached the bottom: stop here and wait 5 s before reversing.
          // The 6020 keeps its phase-B lock during the wait.
          if (!m2006_pullback && out.m2006_relative_rad>=c::HOME_M2006_PULLBACK_RAD) {
            if (!pullback_wait) {
              pullback_wait=true;
              pullback_wait_since=now;
            }
            else if (now-pullback_wait_since>=5000U) {
              m2006_pullback=true;
            }
          }
          if (m2006_pullback) {
            if (out.m2006_relative_rad<=c::HOME_M2006_PULLBACK_STOP_RAD) {
              // Pulled back far enough -> engage the hold (channel 11 goes to 1).
              m2006_hold_locked=true;
              m2006_hold_since=now;
              m2006_hold_angle=f.ma;
              out.m2006_torque_nm=0.0f;
            }
            else {
              out.m2006_torque_nm=-dir*c::HOME_M2006_DRIVE_DIR*c::HOME_M2006_PULLBACK_TORQUE_NM-
                c::HOME_M2006_DAMPING_NM_PER_RAD_S*f.ms;
            }
          }
          else if (pullback_wait) {
            // 5 s dwell at the bottom: no drive, damping only.
            out.m2006_torque_nm=-c::HOME_M2006_DAMPING_NM_PER_RAD_S*f.ms;
          }
          else {
            out.m2006_torque_nm=dir*c::HOME_M2006_DRIVE_DIR*tq-
              c::HOME_M2006_DAMPING_NM_PER_RAD_S*f.ms;
          }
          // Phase B trigger uses the ARM angle: phase A travel + this phase's travel.
          const float arm_travel=dir*(f.pa-pitch_return_origin);
          if (arm_travel>=c::HOME_PITCH_RETURN_MOVE_RAD+c::HOME_M2006_CONTINUE_RAD) {
            m2006_hold_locked=true;
            m2006_hold_since=now;
            m2006_hold_angle=f.ma;
          }
        }
        else {
          // Hold with a small deadband so a little drift is tolerated.
          const float err_hold=m2006_hold_angle-f.ma;
          out.m2006_torque_nm=(std::fabs(err_hold)>0.05f) ?
            clamp(c::HOME_M2006_POSITION_KP*err_hold,
              -c::HOME_M2006_MAX_TORQUE_NM,c::HOME_M2006_MAX_TORQUE_NM) : 0.0f;
          if (now-m2006_hold_since>=5000U) {
            out.stage=Stage::Done;out.homed=true;
            out.pitch_zero_rad=f.pa;out.m2006_zero_rad=f.ma;
            m2006_home_done=true;
            pitch_speed_cmd=0.0f;m_pi={};m_speed_cmd=0.0f;
          }
        }
        out.m2006_locked=m2006_hold_locked;
      }
      if (now-entered>=c::HOME_PITCH_RETURN_TIMEOUT_MS) {
        fault(Fault::MoveTimeout);
      }
      return;
    }
    if (out.stage==Stage::MovePitch || out.stage==Stage::Done) {
      const float delta=f.pa-pitch_return_origin;
      const float target=-c::HOME_PITCH_DIRECTION*c::HOME_PITCH_RETURN_MOVE_RAD;
      out.pitch_return_relative_rad=delta;out.pitch_return_target_relative_rad=target;
      if (out.stage==Stage::MovePitch && now-entered>=c::HOME_PITCH_RETURN_TIMEOUT_MS) {
        // Timeout fallback: if the mechanism already got close to the 30 deg target
        // (mechanical travel may simply be shorter), lock the ACHIEVED position as the
        // zero instead of faulting. Only a large miss is a real failure.
        if (std::fabs(target-delta)<=0.087266463f) {  // 5 deg
          out.stage=Stage::Done;out.homed=true;
          out.pitch_zero_rad=pitch_return_origin+target;
          out.m2006_zero_rad=f.ma;
          pitch_speed_cmd=0.0f;m_pi={};m_speed_cmd=0.0f;
        }
        else {
          fault(Fault::MoveTimeout);
        }
        return;
      }
      // ONLY during the return. In Done the M2006 second-stage rotation drags the 6020
      // through the linkage, so this displacement is expected and must not fault.
      if (out.stage!=Stage::Done &&
          std::fabs(delta)>c::HOME_PITCH_RETURN_MOVE_RAD+c::HOME_PITCH_RETURN_OVERSHOOT_RAD) {
        fault(Fault::MoveOvershoot);return;
      }
      const bool holding=out.stage==Stage::Done;
      if (holding && !c::HOME_HOLD_AFTER_DONE) return;
      // Allow multiple zero-current CAN frames before applying reverse torque.
      if (!holding && now-entered<c::HOME_PITCH_RETURN_RELEASE_MS) return;

      if (holding && !locked) {
        // Stage 4 has been held for HOME_LOCK_MS -> freeze the achieved pose as the zero.
        if (!done_since) done_since=now;
        else if (now-done_since>=c::HOME_LOCK_MS) {
          out.pitch_zero_rad=f.pa;
          out.m2006_zero_rad=f.ma;
          locked=true;
        }
      }
      const float position_goal=holding ? out.pitch_zero_rad : pitch_return_origin+target;
      const float position_kp=holding ? c::HOME_PITCH_HOLD_POSITION_KP : c::HOME_PITCH_RETURN_POSITION_KP;
      const float speed_limit=holding ? c::HOME_PITCH_HOLD_MAX_SPEED_RAD_S : c::HOME_PITCH_RETURN_MAX_SPEED_RAD_S;
      // Deadband: inside 4 deg, command zero speed (pure damping). No position correction,
      // so the loop cannot hunt back and forth around the recorded zero.
      const float err_p=position_goal-f.pa;
      const float dv=c::HOME_TARGET_ACCEL_RAD_S2*dt;
      if (holding) {
        // Single PD controller only. Running the position->speed loop AND the spring
        // together made the two fight each other and vibrate.
        pitch_pi={};
        pitch_speed_cmd=0.0f;
        out.pitch_torque_nm=0.0f;
      }
      else {
        // Backoff: plain constant torque away from the stop. No speed loop -> no vibration.
        out.pitch_torque_nm=-c::HOME_PITCH_DIRECTION*c::HOME_PITCH_RETURN_TORQUE_NM;
      }
      // Locked pose: add a direct position->torque spring so the axis cannot be
      // pushed away. The speed loop alone is soft (velocity-type hold).
      if (holding) {
        const float stiff=0.0f;  // both axes disabled after homing
        out.pitch_torque_nm=clamp(out.pitch_torque_nm+stiff,
          -c::HOME_PITCH_HOLD_MAX_TORQUE_NM,c::HOME_PITCH_HOLD_MAX_TORQUE_NM);
      }

      if (!holding) {
        // Settle test is POSITION ONLY. During the return the M2006 follows the 6020 and
        // the linkage couples them, so both speed readings keep jittering; requiring a
        // low speed never becomes true and the return would always time out.
        // Angle-triggered: once the axis has travelled the full backoff angle, lock.
        // Direction-aware so overshoot cannot re-arm the trigger.
        const float travel=-c::HOME_PITCH_DIRECTION*delta;
        const bool settled=travel>=c::HOME_PITCH_RETURN_MOVE_RAD;
        // First time the position error enters the band -> latch Done immediately.
        // No dwell time, so the controller does not keep making small corrections.
        if (settled) {
          out.stage=Stage::Done;out.homed=true;
          out.pitch_zero_rad=pitch_return_origin+target;out.m2006_zero_rad=f.ma;
          // Keep the GM6020 load integral; capture M2006 ONLY at the final pose.
          pitch_speed_cmd=0.0f;m_pi={};m_speed_cmd=0.0f;
          if (!c::HOME_HOLD_AFTER_DONE) out.pitch_torque_nm=0.0f;
        }
        // M2006 随 6020 转速成比例运动：不锁死（仍可动），但不让重力把
        // 总成拖回挡块。FOLLOW=0 时退化为原来的“零力矩”行为。
        const float follow_target = c::HOME_M2006_RETURN_FOLLOW * f.ps;
        m_speed_cmd += clamp(follow_target - m_speed_cmd, -dv, dv);
        out.m2006_torque_nm = m_pi.step(m_speed_cmd, f.ms, c::HOME_M2006_SPEED_KP,
          c::HOME_M2006_SPEED_KI, c::HOME_M2006_MAX_TORQUE_NM, dt);
        return;
      }

      // ---- M2006 第二段归零 ----
      // 6020 锁定后，M2006 再转 HOME_M2006_FINAL_ROT_RAD（30°），到位后该位置就是最终零点。
      // 周期、限速、PI 都复用 6020 已验证的那套。
      if (locked && !m2006_home_done) {
        if (!m2006_home_armed) {
          m2006_home_armed=true;
          m2006_home_since=now;
          m2006_home_target=f.ma+c::HOME_M2006_FINAL_ROT_RAD*c::HOME_M2006_FINAL_ROT_DIR;
        }
        // Timeout fallback: if the axis cannot reach the full rotation (mechanical travel
        // short, or blocked), accept the achieved position instead of pushing forever.
        if (now-m2006_home_since>=6000U) {
          m2006_home_done=true;
          out.m2006_zero_rad=f.ma;
          return;
        }
        const float err_home=m2006_home_target-f.ma;
        // Constant torque, angle-triggered: no speed loop, no vibration.
        out.m2006_torque_nm=(err_home>0.0f ? 1.0f : -1.0f)*c::HOME_M2006_FINAL_TORQUE_NM;
        if (std::fabs(err_home)<=c::HOME_M2006_FINAL_TOL_RAD) {
          if (!m2006_settle_since) m2006_settle_since=now;
          else if (now-m2006_settle_since>=c::HOME_M2006_LOCK_MS) {
            m2006_home_done=true;
            out.m2006_zero_rad=m2006_home_target;
          }
        }
        else { m2006_settle_since=0; }
        return;  // 6020 保持不受影响（已在上面算完）
      }
      // Deadband: passive inside the band so the two holds stop fighting.
      const float err_m=out.m2006_zero_rad-f.ma;
      // The lead screw cannot self-lock, so the M2006 must keep holding, accepting a
      // small nod from the backlash. Soft spring + wide deadband minimises it.
      m_pi={};
      m_speed_cmd=0.0f;
      (void)err_m;
      out.m2006_torque_nm=0.0f;  // both axes disabled after homing
    }
  }
};
bool submit(uint32_t id,uint8_t * data) {
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0) {++tx_errors;return false;}
  CAN_TxHeaderTypeDef h={};h.StdId=id;h.IDE=CAN_ID_STD;h.RTR=CAN_RTR_DATA;h.DLC=8;
  uint32_t mailbox=0;
  if (HAL_CAN_AddTxMessage(&hcan1,&h,data,&mailbox)!=HAL_OK) {++tx_errors;return false;}
  return true;
}
} // namespace
// Standalone selection; completion never schedules another motor program.
bool selected() {return c::HOMING_ENABLED && !infrared_pitch_auto::selected();}
void start() {taskENTER_CRITICAL();start_requested=true;stop_requested=false;taskEXIT_CRITICAL();}
void cancel() {taskENTER_CRITICAL();finished=false;stop_requested=true;start_requested=false;
  pitch_request=m_request=0.0f;taskEXIT_CRITICAL();}
Status snapshot() {
  taskENTER_CRITICAL();Status s=latest;s.pitch_rx_frames=pitch_rx;s.m2006_rx_frames=m_rx;
  s.tx_errors=tx_errors;taskEXIT_CRITICAL();return s;
}
bool receive_can1(const CAN_RxHeaderTypeDef & h,uint8_t * data,uint32_t now) {
  if (!selected()||!data||h.IDE!=CAN_ID_STD||h.RTR!=CAN_RTR_DATA||h.DLC!=8) return false;
  if (h.StdId==pitch.rx_id) {pitch.read(data,now);pitch_stamp=now;++pitch_rx;return true;}
  if (h.StdId==m2006.rx_id) {m2006.read(data,now);m_stamp=now;++m_rx;return true;}
  return false;
}
void transmit_can1(uint32_t now) {
  if (!selected() || (sent_once && now-last_tx<c::HOME_CAN_INTERVAL_MS)) return;
  taskENTER_CRITICAL();
  const bool live=now-command_stamp<c::HOME_COMMAND_TIMEOUT_MS && !stop_requested;
  const float pt=live?pitch_request:0.0f, mt=live?m_request:0.0f;
  taskEXIT_CRITICAL();
  uint8_t pd[8]={},md[8]={}; // One owner for both groups; yaw and other slots stay zero.
  pitch.cmd(pt);pitch.write(pd);m2006.cmd(mt);m2006.write(md);
  // Independent submissions: a busy group is retried on the next interval.
  submit(pitch.tx_id,pd);submit(m2006.tx_id,md);
  last_tx=now;sent_once=true;
}
void run() {
  Controller ctrl;
  TickType_t wake=xTaskGetTickCount(),previous=wake;
  for (;;) {
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(1));
    const uint32_t now=xTaskGetTickCount();
    const uint32_t elapsed=now-previous;
    if (!elapsed) {wake=now;continue;}
    previous=now;
    taskENTER_CRITICAL();
    const Feedback f={pitch.angle,pitch.speed,pitch.torque,m2006.angle,m2006.speed,m2006.torque,
      pitch.is_open() && m2006.is_open() && now-pitch_stamp<c::HOME_FEEDBACK_TIMEOUT_MS &&
      now-m_stamp<c::HOME_FEEDBACK_TIMEOUT_MS};
    const bool go=start_requested, stop=stop_requested;
    start_requested=false;stop_requested=false;
    taskEXIT_CRITICAL();
    if (stop) ctrl=Controller{};
    else if (go) ctrl.begin(now);
    ctrl.step(f,now,elapsed*0.001f);
    const bool completed=ctrl.out.homed && ctrl.m2006_home_done;
    taskENTER_CRITICAL();
    latest=ctrl.out;pitch_request=ctrl.out.pitch_torque_nm;m_request=ctrl.out.m2006_torque_nm;
    command_stamp=now;
    // Notify completion only; remain in this program and preserve its stop/hold logic.
    if (completed && !finished) {
      finished=true;
      // Signal ONCE: doing this every cycle re-triggered the buzzer forever.
      buzzer_signal_homing_done();
    }
    taskEXIT_CRITICAL();

    // Only this selected program may transmit its motor commands.
    transmit_can1(now);
    if (elapsed>c::HOME_MAX_STEP_MS) wake=now;
  }
}
} // namespace homing
extern "C" void homing_task(void const * argument) {
  (void)argument;
  if (infrared_pitch_auto::selected()) {
    while (!gimbal_can::start()) vTaskDelay(pdMS_TO_TICKS(100));
    infrared_pitch_auto::run();
  }
  if (homing::selected()) {
    while (!gimbal_can::start()) vTaskDelay(pdMS_TO_TICKS(100));
    homing::run();
  }
  for (;;) {vTaskDelay(pdMS_TO_TICKS(100));}
}

