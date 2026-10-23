#include "infrared_pitch_auto.hpp"
#include <cmath>
#include "FreeRTOS.h"
#include "task.h"
#include "control_constants.hpp"
#include "gimbal_can.hpp"
#include "gimbal_imu.hpp"
#include "infrared_status.hpp"
#include "pitch_start.hpp"
#include "trajectory_follow.hpp"
#include "shared_data.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// Adapted from the teammate's IR homing: rear stop -> default pose -> IR ->
// descent -> 5s dwell -> fold back. Its terminal timed release is replaced by
// an encoder-relative 0.8rad hold. Current pitch_start and IMU tracking are reused.
namespace infrared_pitch_auto {
namespace {
namespace c=control_constants;
constexpr float absf(float x) { return x<0.0f ? -x : x; }
constexpr float clamp(float x,float lo,float hi) { return x<lo ? lo : (x>hi ? hi : x); }
constexpr bool handoff_ready(uint32_t infrared_ms,bool triggered,bool stable) {
  return triggered && stable && infrared_ms>=c::INFRARED_AUTO_PREPARE_DELAY_MS;
}
constexpr bool follow_expired(uint32_t now,uint32_t start) {
  return now-start>=c::INFRARED_AUTO_FOLLOW_MS;
}
constexpr HoldOutput weak_hold(float ty,float tp,float ya,float pa,float ys,float ps,float gravity) {
  return {ty,tp,
    clamp(c::INFRARED_AUTO_WEAK_YAW_KP*(ty-ya)-c::INFRARED_AUTO_WEAK_YAW_KD*ys,
      -c::INFRARED_AUTO_WEAK_YAW_MAX_NM,c::INFRARED_AUTO_WEAK_YAW_MAX_NM),
    clamp(c::INFRARED_AUTO_WEAK_PITCH_KP*(tp-pa)-c::INFRARED_AUTO_WEAK_PITCH_KD*ps+gravity,
      -c::INFRARED_AUTO_WEAK_PITCH_MAX_NM,c::INFRARED_AUTO_WEAK_PITCH_MAX_NM)};
}
struct PI {
  float integral=0;
  constexpr float step(float error,float kp,float ki,float limit,float dt) {
    const float inc=ki*error*dt;
    const float candidate=clamp(integral+inc,-limit,limit);
    const float raw=kp*error+candidate;
    if (!((raw>limit && inc>0) || (raw<-limit && inc<0))) integral=candidate;
    return clamp(kp*error+integral,-limit,limit);
  }
};
struct Input {
  float pa=0,ps=0,ma=0,ms=0;
  bool fresh=false, imu_ready=false, infrared_valid=false, triggered=false;
  float distance_cm=0;
  uint32_t infrared_ms=0;
};
struct Controller {
  Stage stage=Stage::WaitImu;
  Fault fault=Fault::None;
  PI pitch_pi,fold_pi;
  uint32_t entered=0,stall_since=0,settle_since=0,pitch_os_since=0,m_os_since=0;
  bool started=false,motion_seen=false,stall=false,settling=false,fold_stable=false;
  float pitch_start=0,stall_angle=0,pitch_origin=0,m_origin=0,pitch_lock=0;
  float pitch_speed_cmd=0,phaseb_torque=0,pt=0,mt=0;
  float observed_pitch_speed=0,observed_m_speed=0,fault_pitch_speed=0,fault_m_speed=0;
  Stage fault_stage=Stage::WaitImu;

  constexpr void fail(Fault code) {
    if (stage==Stage::Fault) return;  // Keep the FIRST failure and its motion evidence.
    fault_stage=stage;fault_pitch_speed=observed_pitch_speed;fault_m_speed=observed_m_speed;
    fault=code;stage=Stage::Fault;pt=mt=0;
  }
  constexpr void enter(Stage next,uint32_t now) { stage=next;entered=now; }
  constexpr void hold_pitch(const Input & f,float dt,bool slew) {
    const float want=clamp(c::HOME_PITCH_PHASEB_STIFF_NM_PER_RAD*(pitch_lock-f.pa)-
      c::HOME_PITCH_PHASEB_DAMP_NM_PER_RAD_S*f.ps,
      -c::HOME_PITCH_PHASEB_MAX_TORQUE_NM,c::HOME_PITCH_PHASEB_MAX_TORQUE_NM);
    phaseb_torque=slew ? phaseb_torque+clamp(want-phaseb_torque,-3.0f*dt,3.0f*dt) : want;
    pt=phaseb_torque;
  }
  constexpr void step(const Input & f,uint32_t now,float dt) {
    pt=mt=0;
    if (stage==Stage::Fault || stage==Stage::PitchStart) return;
    observed_pitch_speed=f.ps;observed_m_speed=f.ms;
    if (stage==Stage::WaitImu) {
      if (!f.imu_ready) return;  // Keep all motors quiet during the existing static calibration.
      enter(Stage::WaitFeedback,now);
    }
    if (stage==Stage::WaitFeedback) {
      if (!f.fresh) {
        if (now-entered>=c::HOME_READY_TIMEOUT_MS) fail(Fault::Feedback);
        return;
      }
      pitch_start=f.pa;pitch_origin=f.pa;m_origin=f.ma;started=true;
      enter(Stage::SeekPitch,now);
    }
    if (!f.fresh) { fail(Fault::Feedback);return; }
    if (!f.imu_ready) { fail(Fault::Imu);return; }
    if (dt<=0 || dt>c::HOME_MAX_STEP_MS*.001f) { fail(Fault::Timing);return; }
    if (absf(f.ps)>c::HOME_OVERSPEED_RAD_S) {
      if (!pitch_os_since) pitch_os_since=now;
    } else pitch_os_since=0;
    if (absf(f.ms)>c::HOME_M2006_OVERSPEED_RAD_S) {
      if (!m_os_since) m_os_since=now;
    } else m_os_since=0;
    if ((pitch_os_since && now-pitch_os_since>=50U) ||
        (m_os_since && now-m_os_since>=50U)) { fail(Fault::Overspeed);return; }
    if (stage==Stage::SeekPitch) {
      if (c::HOME_PITCH_DIRECTION*(f.pa-pitch_start)>=c::HOME_PITCH_MIN_SEEK_TRAVEL_RAD)
        motion_seen=true;
      if (now-entered>=c::HOME_PITCH_TIMEOUT_MS) {
        fail(motion_seen ? Fault::SeekTimeout : Fault::NoSeekMotion);return;
      }
      if (absf(f.pa-pitch_start)>c::HOME_PITCH_MAX_TRAVEL_RAD) { fail(Fault::SeekTravel);return; }
      const float dv=c::HOME_TARGET_ACCEL_RAD_S2*dt;
      pitch_speed_cmd+=clamp(c::HOME_PITCH_DIRECTION*c::HOME_PITCH_SEEK_SPEED_RAD_S-pitch_speed_cmd,-dv,dv);
      pt=pitch_pi.step(pitch_speed_cmd-f.ps,c::HOME_PITCH_SPEED_KP,c::HOME_PITCH_SPEED_KI,
        c::HOME_PITCH_MAX_TORQUE_NM,dt);
      const bool candidate=motion_seen && now-entered>=c::HOME_PITCH_STARTUP_BLANK_MS &&
        c::HOME_PITCH_DIRECTION*pt>=c::HOME_PITCH_STALL_TORQUE_NM;
      if (!candidate) stall=false;
      else {
        if (!stall || absf(f.pa-stall_angle)>c::HOME_PITCH_STALL_TRAVEL_RAD) {
          stall=true;stall_since=now;stall_angle=f.pa;
        }
        if (now-stall_since>=c::HOME_PITCH_STALL_MS) {
          pt=0;pitch_pi={};pitch_speed_cmd=0;pitch_origin=f.pa;m_origin=f.ma;
          enter(Stage::MoveDefault,now);
        }
      }
      return;
    }
    const float dir=-c::HOME_PITCH_DIRECTION;
    const float drive=(absf(f.ms)<c::HOME_M2006_BREAKAWAY_SPEED_RAD_S) ?
      c::HOME_M2006_BREAKAWAY_TORQUE_NM : c::HOME_M2006_CRUISE_TORQUE_NM;
    if (stage==Stage::MoveDefault) {
      mt=dir*c::HOME_M2006_DRIVE_DIR*drive-c::HOME_M2006_DAMPING_NM_PER_RAD_S*f.ms;
      if (dir*(f.pa-pitch_origin)>=c::INFRARED_AUTO_DEFAULT_TRAVEL_RAD) {
        pitch_lock=f.pa;phaseb_torque=0;mt=0;enter(Stage::WaitInfrared,now);
      } else if (now-entered>=c::HOME_PITCH_RETURN_TIMEOUT_MS) fail(Fault::MoveTimeout);
      return;
    }
    if (stage==Stage::WaitInfrared) {
      hold_pitch(f,dt,false);
      if (f.infrared_valid && f.distance_cm<=c::INFRARED_AUTO_TRIGGER_CM) enter(Stage::Descent,now);
      return;
    }
    hold_pitch(f,dt,true);
    if (stage==Stage::Descent) {
      mt=dir*c::HOME_M2006_DRIVE_DIR*drive-c::HOME_M2006_DAMPING_NM_PER_RAD_S*f.ms;
      if (f.ma-m_origin>=c::HOME_M2006_PULLBACK_RAD) {
        enter(Stage::DescentHold,now);
        mt=-c::HOME_M2006_DAMPING_NM_PER_RAD_S*f.ms;
      } else if (now-entered>=c::HOME_PITCH_RETURN_TIMEOUT_MS) fail(Fault::MoveTimeout);
      return;
    }
    if (stage==Stage::DescentHold) {
      mt=-c::HOME_M2006_DAMPING_NM_PER_RAD_S*f.ms;
      if (now-entered>=c::INFRARED_AUTO_DESCENT_HOLD_MS) { fold_pi={};settling=false;enter(Stage::FoldBack,now); }
      return;
    }
    if (stage==Stage::FoldBack) {
      if (f.ma-m_origin>c::INFRARED_AUTO_FOLD_ENCODER_RAD) {
        // Copy the teammate's tested return law exactly: oppose velocity even
        // far from the endpoint. Do NOT replace it with saturated position P.
        mt=-dir*c::HOME_M2006_DRIVE_DIR*c::HOME_M2006_PULLBACK_TORQUE_NM-
          c::HOME_M2006_DAMPING_NM_PER_RAD_S*f.ms;
        if (now-entered>=c::INFRARED_AUTO_FOLD_TIMEOUT_MS) fail(Fault::MoveTimeout);
        return;
      }
      // Only the ending rule differs from teammate homing: 0.8rad instead of
      // a fixed five-second return and zero-current release.
      fold_pi={};settling=false;fold_stable=false;enter(Stage::FoldHold,now);
    }
    if (stage==Stage::FoldHold) {
      const float error=c::INFRARED_AUTO_FOLD_ENCODER_RAD-(f.ma-m_origin);
      // Hold the fixed 0.8rad goal; NEVER recapture an overshoot or sagging pose.
      const float corrected=error>0.002f ? error-0.002f : (error<-.002f ? error+.002f : 0.0f);
      const float kp=c::INFRARED_AUTO_FOLD_KP,kd=c::INFRARED_AUTO_FOLD_KD;
      const float base=kp*corrected-kd*f.ms;
      const float inc=absf(f.ms)<=0.10f ? c::INFRARED_AUTO_FOLD_KI*corrected*dt : 0.0f;
      const float limit=c::HOME_M2006_MAX_TORQUE_NM;
      const float candidate=clamp(fold_pi.integral+inc,-limit,limit);
      const float raw=base+candidate;
      if (!((raw>limit && inc>0) || (raw<-limit && inc<0))) fold_pi.integral=candidate;
      mt=clamp(base+fold_pi.integral,-limit,limit);
      const bool quiet=absf(error)<=c::INFRARED_AUTO_FOLD_TOL_RAD &&
        absf(f.ms)<=c::INFRARED_AUTO_FOLD_QUIET_RAD_S;
      if (!quiet) { settling=false;fold_stable=false; }
      else {
        if (!settling) { settling=true;settle_since=now; }
        fold_stable=now-settle_since>=c::INFRARED_AUTO_FOLD_SETTLE_MS;
      }
      if (handoff_ready(f.infrared_ms,f.triggered,fold_stable)) { stage=Stage::PitchStart;return; }
      if (!fold_stable && now-entered>=c::INFRARED_AUTO_FOLD_TIMEOUT_MS)
        fail(Fault::MoveTimeout);
    }
  }
};

sp::RM_Motor pitch(c::HOME_PITCH_ID,sp::RM_Motors::GM6020,c::HOME_PITCH_RATIO);
sp::RM_Motor m2006(c::HOME_M2006_ID,sp::RM_Motors::M2006,c::HOME_M2006_RATIO);
sp::RM_Motor yaw(c::FOLLOW_YAW_MOTOR_ID,sp::RM_Motors::GM6020,c::FOLLOW_YAW_RATIO);
uint32_t pitch_stamp=0,m_stamp=0,yaw_stamp=0,last_tx=0;
bool sent=false,handed_over=false,tracking_started=false,following_fault=false;
bool holding=false;
uint32_t tracking_start=0;
float yaw_lock=0,hold_yaw=0,hold_pitch=0;
bool yaw_lock_captured=false;
Status latest={};

bool submit(uint32_t id,uint8_t * data) {
  taskENTER_CRITICAL();
  bool ok=false;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0U) {
    CAN_TxHeaderTypeDef h={};uint32_t mailbox=0;
    h.StdId=id;h.IDE=CAN_ID_STD;h.RTR=CAN_RTR_DATA;h.DLC=8;
    ok=HAL_CAN_AddTxMessage(&hcan1,&h,data,&mailbox)==HAL_OK;
  }
  taskEXIT_CRITICAL();return ok;
}
void transmit(float yt,float pt,float mt,uint32_t now) {
  if (sent && now-last_tx<c::HOME_CAN_INTERVAL_MS) return;
  uint8_t pd[8]={},md[8]={};
  yaw.cmd(yt);yaw.write(pd);pitch.cmd(pt);pitch.write(pd);m2006.cmd(mt);m2006.write(md);
  submit(pitch.tx_id,pd);submit(m2006.tx_id,md);
  last_tx=now;sent=true;
}
}

bool selected() { return c::INFRARED_PITCH_AUTO_ENABLE; }
bool preparation_active() {
  if (!selected()) return true;
  taskENTER_CRITICAL();const bool value=handed_over;taskEXIT_CRITICAL();return value;
}
void receive_can1(const CAN_RxHeaderTypeDef & h,uint8_t * data,uint32_t now) {
  if (!selected() || !data || h.IDE!=CAN_ID_STD || h.RTR!=CAN_RTR_DATA || h.DLC!=8) return;
  // Observe without consuming: existing pitch_start/follower keep their own
  // continuous encoder state, feedback timestamps and counters through handoff.
  if (h.StdId==pitch.rx_id) { pitch.read(data,now);pitch_stamp=now; }
  if (h.StdId==m2006.rx_id) { m2006.read(data,now);m_stamp=now; }
  if (h.StdId==yaw.rx_id) { yaw.read(data,now);yaw_stamp=now; }
}
void snapshot(Status & out) { taskENTER_CRITICAL();out=latest;taskEXIT_CRITICAL(); }
bool follow_stop_due(uint32_t now) {
  if (!selected()) return false;
  taskENTER_CRITICAL();
  if (!tracking_started) { tracking_started=true;tracking_start=now; }
  const bool due=follow_expired(now,tracking_start);
  taskEXIT_CRITICAL();return due;
}
HoldOutput hold_follow_pose(float ya,float pa,float ys,float ps) {
  // Fixed IMU targets captured ONCE at the 5s deadline. No recapture on drift.
  taskENTER_CRITICAL();
  if (!holding) { holding=true;hold_yaw=ya;hold_pitch=pa; }
  const float ty=hold_yaw,tp=hold_pitch;
  taskEXIT_CRITICAL();
  return weak_hold(ty,tp,ya,pa,ys,ps,c::FOLLOW_PITCH_GRAVITY_NM*std::cos(pa));
}
void report_follow_fault() { taskENTER_CRITICAL();following_fault=true;taskEXIT_CRITICAL(); }

void run() {
  static_assert(configTICK_RATE_HZ==1000,"IR sequence needs 1ms ticks");
  static Controller ctrl;
  static gimbal_imu::State imu;
  static pitch_start::Status prepared;
  static trajectory_follow::SequenceDiagnostics follow;
  TickType_t wake=xTaskGetTickCount(),previous=wake;
  for (;;) {
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(1));
    const uint32_t now=xTaskGetTickCount(),elapsed=now-previous;
    if (!elapsed) continue;
    previous=now;
    gimbal_imu::read(imu);
    const auto ir=infrared::snapshot();
    taskENTER_CRITICAL();
    const uint32_t current=xTaskGetTickCount();
    Input f={pitch.angle,pitch.speed,m2006.angle,m2006.speed,
      pitch.is_open() && m2006.is_open() && yaw.is_open() &&
      current-pitch_stamp<c::HOME_FEEDBACK_TIMEOUT_MS && current-m_stamp<c::HOME_FEEDBACK_TIMEOUT_MS &&
      current-yaw_stamp<c::HOME_FEEDBACK_TIMEOUT_MS,imu.ready,ir.valid,ir.timer_started,
      ir.distance_cm,g_data.infrared_timer_ms};
    const bool delegated=handed_over;
    const bool was_tracking=tracking_started;
    const bool stopped=holding;
    const bool follow_failed=following_fault;
    const uint32_t follow_start=tracking_start;
    const float ya=yaw.angle,ys=yaw.speed,mt=m2006.torque;
    taskEXIT_CRITICAL();
    float yt=0.0f;
    if (!delegated) {
      if (!std::isfinite(f.pa) || !std::isfinite(f.ps) || !std::isfinite(f.ma) ||
          !std::isfinite(f.ms) || !std::isfinite(ya) || !std::isfinite(ys))
        ctrl.fail(Fault::Feedback);
      ctrl.step(f,now,elapsed*.001f);
      if (ctrl.started && !yaw_lock_captured) { yaw_lock=ya;yaw_lock_captured=true; }
      if (ctrl.stage!=Stage::Fault && ctrl.started && yaw_lock_captured)
        yt=clamp(c::INFRARED_AUTO_YAW_HOLD_KP*(yaw_lock-ya)-c::INFRARED_AUTO_YAW_HOLD_KD*ys,
          -c::INFRARED_AUTO_YAW_HOLD_MAX_NM,c::INFRARED_AUTO_YAW_HOLD_MAX_NM);
      if (ctrl.stage==Stage::PitchStart) {
        // No further legacy frames are sent once this irreversible gate opens.
        // start() only posts a request; pitch_start executes it in its own task.
        pitch_start::start();
        taskENTER_CRITICAL();handed_over=true;taskEXIT_CRITICAL();
      } else transmit(yt,ctrl.pt,ctrl.mt,now);
    }
    Stage visible=ctrl.stage;Fault code=ctrl.fault;
    if (delegated) {
      pitch_start::snapshot(prepared);
      trajectory_follow::read_sequence_diagnostics(follow);
      visible=stopped ? Stage::Holding : (was_tracking ? Stage::Tracking : Stage::PitchStart);
      if (prepared.stage==pitch_start::Stage::Fault) { visible=Stage::Fault;code=Fault::Preparation; }
      else if (follow_failed || follow.fault) { visible=Stage::Fault;code=Fault::Following; }
    }
    taskENTER_CRITICAL();
    const uint32_t follow_age=was_tracking ? now-follow_start : 0U;
    latest={visible,code,now,f.infrared_ms,
      stopped && follow_age>c::INFRARED_AUTO_FOLLOW_MS ? c::INFRARED_AUTO_FOLLOW_MS : follow_age,
      f.ma-ctrl.m_origin,c::INFRARED_AUTO_FOLD_ENCODER_RAD,
      delegated ? prepared.m2006_torque_nm : ctrl.mt,mt,delegated ? prepared.pitch_torque_nm : ctrl.pt,
      yt,ir.distance_cm,imu.ready ? 1.0f :
        clamp(static_cast<float>(imu.calibration_elapsed_ms)/c::IMU_CALIBRATION_MS,0.0f,1.0f),
      f.triggered,ctrl.fold_stable,handed_over,
      ctrl.fault_pitch_speed,ctrl.fault_m_speed,static_cast<uint8_t>(ctrl.fault_stage)};
    taskEXIT_CRITICAL();
    if (elapsed>c::HOME_MAX_STEP_MS) wake=now;
  }
}
}
