#include <limits>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "control_constants.hpp"
#include "grading.hpp"
#include "gimbal_imu.hpp"
#include "homing.hpp"
#include "pitch_start.hpp"
#include "infrared_status.hpp"
#include "io/plotter/plotter.hpp"
#include "task.h"
#include "trajectory_follow.hpp"
#include "trajectory_task.hpp"
#include "usart.h"

// USART1 / physical 4-pin UART2: AA BB + byte length + little-endian floats.
// Yaw/pitch test: target angle (rad), actual IMU angle (rad), score (0..1); 3 floats.
// Infrared mode, when explicitly enabled: distance and reception diagnostics.
// Other plot modes below are inactive while INFRARED_PLOT_TEST is true.

sp::Plotter plotter(&huart1, false);

// 串口用角度（°）显示，方便读曲线。
static constexpr float kRadToDeg = 57.29577951f;

// 底盘诊断快照（定义在 chassis_task.cpp）：0=FL 1=FR 2=RL 3=RR

extern "C" void plot_task()
{
  uint32_t last_wake = osKernelSysTick();
  for (;;) {
    osDelayUntil(&last_wake, 20);
    if (control_constants::FOLLOW_BOTH_ENABLE) {
      const auto score = grading::read();
      // First six channels share one trajectory phase; optional diagnostics follow.
      if (control_constants::FOLLOW_BOTH_RUNTIME_DIAGNOSTICS) {
        static trajectory_follow::SequenceDiagnostics trace;
        static pitch_start::CanDiagnostics bus;
        trajectory_follow::read_sequence_diagnostics(trace);
        pitch_start::read_can_diagnostics(bus);
        static gimbal_imu::State startup_imu;
        static pitch_start::Status startup_preparation;
        gimbal_imu::read(startup_imu);
        if (control_constants::IMU_STARTUP_DIAGNOSTICS && !startup_imu.aligned) {
          // Same 18-float frame. Ch7=-1 identifies the startup-only layout.
          // Keep it visible through calibration and preparation faults, until
          // the follower captures its IMU origin before the 3-degree prelift.
          pitch_start::snapshot(startup_preparation);
          static trajectory_follow::TransportStatus startup_transport;
          static uint32_t previous_rx[3] = {};
          static float rx_frames[3] = {};
          startup_transport = trajectory_follow::read_transport();
          rx_frames[0] = static_cast<float>(startup_preparation.pitch_rx_frames - previous_rx[0]);
          rx_frames[1] = static_cast<float>(startup_preparation.m2006_rx_frames - previous_rx[1]);
          rx_frames[2] = static_cast<float>(startup_transport.yaw_frames - previous_rx[2]);
          previous_rx[0] = startup_preparation.pitch_rx_frames;
          previous_rx[1] = startup_preparation.m2006_rx_frames;
          previous_rx[2] = startup_transport.yaw_frames;
          if (startup_imu.phase == gimbal_imu::Phase::Ready &&
              (startup_preparation.stage == pitch_start::Stage::WaitFeedback ||
               (startup_preparation.stage == pitch_start::Stage::Fault &&
                startup_preparation.fault == pitch_start::Fault::Feedback))) {
            // Ch7=-2: calibration passed, diagnose the exact startup gates.
            // 10 IMU fresh, 11/12/13 pitch/M2006/yaw RX per plot interval,
            // 14 missing-input mask, 15/16 IMU/controller age, 17 stage, 18 fault.
            const uint32_t diagnostic_now = xTaskGetTickCount();
            plotter.plot((score.target_yaw_rad * kRadToDeg), (score.actual_yaw_rad * kRadToDeg),
              (score.target_pitch_rad * kRadToDeg), (score.actual_pitch_rad * kRadToDeg), score.yaw_ratio, score.pitch_ratio,
              -2.0f, static_cast<float>(startup_imu.phase), static_cast<float>(startup_imu.fault),
              startup_imu.ready ? 1.0f : 0.0f, rx_frames[0], rx_frames[1], rx_frames[2],
              static_cast<float>(bus.waiting_inputs),
              static_cast<float>(diagnostic_now - startup_imu.stamp_ms),
              static_cast<float>(diagnostic_now - startup_preparation.stamp_ms),
              static_cast<float>(startup_preparation.stage), static_cast<float>(startup_preparation.fault));
            continue;
          }
          float progress = static_cast<float>(startup_imu.calibration_elapsed_ms) /
            control_constants::IMU_CALIBRATION_MS;
          const float samples = static_cast<float>(startup_imu.calibration_samples) /
            control_constants::IMU_CALIBRATION_MIN_SAMPLES;
          if (samples < progress) progress = samples;
          if (progress > 1.0f || startup_imu.ready) progress = 1.0f;
          constexpr float degrees = 57.2957795f;
          plotter.plot((score.target_yaw_rad * kRadToDeg), (score.actual_yaw_rad * kRadToDeg),
            (score.target_pitch_rad * kRadToDeg), (score.actual_pitch_rad * kRadToDeg), score.yaw_ratio, score.pitch_ratio,
            -1.0f, static_cast<float>(startup_imu.phase), static_cast<float>(startup_imu.fault),
            progress, startup_imu.max_raw_gyro_rad_s * degrees,
            static_cast<float>(startup_imu.calibration_restart_reason),
            startup_imu.calibration_std_rad_s * degrees,
            startup_imu.calibration_rejected_std_rad_s * degrees,
            startup_imu.acceleration_norm_mps2, static_cast<float>(startup_imu.calibration_restarts),
            static_cast<float>(startup_preparation.stage), static_cast<float>(startup_preparation.fault));
          continue;
        }
        // First six channels stay identical. Fault data 8..12 freezes on the
        // FIRST failure, before yaw is zeroed by the stop/hold handler.
        // 15..18 also freeze: motor travel, motor speed, feedback torque, IMU travel.
        // 7 phase, 8 fault, 9 cause, 10 yaw error, 11 gyro yaw rate,
        // 12 last queued yaw torque, 13 max control dt, 14 max GM queue gap.
        plotter.plot((score.target_yaw_rad * kRadToDeg), (score.actual_yaw_rad * kRadToDeg),
          (score.target_pitch_rad * kRadToDeg), (score.actual_pitch_rad * kRadToDeg), score.yaw_ratio, score.pitch_ratio,
          static_cast<float>(trace.phase), static_cast<float>(trace.fault),
          static_cast<float>(trace.cause), trace.yaw_error_rad, trace.yaw_speed_rad_s,
          trace.yaw_torque_nm, static_cast<float>(trace.max_step_ms), static_cast<float>(bus.max_gap_ms),
          trace.yaw_motor_travel_rad, trace.yaw_motor_speed_rad_s,
          trace.yaw_feedback_torque_nm, trace.yaw_imu_travel_rad);
      } else {
        plotter.plot((score.target_yaw_rad * kRadToDeg), (score.actual_yaw_rad * kRadToDeg),
          (score.target_pitch_rad * kRadToDeg), (score.actual_pitch_rad * kRadToDeg), score.yaw_ratio, score.pitch_ratio);
      }
    }
    else if (control_constants::FOLLOW_PITCH_SEQUENCE_ENABLED) {
      const auto score = grading::read();
      if (control_constants::FOLLOW_PITCH_SEQUENCE_DIAGNOSTICS) {
        const auto state = trajectory_follow::read();
        // 4: 0 waiting / 1 prelift / 2 tracking / 3 fault. 5: FAULT_*.
        plotter.plot((score.target_pitch_rad * kRadToDeg), (score.actual_pitch_rad * kRadToDeg), score.pitch_ratio,
          static_cast<float>(state.pitch_sequence_phase), static_cast<float>(state.fault));
      } else {
        plotter.plot((score.target_pitch_rad * kRadToDeg), (score.actual_pitch_rad * kRadToDeg), score.pitch_ratio);
      }
    }
    else if (homing::selected() || pitch_start::selected()) {
      // 两个模块的 Status 类型不同，用共同字段手工取值。
      const bool use_home = homing::selected();
      const auto hs = homing::snapshot();
      const auto ps = pitch_start::snapshot();
      const float picked_stage = use_home ? static_cast<float>(hs.stage) : static_cast<float>(ps.stage);
      const float picked_pitch = use_home ? hs.pitch_angle_rad : ps.pitch_angle_rad;
      const float picked_target =
        use_home ? hs.pitch_return_target_relative_rad : ps.pitch_return_target_relative_rad;
      const float picked_rel =
        use_home ? hs.pitch_return_relative_rad : ps.pitch_return_relative_rad;
      const float picked_fault = use_home ? static_cast<float>(hs.fault) : static_cast<float>(ps.fault);
      const float picked_homed = (use_home ? hs.homed : ps.homed) ? 1.0f : 0.0f;
      const float picked_pitch_tq = use_home ? hs.pitch_torque_nm : ps.pitch_torque_nm;
      const float picked_m_tq = use_home ? hs.m2006_torque_nm : ps.m2006_torque_nm;
      const float picked_stall = static_cast<float>(use_home ? hs.stall_elapsed_ms : ps.stall_elapsed_ms);
      const float picked_mrel = use_home ? hs.m2006_relative_rad : ps.m2006_relative_rad;
      const float picked_mlock = (use_home ? hs.m2006_locked : ps.m2006_locked) ? 1.0f : 0.0f;
      const float picked_mfb_tq =
        use_home ? hs.m2006_feedback_torque_nm : ps.m2006_feedback_torque_nm;
      // 1 phase, 2 GM6020 absolute rad, 3/4 GM6020 return target/actual rad,
      // 5 fault, 6 done; optional 7/8 command Nm and 9/10 feedback Nm (GM6020/M2006).
      if (control_constants::HOME_PLOT_TORQUES) {
        // 10 通道：1 stage 2 绝对角 3/4 回退目标/实际 5 fault 6 homed
        //          7 6020指令力矩 8 M2006指令力矩
        //          9 stall_elapsed_ms（顶挡块计时，0→300 说明判定成功）
        //          10 M2006反馈力矩
        plotter.plot(
          picked_stage, picked_pitch,
          picked_target, picked_rel,
          picked_fault, picked_homed, picked_pitch_tq,
          picked_m_tq, picked_stall, picked_mrel,
          picked_mlock,
        picked_mfb_tq);  // ch10 angle, ch11 locked, ch12 M2006 feedback torque
      }
      else {
        // 8 通道：1 stage 2 绝对角 3/4 回退目标/实际 5 fault 6 homed
        //         7 stall_elapsed_ms（顶挡块计时，0→300 说明判定成功）
        //         8 6020 指令力矩（顶住时应达到 0.40）
        plotter.plot(
          picked_stage, picked_pitch,
          picked_target, picked_rel,
          picked_fault, picked_homed,
          picked_stall, picked_pitch_tq);
      }
    }
    else if (control_constants::IMU_STARTUP_DIAGNOSTICS) {
      const auto score = grading::read();
      const auto imu = gimbal_imu::read();
      const auto bus = trajectory_follow::read_transport();
      static uint32_t previous_yaw_frames = 0;
      float progress = static_cast<float>(imu.calibration_elapsed_ms) /
                       control_constants::IMU_CALIBRATION_MS;
      const float sample_progress = static_cast<float>(imu.calibration_samples) /
                                    control_constants::IMU_CALIBRATION_MIN_SAMPLES;
      if (sample_progress < progress) progress = sample_progress;
      if (progress > 1.0f) progress = 1.0f;
      if (imu.ready) progress = 1.0f;
      // First three channels are unchanged. Remaining channels diagnose startup.
      plotter.plot(
        (score.target_yaw_rad * kRadToDeg), (score.actual_yaw_rad * kRadToDeg), score.yaw_ratio,
        static_cast<float>(imu.phase), static_cast<float>(imu.fault),
        static_cast<float>(imu.init_error), progress,
        static_cast<float>(bus.yaw_frames - previous_yaw_frames),
        static_cast<float>(score.follow_fault), imu.acceleration_norm_mps2,
        imu.max_raw_gyro_rad_s * 57.2957795f,
        static_cast<float>(imu.calibration_restart_reason));
      previous_yaw_frames = bus.yaw_frames;
    }
    else if (control_constants::INFRARED_PLOT_TEST) {
      static uint32_t previous_bytes = 0, previous_frames = 0;
      const auto status = infrared::snapshot();
      const float missing = std::numeric_limits<float>::quiet_NaN();
      const float bytes = static_cast<float>(status.received_bytes - previous_bytes);
      const float frames = static_cast<float>(status.valid_frames - previous_frames);
      if (control_constants::INFRARED_PLOT_RAW_FIELDS) {
        // Append ONLY fields from the last checksum-correct frame. No smoothing.
        plotter.plot(
          status.distance_cm, status.valid ? 1.0f : 0.0f, bytes, frames,
          static_cast<float>(status.raw_distance), static_cast<float>(status.payload_byte2),
          static_cast<float>(status.payload_byte3), static_cast<float>(status.last_payload_length));
      }
      else {
        plotter.plot(
          status.valid ? status.distance_cm : missing, status.valid ? 1.0f : 0.0f, bytes, frames);
      }
      previous_bytes = status.received_bytes;
      previous_frames = status.valid_frames;
    }
    else if (control_constants::FOLLOW_CONSTANT_TORQUE_TEST) {
      const auto state = trajectory_follow::read();
      // 恒力矩测试（当前用于方向符号测试）：
      //   通道 1：实际角度（rad）—— 正力矩下应该增大；若减小说明 FOLLOW_YAW_DIRECTION 反了
      //   通道 2：实际转速（rad/s）—— 用来判断电机是否真的在转（不转=力矩不够或卡住）
      //   通道 3：输出力矩（N·m）
      plotter.plot(
        state.yaw.actual_angle_rad, state.yaw.actual_speed_rad_s, state.yaw.control_output);
    }

    else if (control_constants::FOLLOW_PLOT_MODE == 2U) {
      const TrajectoryPoint target = trajectory_read();
      plotter.plot(target.yaw, target.pitch);
    }

    else if (control_constants::FOLLOW_PLOT_MODE == 1U) {
      const auto state = trajectory_follow::read();
      if (control_constants::FOLLOW_SINGLE_YAW) {
        if (control_constants::FOLLOW_YAW_DIAGNOSTICS) {
          static uint32_t last_yaw_frames = 0, last_tx_frames = 0;
          const auto bus = trajectory_follow::read_transport();
          // TX counts HAL mailbox submissions, not verified motor actuation.
          plotter.plot(
            state.yaw.target_angle_rad, state.yaw.actual_angle_rad, state.yaw.control_output,
            state.active ? 1.0f : 0.0f, state.feedback_fresh ? 1.0f : 0.0f,
            static_cast<float>(state.fault), static_cast<float>(bus.yaw_frames - last_yaw_frames),
            static_cast<float>(bus.tx_frames - last_tx_frames));
          last_yaw_frames = bus.yaw_frames;
          last_tx_frames = bus.tx_frames;
        }
        else {
          // Same aligned IMU values as scoring; NaN while not ready/aligned.
          const auto score = grading::read();
          plotter.plot(
            (score.target_yaw_rad * kRadToDeg), (score.actual_yaw_rad * kRadToDeg), score.yaw_ratio);
        }
      }
      else {
        // 模式 1：按 FOLLOW_PLOT_AXIS 选择画哪根轴（0 = yaw，1 = pitch）
        if (control_constants::FOLLOW_YAW_DIAGNOSTICS) {
          // 双轴反馈诊断（4 通道）：
          //   1 = pitch 实际角   2 = pitch 目标角
          //   3 = pitch 反馈帧数（每 20 ms）  4 = yaw 反馈帧数（每 20 ms）
          // 判读：帧数 ~15-20 = 在线；帧数为 0 = 该电机没有反馈（ID/接线/开关错）
          static uint32_t last_yaw_frames = 0, last_pitch_frames = 0;
          const auto bus = trajectory_follow::read_transport();
          plotter.plot(
            state.pitch.actual_angle_rad, state.pitch.target_angle_rad,
            static_cast<float>(bus.pitch_frames - last_pitch_frames),
            static_cast<float>(bus.yaw_frames - last_yaw_frames));
          last_yaw_frames = bus.yaw_frames;
          last_pitch_frames = bus.pitch_frames;
        }
        else if (control_constants::FOLLOW_PLOT_AXIS == 1U) {
          const auto score = grading::read();
          plotter.plot((score.target_pitch_rad * kRadToDeg), (score.actual_pitch_rad * kRadToDeg));
        }
        else {
          // 双轴跟随：4 通道
          //   1 = yaw 目标   2 = yaw 实际
          //   3 = pitch 目标 4 = pitch 实际
          const auto score = grading::read();
          // 4 通道（单位：度 °）
          //   1 = yaw 目标   2 = yaw 实际
          //   3 = pitch 目标 4 = pitch 实际
          plotter.plot(
            score.target_yaw_rad * kRadToDeg, score.actual_yaw_rad * kRadToDeg,
            score.target_pitch_rad * kRadToDeg, score.actual_pitch_rad * kRadToDeg);
        }
      }
    }
    else {
      const auto score = grading::read();
      plotter.plot(
        (score.target_yaw_rad * kRadToDeg), (score.actual_yaw_rad * kRadToDeg), (score.target_pitch_rad * kRadToDeg),
        (score.actual_pitch_rad * kRadToDeg));
    }
  }
}
