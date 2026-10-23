#ifndef CONTROL_CONSTANTS_HPP
#define CONTROL_CONSTANTS_HPP

namespace control_constants
{
// Standalone motor-test switches. Tracking selectors below can request their
// own preparation; homing remains exclusive. No test auto-starts another test.
// true: IR homing -> encoder 0.8rad hold -> IR age >=20s -> existing pitch_start
// -> dual IMU following for 5s -> weak GM6020 holds; M2006 keeps strong holding.
// false: every original standalone/one-axis/two-axis test selector works as before.
constexpr bool INFRARED_PITCH_AUTO_ENABLE = true;

// infrared_pitch_auto 运行时的串口模式：
//   true  = 7 通道精简模式：yaw目标/实际(°)、pitch目标/实际(°)、yaw分数、pitch分数、红外距离(cm)
//   false = 18 通道诊断模式
constexpr bool INFRARED_AUTO_PLOT_COMPACT = true;
constexpr float INFRARED_AUTO_TRIGGER_CM = 50.0f;
constexpr unsigned INFRARED_AUTO_PREPARE_DELAY_MS = 43000U;
constexpr unsigned INFRARED_AUTO_FOLLOW_MS = 10000U;
constexpr float INFRARED_AUTO_FOLD_ENCODER_RAD = 0.8f;
// Teammate's actual default-pose parameter is 25 degrees; keep it local to this mode.
constexpr float INFRARED_AUTO_DEFAULT_TRAVEL_RAD = 0.4363323130f;
constexpr float INFRARED_AUTO_YAW_HOLD_KP = 12.6f;  // Teammate: 70 * 0.18 Nm/rad.
constexpr float INFRARED_AUTO_YAW_HOLD_KD = 0.18f;
constexpr float INFRARED_AUTO_YAW_HOLD_MAX_NM = 0.95f;
constexpr float INFRARED_AUTO_FOLD_KP = 10.0f;
constexpr float INFRARED_AUTO_FOLD_KI = 2.0f;
constexpr float INFRARED_AUTO_FOLD_KD = 0.30f;
constexpr float INFRARED_AUTO_FOLD_TOL_RAD = 0.015f;
constexpr float INFRARED_AUTO_FOLD_QUIET_RAD_S = 0.05f;
// 折叠到最低点（Descent 结束时）的保持时间。
constexpr unsigned INFRARED_AUTO_DESCENT_HOLD_MS = 10000U;
constexpr unsigned INFRARED_AUTO_FOLD_SETTLE_MS = 200U;
constexpr unsigned INFRARED_AUTO_FOLD_TIMEOUT_MS = 15000U;
constexpr float INFRARED_AUTO_WEAK_YAW_KP = 3.0f;
constexpr float INFRARED_AUTO_WEAK_YAW_KD = 0.15f;
constexpr float INFRARED_AUTO_WEAK_YAW_MAX_NM = 0.35f;
constexpr float INFRARED_AUTO_WEAK_PITCH_KP = 3.0f;
constexpr float INFRARED_AUTO_WEAK_PITCH_KD = 0.15f;
constexpr float INFRARED_AUTO_WEAK_PITCH_MAX_NM = 0.35f;
static_assert(
  INFRARED_AUTO_PREPARE_DELAY_MS > 0U && INFRARED_AUTO_FOLLOW_MS > 0U &&
    INFRARED_AUTO_FOLD_SETTLE_MS > 0U &&
    INFRARED_AUTO_FOLD_TIMEOUT_MS > INFRARED_AUTO_FOLD_SETTLE_MS &&
    INFRARED_AUTO_FOLD_ENCODER_RAD > 0.0f && INFRARED_AUTO_TRIGGER_CM > 0.0f &&
    INFRARED_AUTO_FOLD_KP > 0.0f && INFRARED_AUTO_FOLD_KI >= 0.0f && INFRARED_AUTO_FOLD_KD > 0.0f,
  "Invalid IR automatic sequence settings");
constexpr bool HOMING_ENABLED = false;
constexpr bool PITCH_START_ENABLED = false;
// Highest tracking priority: prepare pitch_start, prelift, then start both axes.
// false preserves pitch-only/yaw-only selections below.
constexpr bool FOLLOW_BOTH_ENABLE = true;
constexpr bool FOLLOW_BOTH_ACTIVE = INFRARED_PITCH_AUTO_ENABLE || FOLLOW_BOTH_ENABLE;
static_assert(
  INFRARED_PITCH_AUTO_ENABLE || !(HOMING_ENABLED && (PITCH_START_ENABLED || FOLLOW_BOTH_ENABLE)),
  "Enable only one motor test: HOMING_ENABLED or PITCH_START_ENABLED (shared motors/CAN frames)");

// BMI088 on the moving gimbal C-board. All angles/rates are radians.
constexpr unsigned IMU_WARMUP_MS = 3000U;
constexpr unsigned IMU_CALIBRATION_MS = 2000U;
// Calibrate for real elapsed time; task executions are not milliseconds.
constexpr unsigned IMU_CALIBRATION_MIN_SAMPLES = 500U;
constexpr bool IMU_STARTUP_DIAGNOSTICS = true;  // 12 plot channels; false restores 3.
constexpr unsigned IMU_MAX_GAP_MS = 10U;
constexpr bool IMU_GIMBAL_AXIS_PROJECTION =
  true;  // Dual mode only; pitch axis supplies tilt-safe heading.
constexpr float IMU_CALIBRATION_MAX_RATE_RAD_S = 0.052359878f;  // 3 deg/s
// Whole-window standard deviation and 100ms block-mean spread (startup only).
constexpr float IMU_CALIBRATION_STD_RAD_S = 0.0052359878f;  // 0.30 deg/s
constexpr unsigned IMU_CALIBRATION_BLOCK_MS = 100U;
constexpr float IMU_CALIBRATION_MEAN_SPREAD_RAD_S = 0.013962634f;  // 0.8 deg/s
static_assert(
  IMU_CALIBRATION_STD_RAD_S > 0.0f && IMU_CALIBRATION_BLOCK_MS > 0U,
  "Invalid IMU calibration noise configuration");
constexpr float IMU_ATTITUDE_KP = 0.5f;
constexpr float IMU_YAW_DIRECTION =
  1.0f;  // TEST: if the gyro yaw rate has the opposite sign to the position, the inner loop was positive feedback
constexpr float IMU_PITCH_DIRECTION = 1.0f;
// Existing middleware C-board convention: board flat, CAN side facing forward.
// Horizontal mounting heading does not affect relative yaw. Verify this matrix
// before pitch tests: rows are gimbal X(forward), Y(left), Z(up) in sensor axes.
constexpr float IMU_SENSOR_TO_GIMBAL[3][3] = {
  {0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
static_assert(IMU_MAX_GAP_MS <= 10U && IMU_CALIBRATION_MS > 0U, "Invalid IMU timing");
static_assert(
  (IMU_YAW_DIRECTION == 1.0f || IMU_YAW_DIRECTION == -1.0f) &&
    (IMU_PITCH_DIRECTION == 1.0f || IMU_PITCH_DIRECTION == -1.0f),
  "Invalid IMU directions");

// Shared GM6020/M2006 test parameters below; each program owns its own stop logic.
// Temporary RC/CAN diagnosis. false restores the selected power/trajectory plot.
constexpr bool CHASSIS_CONTROL_DIAGNOSTICS = true;
constexpr unsigned HOME_PITCH_ID = 2U;
constexpr unsigned HOME_M2006_ID = 1U;
constexpr float HOME_PITCH_RATIO = 1.0f;
constexpr float HOME_M2006_RATIO = 36.0f;     // Output-shaft angle/speed/torque.
constexpr float HOME_PITCH_DIRECTION = 1.0f;  // Observed positive motion approaches the rear stop.

// Initial trial settings, not a mechanical rating or a tuned controller.
constexpr float HOME_PITCH_SEEK_SPEED_RAD_S = 0.35f;  // reverted  // seek crawl speed: 0.35 -> 0.80
constexpr float HOME_PITCH_SPEED_KP =
  +0.08f;  // tau = J/Kp = 12.5ms（原 0.25 下 4ms，与 2ms 周期接近会自激）
constexpr float HOME_PITCH_SPEED_KI = 0.45f;       // 与 hold 段一致
constexpr float HOME_PITCH_MAX_TORQUE_NM = 0.40f;  // 顶挡块力矩：0.50 太高，硬顶会嗡嗡震动
constexpr float HOME_PITCH_STALL_SPEED_RAD_S =
  0.15f;  // Tolerate +/-1 rpm CAN quantization; angle window still required.
constexpr float HOME_PITCH_STALL_TORQUE_NM =
  0.30f;  // stall 力矩阈值：比顶挡块力矩低 0.10 留余量 // Feedback AND commanded torque.
constexpr float HOME_PITCH_STALL_TRAVEL_RAD =
  0.05235988f;  // 3 deg：嘎噡震动的角度幅度可能超 0.5° // <=0.5 deg / stall window.
// Startup stiction is NOT a reference. Rearward motion must be seen first.
// If powered on against the stop, report NoSeekMotion instead of inventing zero.
constexpr float HOME_PITCH_MIN_SEEK_TRAVEL_RAD =
  0.087266463f;  // 5 deg net rearward travel (must exceed stall gate).
constexpr unsigned HOME_PITCH_STALL_MS = 300U;
constexpr unsigned HOME_PITCH_STARTUP_BLANK_MS = 300U;
constexpr unsigned HOME_PITCH_TIMEOUT_MS = 10000U;
constexpr float HOME_PITCH_MAX_TRAVEL_RAD = 1.570796327f;  // Software search cap 90 deg.

// After rear-stop detection, GM6020 reverses relative to its own seated angle.
// M2006 remains zero-current until the GM6020 return is complete and stationary.
constexpr float HOME_PITCH_RETURN_MOVE_RAD =
  0.3490658504f;  // reverted to 20 deg  // phase A lift: 3 deg (was 20)  // phase A lift: 20 deg  // phase A lift: 25 deg (was 20)  // 20 deg first phase  // 15 deg backoff driven by the M2006 // GM6020 shaft 30 deg backoff. // GM6020 shaft 15 deg backoff.
constexpr unsigned HOME_PITCH_RETURN_RELEASE_MS = 100U;  // Clear rearward current first.
constexpr float HOME_PITCH_RETURN_POSITION_KP =
  3.0f;  // reverted  // return position gain: 3.0 -> 5.0 (reach the limit sooner)
constexpr float HOME_PITCH_RETURN_SPEED_KP = 0.08f;  // 同上，回退段不再自激
constexpr float HOME_PITCH_RETURN_SPEED_KI = 0.45f;  // 同上
constexpr float HOME_PITCH_RETURN_MAX_TORQUE_NM = 0.80f;
constexpr float HOME_PITCH_RETURN_MAX_SPEED_RAD_S =
  0.35f;  // reverted  // return speed: 0.35 -> 0.80
constexpr unsigned HOME_PITCH_RETURN_TIMEOUT_MS =
  20000U;  // phase A + phase B are slow; 8 s was too short
// After entering Done, wait this long then freeze the ACHIEVED pose as the final zero
// (stage 4 for 500 ms -> lock). No further fine correction of the zero point.
constexpr unsigned HOME_LOCK_MS = 500U;
constexpr unsigned HOME_M2006_LOCK_MS =
  500U;  // 0.5 s hold after the M2006 arrives, then lock  // lock 0.5 s after entering Done
// Hold deadband: inside this band the controller only damps (speed target 0)
// and makes NO position correction, so the gimbal stops hunting.
constexpr float HOME_PITCH_HOLD_DEADBAND_RAD = 0.0f;
// Done-stage lock: no deadband (deadband + saturation caused the nodding relay
// oscillation) and an over-damped PD.
constexpr float HOME_PITCH_FINAL_DEADBAND_RAD =
  0.0f;  // no deadband: stiff hold  // 4 deg  // lock 250 ms after entering Done
// Second homing stage: after the 6020 locks, the M2006 rotates this much as its final zero.
constexpr float HOME_M2006_FINAL_ROT_RAD = 0.5235987756f;  // 30 deg
constexpr float HOME_M2006_FINAL_ROT_DIR =
  1.0f;  // flipped: -1.0 pushed into a stop          // same sense as the 6020 return; flip to +1 if wrong           // +1 or -1; flip if it goes the wrong way
constexpr float HOME_M2006_FINAL_TOL_RAD =
  0.06981317f;  // 4 deg (no fine hunting)   // 1 deg arrival band  // 30 deg move needs more time  // fail fast instead of waiting 15 s
constexpr float HOME_PITCH_RETURN_TOLERANCE_RAD = 0.034906585f;  // 2 deg settle band // 1 deg.
constexpr float HOME_PITCH_RETURN_SETTLE_SPEED_RAD_S = 0.15f;    // Tolerate 1 rpm quantization.
constexpr unsigned HOME_PITCH_RETURN_SETTLE_MS = 250U;
constexpr float HOME_PITCH_RETURN_OVERSHOOT_RAD = 0.261799388f;  // 15 deg past target.
constexpr float HOME_TARGET_ACCEL_RAD_S2 =
  0.5f;  // reverted: 2.0 broke the stall detection  // speed ramp: was 0.5 (very slow)

// Final GM6020 hold uses the same 60-degree target; preserve its load integral.
// M2006 captures and holds its ACTUAL final angle, without a separate 60-degree demand.
constexpr bool HOME_HOLD_AFTER_DONE = false;
constexpr float HOME_PITCH_HOLD_POSITION_KP = 10.0f;  // was 3.0: much stiffer
constexpr float HOME_PITCH_HOLD_SPEED_KP =
  0.08f;  // reverted  // damping raised (tau = J/Kp = 6.7 ms, > 2 ms CAN period)
constexpr float HOME_PITCH_HOLD_SPEED_KI =
  0.45f;  // reverted  // integral provides the steady holding torque
constexpr float HOME_PITCH_HOLD_MAX_SPEED_RAD_S = 2.0f;  // was 0.25: allow a hard push
constexpr float HOME_PITCH_HOLD_MAX_TORQUE_NM =
  0.45f;  // unchanged  // 0.45 Nm was proven enough to hold max gravity  // GM6020 absolute max; this is the ceiling  // final lock: near GM6020 max
// Weak lock used only between phase A and phase B, so the M2006 translation
// can still drag the axis a little.
constexpr float HOME_PITCH_PHASEB_MAX_TORQUE_NM = 0.38f;  // 0.30 did not hold this time
constexpr float HOME_PITCH_PHASEB_STIFF_NM_PER_RAD =
  5.0f;  // 30 -> 5: much softer, less fight with the M2006 (less rocking)
// Damping for the phase-B spring: without it the pure spring oscillates.
constexpr float HOME_PITCH_PHASEB_DAMP_NM_PER_RAD_S =
  0.08f;  // scaled down with the stiffness  // matching stiffness  // must stay below the M2006 drive (0.45) or phase B stalls
// Direct position->torque stiffness for the locked pose (a hard virtual spring).
// 60 Nm/rad means a 1 deg error already commands ~1 Nm; the torque limit caps it.
constexpr float HOME_PITCH_HOLD_STIFF_NM_PER_RAD =
  10.0f;  // Done only: 30 -> 10 (linear, no saturation hunting)  // 100 caused +/-2.2 limit cycle; phase-B value
// Damping for the Done-stage spring. K=100 with J~1e-3 needs Kd~0.6 for critical
// damping; 0.40 gives zeta~0.65 which stops the +/-2.2 Nm limit cycle.
constexpr float HOME_PITCH_HOLD_DAMP_NM_PER_RAD_S =
  0.20f;  // Done only: 0.12 -> 0.20 (over-damped)  // matching phase B  // reverted to the pre-11:22 value  // 100 caused a +/-2.2 Nm limit cycle  // final lock stiffness: 1 deg -> 1.75 Nm  // final lock stiffness  // softer spring: yields to the M2006 translation
// Constant torque used to drive the backoff and the M2006 final rotation.
// Angle-triggered, no speed loop -> no vibration.
constexpr float HOME_PITCH_RETURN_TORQUE_NM = 0.80f;  // 0.30 was not enough to move the axis
constexpr float HOME_M2006_FINAL_TORQUE_NM = 0.15f;
// Two-level drive: a larger torque to break static friction, a small one to cruise.
// A single value cannot satisfy both (0.15 did not move, 0.80 ran away).
constexpr float HOME_M2006_BREAKAWAY_TORQUE_NM = 0.65f;  // 0.45 -> 0.60: cannot pull the structure
constexpr float HOME_M2006_CRUISE_TORQUE_NM = 0.28f;     // 0.18 -> 0.26
// When the M2006 reaches HOME_M2006_PULLBACK_RAD it reverses and pulls the gimbal
// back with this (larger) torque.
constexpr float HOME_M2006_PULLBACK_RAD = 2.2f;
constexpr float HOME_M2006_PULLBACK_TORQUE_NM = 1.0f;  // 0.90 -> 1.00 (M2006 output limit)
// Pull back until the M2006 relative angle falls to this value, then lock.
constexpr float HOME_M2006_PULLBACK_STOP_RAD =
  0.0f;  // pull back to the stop pose (relative 0)  // slow version  // 0.18 -> 0.22  // slow crawl  // more headroom against damping
// Damping: brake torque proportional to the axis speed. Without it gravity can
// accelerate the mechanism past the point of control and the overspeed trip fires.
constexpr float HOME_M2006_DAMPING_NM_PER_RAD_S =
  0.30f;  // slow version damping  // caps the axis speed near 0.6 rad/s  // 0.15 cancelled the cruise torque at 1.3 rad/s  // raised so phase B keeps moving
constexpr float HOME_M2006_BREAKAWAY_SPEED_RAD_S =
  0.06f;  // 0.80 was a runaway; gearbox torque is large
// Sign of the M2006 drive torque. -1 means it pushes alonCHASSIS_CONTROL_DIAGNOSTICSg +HOME_PITCH_DIRECTION.
// If the axis does not move (or pushes into the stop), flip this to +1.
constexpr float HOME_M2006_DRIVE_DIR =
  -1.0f;  // flipped: previous sign pushed into the stop  // gearbox gives headroom: was 0.45 (25% of axis max)  // constant torque used for both M2006 moves
// After the 15 deg backoff the 6020 locks, then the M2006 keeps turning 45 deg more.
constexpr float HOME_M2006_CONTINUE_RAD =
  0.3490658504f;  // reverted to 20 deg  // phase B: another 3 deg  // phase B: 20 deg more (arm angle)  // phase B: 30 deg  // 30 deg instead of 45 deg  // max available for the M2006 path  // was 0.80: near GM6020 max (2.2 Nm)
constexpr float HOME_M2006_POSITION_KP = 3.0f;
// Done-stage M2006 hold is DAMPING ONLY: the position loop hunted through the
// lead-screw backlash and rocked the whole pitch axis.
constexpr float HOME_M2006_HOLD_DAMP_NM_PER_RAD_S = 0.30f;
// Final lock: both axes hold their recorded pose with a soft spring (0.45 Nm).
// The M2006 uses a wide deadband so lead-screw backlash cannot start a hunt.
constexpr float HOME_M2006_FINAL_STIFF_NM_PER_RAD = 6.0f;
constexpr float HOME_M2006_FINAL_DAMP_NM_PER_RAD_S = 0.20f;
constexpr float HOME_M2006_FINAL_DEADBAND_RAD =
  0.08f;  // final M2006 hold: 3.0 (was 1.0)  // lowered from 4.0: avoid fighting the 6020 hold
constexpr float HOME_M2006_SPEED_KP = 0.10f;
// 回退阶段: M2006 转速 = FOLLOW × 6020转速。不锁死（仍能动），但保持柄结构一致，
// 避免 M2006 完全自由时重力把 pitch 总成拖回挡块。正负号决定方向，反了就改成负值。
constexpr float HOME_M2006_RETURN_FOLLOW = -1.0f;  // flipped sign: try -1 first, then 0.5 / 2.0
// Deadband for the M2006 hold: inside this error band the M2006 stays passive,
// so it does not fight the 6020 through the linkage (removes the small oscillation).
constexpr float HOME_M2006_HOLD_DEADBAND_RAD =
  0.0f;  // no deadband: both axes hard-held at the end  // 4 deg  // 0 = 关闭该功能
constexpr float HOME_M2006_SPEED_KI = 0.40f;
constexpr float HOME_M2006_MAX_TORQUE_NM =
  1.00f;  // M2006 output-shaft hardware maximum  // M2006 axis max: the gearbox can also hold the pitch  // allow the M2006 path up to 1.2 Nm on the axis // HOLD only, never used during return.
static_assert(
  !INFRARED_PITCH_AUTO_ENABLE ||
    (INFRARED_AUTO_FOLD_ENCODER_RAD < HOME_M2006_PULLBACK_RAD && INFRARED_AUTO_WEAK_YAW_KP > 0.0f &&
     INFRARED_AUTO_WEAK_YAW_KD >= 0.0f && INFRARED_AUTO_WEAK_PITCH_KP > 0.0f &&
     INFRARED_AUTO_WEAK_PITCH_KD >= 0.0f && INFRARED_AUTO_WEAK_YAW_MAX_NM > 0.0f &&
     INFRARED_AUTO_WEAK_YAW_MAX_NM <= 2.223f && INFRARED_AUTO_WEAK_PITCH_MAX_NM > 0.0f &&
     INFRARED_AUTO_WEAK_PITCH_MAX_NM <= 2.223f),
  "Invalid IR return target or weak-hold configuration");
constexpr float HOME_M2006_MAX_SPEED_RAD_S = 0.35f;  // reverted  // M2006 axis speed: 0.35 -> 0.80
constexpr float HOME_M2006_SETTLE_SPEED_RAD_S = 0.05f;
// Append torque diagnostics to the existing six homing plot channels.
constexpr bool HOME_PLOT_TORQUES = true;  // true: 10 float channels; false: original 6.

constexpr unsigned HOME_READY_TIMEOUT_MS = 5000U;
constexpr unsigned HOME_FEEDBACK_TIMEOUT_MS = 100U;
constexpr unsigned HOME_MAX_STEP_MS = 10U;
constexpr unsigned HOME_CAN_INTERVAL_MS = 5U;
constexpr unsigned HOME_COMMAND_TIMEOUT_MS = 20U;
constexpr float HOME_OVERSPEED_RAD_S =
  8.0f;  // homing only: gravity can back-drive the axis, 2.0 was too tight
// Separate overspeed trip for the M2006 path: its axis travels far and gravity
// can accelerate it well past the GM6020 figure, which is not a real fault.
constexpr float HOME_M2006_OVERSPEED_RAD_S = 8.0f;
static_assert(
  HOME_PITCH_HOLD_POSITION_KP > 0.0f && HOME_PITCH_HOLD_SPEED_KP > 0.0f &&
    HOME_PITCH_HOLD_SPEED_KI >= 0.0f && HOME_PITCH_HOLD_MAX_SPEED_RAD_S > 0.0f &&
    HOME_PITCH_HOLD_MAX_TORQUE_NM > 0.0f && HOME_PITCH_HOLD_MAX_TORQUE_NM <= 0.741f * 3.0f,
  "Invalid pitch hold settings");
static_assert(
  HOME_PITCH_RETURN_MOVE_RAD > 0.0f && HOME_PITCH_RETURN_TOLERANCE_RAD > 0.0f &&
    HOME_PITCH_RETURN_POSITION_KP > 0.0f && HOME_PITCH_RETURN_SPEED_KP > 0.0f &&
    HOME_PITCH_RETURN_SPEED_KI >= 0.0f && HOME_PITCH_RETURN_MAX_TORQUE_NM > 0.0f &&
    HOME_PITCH_RETURN_MAX_TORQUE_NM <= 0.741f * 3.0f && HOME_PITCH_RETURN_MAX_SPEED_RAD_S > 0.0f &&
    HOME_PITCH_RETURN_MAX_SPEED_RAD_S < HOME_OVERSPEED_RAD_S &&
    HOME_PITCH_RETURN_SETTLE_SPEED_RAD_S > 0.0f && HOME_PITCH_RETURN_SETTLE_MS > 0U &&
    HOME_PITCH_RETURN_OVERSHOOT_RAD > 0.0f &&
    HOME_PITCH_RETURN_RELEASE_MS >= 2U * HOME_CAN_INTERVAL_MS &&
    HOME_PITCH_RETURN_TIMEOUT_MS > HOME_PITCH_RETURN_RELEASE_MS,
  "Invalid GM6020 return settings");
static_assert(HOME_PITCH_ID == 2U && HOME_M2006_ID == 1U, "Homing ID mapping changed");
static_assert(HOME_M2006_RATIO > 0.0f && HOME_PITCH_RATIO > 0.0f, "Invalid home ratios");
static_assert(
  HOME_PITCH_DIRECTION == 1.0f || HOME_PITCH_DIRECTION == -1.0f, "Invalid home direction");
static_assert(
  HOME_PITCH_MIN_SEEK_TRAVEL_RAD > HOME_PITCH_STALL_TRAVEL_RAD &&
    HOME_PITCH_MIN_SEEK_TRAVEL_RAD < HOME_PITCH_MAX_TRAVEL_RAD,
  "Invalid minimum rearward travel");
static_assert(
  HOME_PITCH_STALL_TORQUE_NM > 0.0f && HOME_PITCH_STALL_TORQUE_NM <= HOME_PITCH_MAX_TORQUE_NM,
  "Stall threshold exceeds available torque");
static_assert(
  HOME_PITCH_STALL_MS > HOME_FEEDBACK_TIMEOUT_MS && HOME_CAN_INTERVAL_MS > 0U &&
    HOME_COMMAND_TIMEOUT_MS > HOME_CAN_INTERVAL_MS,
  "Invalid home timing");

// GY-56 receive test: physical 3-pin UART1 = USART6.
constexpr bool INFRARED_PLOT_TEST = false;
constexpr bool INFRARED_PLOT_RAW_FIELDS = true;   // 8 channels; false restores 4.
constexpr unsigned INFRARED_UART_BAUD = 115200U;  // Set 115200 if module was configured so.
constexpr bool INFRARED_AUTO_BAUD = true;
constexpr unsigned INFRARED_BAUD_PROBE_MS = 2000U;
// 0: accept 3/4-byte payloads; nonzero: require this exact payload length.
constexpr unsigned INFRARED_FRAME_DATA_LENGTH = 0U;
// Assumed GY-56 distance field is cm. Use 0.1 if its manual specifies mm.
constexpr float INFRARED_RAW_TO_CM = 1.0f;
constexpr unsigned INFRARED_VALID_TIMEOUT_MS = 500U;
static_assert(
  INFRARED_UART_BAUD == 9600U || INFRARED_UART_BAUD == 115200U, "Unsupported GY-56 baud rate");
static_assert(
  (INFRARED_FRAME_DATA_LENGTH == 0U ||
   (INFRARED_FRAME_DATA_LENGTH >= 2U && INFRARED_FRAME_DATA_LENGTH <= 8U)),
  "Invalid GY-56 payload length");
static_assert(INFRARED_RAW_TO_CM > 0.0f, "Invalid distance unit conversion");

// Initial tuning values only; NOT calibrated for this chassis.
// Torque: gearbox output shaft N*m; speed: same shaft rad/s.
// Refit K1/K2 if the shaft convention or actual gear ratio changes.
// K1 是铜损系数；按参考文档它还承担"加减速阶段由加速度引起的额外能耗"。
// 【2026-10 标定尝试记录】用 data_1（1291 采样，扣除 7.4% 刹车/再生）最小二乘得
//   K1 = 3.97, K2 ≈ 0, K3 = 4.82，残差 ±1.49 W、R² = 0.981。
//   但该批数据最高只到 62 W，76~80 W 属于外推 → 实机表现反而比下面这组经验值差，
//   所以暂时回到"经验值 + 8 W 裕量"的方案。
//   若要让标定真正生效：把 CHASSIS_POWER_LIMIT_W 临时设 200、绘图模式设 4，
//   录一段真正跑到 90~100 W 的数据重新拟合，再用 3~4 W 裕量。
// K2 must be calibrated in the actual shaft-speed coordinates.
// Rotor-to-output conversion multiplies the speed-squared coefficient by ratio^2.
constexpr float POWER_K1 = 2.80f;  // bias-era coefficient  // bias-era coefficient   // W / (N*m)^2
constexpr float POWER_K2 =
  0.009f;                         // bias-era coefficient  // bias-era coefficient  // W / (rad/s)^2
constexpr float POWER_K3 = 4.0f;  // bias-era coefficient
// ---- Measured-power slow correction (bias model) ----
// effective limit = (limit - margin) x bias, bias in [MIN, MAX], regulated on the
// low-pass filtered measured power so the mean sits on the limit.
constexpr float POWER_BIAS_DOWN_PER_W = 5e-5f;
constexpr float POWER_BIAS_UP_PER_W = 2e-5f;
constexpr float POWER_BIAS_MIN = 0.6f;
constexpr float POWER_BIAS_MAX = 1.3f;
constexpr float POWER_BIAS_RECOVER_BAND_W = 2.0f;
constexpr float POWER_BIAS_FILTER_S = 0.3f;
constexpr float POWER_MECH_LOSS_RATIO = 0.25f;  // transmission/ESC loss
constexpr float POWER_RESERVE_LEAD_S = 0.05f;
constexpr float POWER_RESERVE_MAX_W = 15.0f;  // bias-era coefficient    // W, 整机空载损耗

constexpr bool POWER_MODEL_CALIBRATED = false;

constexpr float CHASSIS_POWER_LIMIT_W =
  80.0f;  // hard limit  // restored: real 80 W limit  // temporarily raised: compare prediction vs measured without limiting
// Measured-power PI target = LIMIT - MARGIN; model budget = target + bounded PI trim.
// 标定后模型误差只有 ±1.5 W（最大 6.5 W），所以裕量可以从 8 W 降回 4 W：
//   限幅点 76 W，均值贴 76，峰值约 80~82。
// 想更激进用 3；想更保守用 5~6。
constexpr float POWER_LIMIT_MARGIN_W =
  8.0f;  // nominal budget 72 W  // limit target 77 W (80 - 3)  // bias-era margin (target 72 W)
static_assert(CHASSIS_POWER_LIMIT_W > 0.0f, "Power limit must be positive");
static_assert(
  POWER_LIMIT_MARGIN_W >= 0.0f && POWER_LIMIT_MARGIN_W < CHASSIS_POWER_LIMIT_W,
  "Invalid power margin");
constexpr unsigned POWER_LIMIT_SEARCH_STEPS = 24;
static_assert(POWER_LIMIT_SEARCH_STEPS > 0, "Search needs at least one step");

// 功率计多久没更新就视为无效（不再参与修正）
constexpr unsigned POWER_METER_TIMEOUT_MS = 200U;

// Measured-power PI (budget correction in W), matching power_model.cpp.
constexpr float POWER_PI_KP = 0.4f;
constexpr float POWER_PI_KI = 1.5f;
constexpr float POWER_PI_FILTER_S = 0.03f;
constexpr float POWER_PI_DEADBAND_W = 0.3f;
constexpr float POWER_PI_TRIM_MIN_W = -40.0f;
constexpr float POWER_PI_TRIM_MAX_W = 12.0f;
constexpr float POWER_PI_RISE_W_PER_S = 20.0f;
constexpr float POWER_FAST_DROP_KP = 2.0f;
constexpr float POWER_FAST_OVER_MAX_SCALE = 0.95f;
static_assert(
  POWER_PI_KP >= 0.0f && POWER_PI_KI >= 0.0f && POWER_PI_FILTER_S > 0.0f &&
    POWER_PI_DEADBAND_W >= 0.0f && POWER_PI_TRIM_MIN_W <= 0.0f && POWER_PI_TRIM_MAX_W >= 0.0f &&
    POWER_PI_RISE_W_PER_S > 0.0f,
  "Invalid measured-power PI");
static_assert(
  POWER_FAST_DROP_KP > 0.0f && POWER_FAST_OVER_MAX_SCALE > 0.0f && POWER_FAST_OVER_MAX_SCALE < 1.0f,
  "Invalid fast power guard");

// Fast measured-power trend guard, separate from the original prediction formula.
constexpr float POWER_TREND_LEAD_S = 0.05f;    // Forecast horizon.
constexpr float POWER_TREND_FILTER_S = 0.04f;  // Filter power slope only.
constexpr float POWER_TREND_RECOVER_PER_S =
  2.0f;  // Recovery rate: up to two scale units per second.
constexpr float POWER_TREND_RECOVER_BAND_W = 0.25f;
static_assert(
  POWER_TREND_LEAD_S >= 0.0f && POWER_TREND_LEAD_S <= 0.5f, "Invalid power forecast horizon");
static_assert(
  POWER_TREND_FILTER_S > 0.0f && POWER_TREND_RECOVER_PER_S > 0.0f &&
    POWER_TREND_RECOVER_BAND_W >= 0.0f,
  "Invalid power trend guard parameters");

// =====================================================================
// ===== 云台轨迹跟随 =====
// 结构：位置环 P(+可选 I/D，速度前馈) → 速度限幅/斜坡 → 速度环 PI → 力矩限幅
//       力矩前馈：加速度 J·α + 重力 K_g·cosθ + 摩擦 τ_c·sgn(ω)
// 单位：角度 rad、角速度 rad/s、角加速度 rad/s^2、力矩 N·m、惯量 kg·m^2
// =====================================================================

// ---- 电机与装配 ----
// GM6020 的 ID 由电机上的拨码开关决定：1/2/3 位是 ID 的二进制位，第 4 位是 120 Ω 终端电阻。
//   拨成 ID=1：1 位 ON，2、3 位 OFF
//   拨成 ID=2：2 位 ON，1、3 位 OFF
//   拨成 ID=4：3 位 ON，1、2 位 OFF
// 当前硬件是 ID 2 和 4，所以这里按 2/4 配置。若要把两台电机统一改成 1 和 2：
// 改拨码后，把下面两个数字改成 1 和 2 重烧。
// CAN 滤波器是"全通"（掩码全 0），任何 ID 都能收到，不需要改滤波配置。
constexpr unsigned FOLLOW_YAW_MOTOR_ID = 4U;
constexpr unsigned FOLLOW_PITCH_MOTOR_ID = 2U;
constexpr float FOLLOW_YAW_RATIO = 1.0f;
constexpr float FOLLOW_PITCH_RATIO = 1.0f;
// 关节角随编码器角增长。若某轴反向追、误差越追越大，把对应方向改成 -1。
constexpr float FOLLOW_YAW_DIRECTION = 1.0f;
constexpr float FOLLOW_PITCH_DIRECTION =
  -1.0f;  // reverted to previous version  // reverted: direction was correct  // FIX: encoder sign was inverted -> loop ran to the limit

// Both > single yaw > pitch-only sequence > standalone test. No PID changes.
constexpr bool FOLLOW_SINGLE_YAW =
  false;  // true with FOLLOW_BOTH_ENABLE=false starts the existing yaw-only test.
constexpr bool FOLLOW_AFTER_PITCH_START = true;
// A tracking selector can request its own preparation even if the standalone
// PITCH_START_ENABLED test switch is false. Homing remains an exclusive test.
constexpr bool PITCH_START_SELECTED =
  INFRARED_PITCH_AUTO_ENABLE ||
  (!HOMING_ENABLED && (FOLLOW_BOTH_ENABLE ||
                       (!FOLLOW_SINGLE_YAW && (PITCH_START_ENABLED || FOLLOW_AFTER_PITCH_START))));
constexpr bool FOLLOW_PITCH_SEQUENCE_ENABLED =
  PITCH_START_SELECTED && (FOLLOW_BOTH_ACTIVE || FOLLOW_AFTER_PITCH_START);
// Dual yaw handoff: wait for a quiet axis, clear history, then ramp authority.
constexpr unsigned FOLLOW_BOTH_YAW_STARTUP_MS = 150U;
constexpr bool FOLLOW_BOTH_RUNTIME_DIAGNOSTICS =
  true;  // 18 channels; false restores the original six.
constexpr float FOLLOW_BOTH_YAW_START_QUIET_RAD_S = 0.10f;
constexpr float FOLLOW_BOTH_YAW_JUMP_SLACK_RAD =
  0.00872664626f;  // 0.5 deg beyond rate-based travel.
constexpr unsigned FOLLOW_BOTH_YAW_DIRECTION_GUARD_MS =
  600000U;  // needs 10 min, effectively disabled  // 600 -> 0: disable startup wrong-way guard
constexpr unsigned FOLLOW_BOTH_YAW_WRONG_WAY_MS =
  60000U;  // needs 60 s of continuous wrong-way, effectively disabled  // 20 -> 60000 ms: effectively disabled
constexpr float FOLLOW_BOTH_YAW_WRONG_WAY_RAD = 0.01745329252f;
constexpr float FOLLOW_BOTH_YAW_WRONG_WAY_RAD_S = 0.20f;
static_assert(
  FOLLOW_BOTH_YAW_STARTUP_MS > 0U && FOLLOW_BOTH_YAW_START_QUIET_RAD_S > 0.0f &&
    FOLLOW_BOTH_YAW_JUMP_SLACK_RAD > 0.0f && FOLLOW_BOTH_YAW_WRONG_WAY_MS > 0U &&
    FOLLOW_BOTH_YAW_DIRECTION_GUARD_MS >= FOLLOW_BOTH_YAW_WRONG_WAY_MS &&
    FOLLOW_BOTH_YAW_WRONG_WAY_RAD > 0.0f && FOLLOW_BOTH_YAW_WRONG_WAY_RAD_S > 0.0f,
  "Invalid dual yaw handoff guards");
constexpr float FOLLOW_PITCH_PRELIFT_RAD = 0.05235987756f;  // Actual body travel: 3 deg.
constexpr unsigned FOLLOW_PITCH_PRELIFT_TIME_MS = 2000U;
constexpr unsigned FOLLOW_PITCH_PRELIFT_SETTLE_MS = 300U;
constexpr unsigned FOLLOW_PITCH_PRELIFT_TIMEOUT_MS = 8000U;
constexpr float FOLLOW_PITCH_PRELIFT_TOL_RAD = 0.0034906585f;  // 0.2 deg.
constexpr float FOLLOW_PITCH_PRELIFT_QUIET_RAD_S = 0.035f;
constexpr float FOLLOW_IMU_SPEED_FILTER_S = 0.005f;
constexpr float FOLLOW_PITCH_USABLE_TRAVEL_RAD = 1.570796327f;  // Measured about 90 deg.
constexpr float FOLLOW_PITCH_LOWER_MARGIN_RAD = 0.01745329252f;
constexpr float FOLLOW_PITCH_UPPER_MARGIN_RAD = 0.03490658504f;
// Prelift only. Tracking uses FOLLOW_PITCH_MAX_SPEED/MAX_ACCEL below, without hidden overrides.
constexpr float FOLLOW_PITCH_PRELIFT_MAX_SPEED_RAD_S = 0.5f;
constexpr float FOLLOW_PITCH_PRELIFT_MAX_ACCEL_RAD_S2 = 3.0f;
constexpr float FOLLOW_PITCH_TRACK_FLOOR_TOL_RAD =
  0.01745329252f;  // 1 deg below the lifted start; 2 deg above lower stop.
// Full-period braking estimate. 20 opposed even the ideal descending reference;
// with 15ms latency and the configured floor, that reference requires about 124.
// This model parameter must ultimately agree with measured braking capability.
constexpr float FOLLOW_PITCH_BRAKE_DECEL_RAD_S2 = 130.0f;
constexpr float FOLLOW_PITCH_BRAKE_LATENCY_S = 0.015f;
constexpr float FOLLOW_PITCH_BRAKE_GAIN_NM_PER_RAD_S = 0.20f;
constexpr float FOLLOW_PITCH_FAULT_HOLD_KP = 4.0f;
constexpr float FOLLOW_PITCH_FAULT_HOLD_MAX_SPEED_RAD_S = 0.5f;
constexpr bool FOLLOW_PITCH_SEQUENCE_DIAGNOSTICS = true;  // Append phase/fault as channels 4/5.
static_assert(
  FOLLOW_PITCH_PRELIFT_TIME_MS > 0U && FOLLOW_PITCH_PRELIFT_MAX_SPEED_RAD_S > 0.0f &&
    FOLLOW_PITCH_PRELIFT_MAX_ACCEL_RAD_S2 > 0.0f &&
    FOLLOW_PITCH_PRELIFT_TIMEOUT_MS > FOLLOW_PITCH_PRELIFT_TIME_MS &&
    FOLLOW_PITCH_PRELIFT_RAD > FOLLOW_PITCH_LOWER_MARGIN_RAD &&
    FOLLOW_PITCH_USABLE_TRAVEL_RAD >
      FOLLOW_PITCH_PRELIFT_RAD + 1.047197552f + FOLLOW_PITCH_UPPER_MARGIN_RAD,
  "Invalid pitch prelift/physical travel");
static_assert(
  FOLLOW_PITCH_TRACK_FLOOR_TOL_RAD > 0.0f &&
    FOLLOW_PITCH_TRACK_FLOOR_TOL_RAD < FOLLOW_PITCH_PRELIFT_RAD - FOLLOW_PITCH_LOWER_MARGIN_RAD &&
    FOLLOW_PITCH_BRAKE_DECEL_RAD_S2 > 0.0f && FOLLOW_PITCH_BRAKE_LATENCY_S >= 0.0f &&
    FOLLOW_PITCH_BRAKE_GAIN_NM_PER_RAD_S > 0.0f && FOLLOW_PITCH_FAULT_HOLD_KP > 0.0f &&
    FOLLOW_PITCH_FAULT_HOLD_MAX_SPEED_RAD_S > 0.0f,
  "Invalid pitch boundary/hold configuration");

// ---- 运行模式 ----
// Select both / yaw-only / prepared pitch-only with the switches above.
// 恒力矩台架测试：不做闭环，只输出固定力矩（测静摩擦、确认转向）。测完必须改回 false。
// 【方向符号已验证：正力矩 → 实际角度增大，符号正确，DIRECTION 保持 1.0】
constexpr bool FOLLOW_CONSTANT_TORQUE_TEST = false;
constexpr float FOLLOW_TEST_TORQUE_NM = 0.12f;
constexpr float FOLLOW_TEST_TRIP_SPEED_RAD_S = 5.23598776f;  // 50 rpm，超速即锁存停机
// 机械行程保护：裸电机阶段关闭；装上云台后打开并填真实行程。
constexpr bool FOLLOW_ENFORCE_ANGLE_LIMITS = false;
// 时间缩放：1 = 原始 680 ms 周期（最快）；2/4 = 放慢并同时降低速度、加速度。
constexpr unsigned FOLLOW_TRAJECTORY_TIME_SCALE =
  1U;  // 0:03 baseline: period x1  // required: original period  // period x2 = the setting where it fitted perfectly  // Pitch sequence: original 680 ms cycle; original trajectory files stay unchanged.
constexpr bool FOLLOW_START_ENABLED =
  true;  // pitch follow test  // pitch_start test: yaw follow off  // Auto-start yaw after fresh feedback and initial pose capture.
constexpr bool FOLLOW_AUTO_CAPTURE_INITIAL_POSE = true;  // 上电自动记录初始姿态
constexpr bool FOLLOW_USE_RC_STOP = false;               // 用遥控器右拨杆下档急停

// ---- 时序 ----
constexpr unsigned FOLLOW_PERIOD_MS = 1U;  // 控制周期 1 ms
// 电流帧发送周期 = 执行器的采样周期。电机只在收到新帧时更新力矩，
// 所以闭环时间常数 τ 必须 ≥ 5~10 倍这个值，否则采样环自激（嗡嗡响）。
// 取 2 ms（500 Hz）是总线负载与带宽的折中；总线余量足够时可试 1 ms。
constexpr unsigned FOLLOW_CAN_TX_INTERVAL_MS =
  1U;  // 2 -> 1 ms: closes the loop twice as fast (control only)
constexpr unsigned FOLLOW_COMMAND_TIMEOUT_MS = 20U;
constexpr unsigned FOLLOW_FEEDBACK_TIMEOUT_MS = 100U;
constexpr unsigned FOLLOW_MAX_STEP_MS = 10U;  // 调度间隔超过它就重新初始化 PID

// ---- 安全判据 ----
constexpr float FOLLOW_MAX_TRACKING_ERROR_RAD =
  1.20f;  // 0.349(20deg) -> 1.20(69deg): never latch during follow  // 20°
constexpr unsigned FOLLOW_TRACKING_ERROR_MS =
  2000U;  // 50 -> 2000 ms: effectively no latch            // 连续超限 50 ms 即停机
constexpr float FOLLOW_OVERSPEED_FACTOR = 1.5f;

// ---- 前馈 ----
// 力矩前馈 = J·α_ref + K_g·cos(pitch) + τ_c·sgn(ω)
// 裸电机 J 很小(≈1.3e-4)、无重力，所以前两项为 0；摩擦项是裸电机跟踪的关键。
constexpr float FOLLOW_YAW_ACCEL_FF_J = 0.012f;  // kg·m^2
constexpr float FOLLOW_PITCH_ACCEL_FF_J = 0.0f;
constexpr float FOLLOW_PITCH_GRAVITY_NM =
  0.10f;  // tau = 0.15kg * 9.81 * 0.07m = 0.103 Nm  // 0 -> 0.80: heavy head needs gravity feedforward  // m·g·r
// 库仑摩擦前馈（N·m）。裸电机实测启动摩擦约 0.1 A ≈ 0.074 N·m；
// 取 0.05 既能把低速"粘住"补掉，又不会在停顿反向点造成震动（实测最优）。
constexpr float FOLLOW_YAW_FRICTION_NM =
  0.05f;  // revert: 0.07 was worse  // 0.05 -> 0.07  // back to the 0.237 best  // 0.05 -> 0.10: must break stiction at the reversal notch  // restored: removing it dropped the score 0.237 -> 0.168  // test: friction feedforward off, to check half-cycle asymmetry
constexpr float FOLLOW_PITCH_FRICTION_NM = 0.05f;
// 平滑符号函数的过渡带（rad/s）：信号小于它时前馈按比例减小，避免过零抖动
constexpr float FOLLOW_FRICTION_BLEND_RAD_S =
  0.02f;  // revert: 0.05 was worse  // 0.02 -> 0.05: wider blend = smoother at each of the 3 reversals
// 摩擦前馈方向信号的低通时间常数（s）。轨迹每周期有 6 个"停住再反向"的点，
// 速度指令会在零点逐拍抖动；直接取符号会让力矩每拍翻转 ±τ_c → 震动。
// 先低通再取符号，前馈变成缓变的 ±τ_c，震动消失而补偿能力基本保留。设 0 = 不滤波。
constexpr float FOLLOW_FRICTION_FILTER_S = 0.03f;
// 静摩擦脱困（带滞环的辅助力矩）：误差 > ON 且几乎不动 → 沿误差方向补 τ_c；
// 误差 < OFF 或已经动起来 → 撤销。设为 0 即关闭（当前关闭，改用摩擦前馈）。
constexpr float FOLLOW_BREAKAWAY_ON_RAD =
  0.0f;  // disabled again: the kick fired at every peak and added error  // 0 -> 0.015 rad (0.86 deg): ENABLE the anti-stiction kick
constexpr float FOLLOW_BREAKAWAY_OFF_RAD =
  0.010f;  // original value  // 0.010 -> 0.006: must be below ON
constexpr float FOLLOW_BREAKAWAY_SPEED_RAD_S = 0.15f;

// ---- 增益与限幅 ----
// 增益量级的来历（已验证）：
//   速度环 Kp ≈ 0.015 N·m/(rad/s) = 0.02 A/(rad/s)。再高会自激：
//   闭环时间常数 τ = J/(Kt·Kp)，J≈1.3e-4、Kt=0.741、Kp=0.04 时 τ≈4.4 ms，
//   而电流帧周期 5 ms 时 τ < 周期 → 采样环必然振荡（之前"震动"的根因）。
//   积分上限必须 ≥ 静摩擦对应的力矩，否则积分顶不过摩擦。
//   位置环 Kp：12 → 15 用于压缩高速段的"切角"滞后。
constexpr float FOLLOW_YAW_ANGLE_KP =
  30.0f;  // 50 -> 70: feedback only, no FF  // 75 -> 50: 69% overshoot means it was near oscillation  // 90 -> 75: gentler, no overshoot  // 105 still diverged -> back to the proven stable 90  // 140 diverged -> 105 (stable zone, slightly above the 90 that worked)  // 90 -> 140: outer bandwidth must cover the 3-hump detail  // 70 -> 90: match the proven pitch value  // 85 gave no gain -> back to the 1 ms best  // 70 -> 85: outer loop, no buzz risk  // 55 -> 70: score scales with outer gain (0.083 -> 0.198)  // 35 -> 55: outer loop can go higher without buzz  // 20 -> 35: outer loop only; buzz comes from the inner loop, so this is safe  // 25 -> 20: slight reduction to remove residual wobble  // 10 -> 25: raise position bandwidth (was attenuating to 50%)  // (rad/s)/rad【原始周期：带宽需 ≥3×1.47Hz，10 太小】
constexpr float FOLLOW_YAW_ANGLE_KI =
  5.0f;  // 4 -> 2: minimal integral  // 6 -> 4: less integral overshoot  // revert: 14 was worse  // 6 -> 14: absorb the constant yaw bias within one 1.36 s cycle  // 4 -> 6  // 2 -> 4: helps the lag without adding high-frequency content   // 消除峰谷处的静态误差（纯 P 压不掉）
constexpr float FOLLOW_YAW_ANGLE_KD =
  0.85f;  // matched with FF  // matched with FF  // matched with FF  // matched with FF  // keep matched with FF  // 0.8 mismatched the 1.0 velocity FF and ate 80% of the drive  // 与前馈(1.0)冲突：KD 必须为 0 或 1.0，中间值会吃掉驱动
constexpr float FOLLOW_YAW_ANGLE_INTEGRAL_LIMIT =
  0.30f;  // revert: 0.45 was worse  // 0.30 -> 0.45: allow enough authority for the bias                                      // rad/s（积分最多贡献 0.3 rad/s 的速度指令）
constexpr float FOLLOW_YAW_SPEED_KP =
  0.12f;  // 0.22 gave no gain -> back to the 1 ms best  // 0.18 -> 0.22: 1 ms command rate gives headroom  // 0.13 -> 0.18: inner loop can be faster with the 1 ms command rate  // back to the known-stable value (tau 7.7 ms)  // 0.20 -> 0.16: 0.20 was self-oscillating (tau near CAN period)  // 0.13 -> 0.20: faster inner loop, tau = J/Kp = 5 ms > 2 ms CAN  // N·m/(rad/s)【装云台后：0.015→0.100】
constexpr float FOLLOW_YAW_SPEED_KI =
  0.55f;  // 0.45 -> 0.35: integral also feeds high-frequency buzz  // N·m/rad（恢复验证值）
constexpr float FOLLOW_YAW_SPEED_KD = 0.001f;
constexpr float FOLLOW_YAW_SPEED_INTEGRAL_LIMIT = 0.25f;  // N·m
// Dual-axis IMU velocity loop: conservative trial after observed speed reversals.
// The legacy yaw-only branch keeps FOLLOW_YAW_SPEED_KP/KI/KD above.
constexpr float FOLLOW_BOTH_YAW_SPEED_KP =
  0.12f;  // 0.14 -> 0.18  // 0.18 -> 0.14: more damping margin  // 0.20 -> 0.18: single-axis value  // 0.21 -> 0.20  // 0.26 diverged -> 0.21  // 0.20 -> 0.26: inner loop bandwidth  // 0.16 -> 0.20: closer to single-axis 0.18*1.1  // 0.09 -> 0.16: dual-mode inner loop was half strength
constexpr float FOLLOW_BOTH_YAW_SPEED_KI =
  0.15f;  // Step 4: reduce integral buildup through reversals; evaluate peak overshoot.
constexpr float FOLLOW_BOTH_YAW_SPEED_KD = 0.0f;
static_assert(
  FOLLOW_BOTH_YAW_SPEED_KP > 0.0f && FOLLOW_BOTH_YAW_SPEED_KI >= 0.0f &&
    FOLLOW_BOTH_YAW_SPEED_KD >= 0.0f,
  "Invalid dual-axis yaw velocity PID");
constexpr float FOLLOW_YAW_ANGLE_MIN_RAD = 0.0f;
constexpr float FOLLOW_YAW_ANGLE_MAX_RAD = 0.0f;
constexpr float FOLLOW_YAW_MAX_SPEED_RAD_S = 15.0f;  // 10 -> 15: match pitch  // 原始周期峰值 7.15
constexpr float FOLLOW_YAW_MAX_ACCEL_RAD_S2 =
  650.0f;  // 300 -> 650: match pitch  // revert: 450 was worse  // 300 -> 450: the small spikes at each peak need sharper accel  // 130 -> 300: 130 clipped the corner accelerations  // 原始周期峰值 106.9
constexpr float FOLLOW_YAW_MAX_TORQUE_NM =
  2.0f;  // Restored from 0.02 with explicit user approval for yaw tracking.
constexpr float FOLLOW_YAW_VELOCITY_FF =
  1.0f;  // Step 2: full reference velocity feedforward after yaw reference-D compensation.
constexpr bool FOLLOW_YAW_CONFIGURED = true;

// pitch 轴：与 yaw 同步为"装云台"整定值（内环必须加强，否则跟随严重滞后）
constexpr float FOLLOW_PITCH_ANGLE_KP =
  90.0f;  // 70 -> 90: pushes the peak closer  // 40 -> 70: yaw-proven value, amplitude follows gain  // 0:03 baseline  // 40 -> 70: match the yaw-proven value  // 1.0 -> 40: was 70x too small  // pitch 惯量小，yaw 的 15 会超调 60%，先降到 6
constexpr float FOLLOW_PITCH_ANGLE_KI =
  3.0f;  // 0:03 baseline  // 3 -> 6  // 8 -> 3: integral drove the overshoot at the peak  // reverted  // 8 -> 3: integral windup drove it up  // 2 -> 8: needs integral against gravity
constexpr float FOLLOW_PITCH_ANGLE_KD =
  0.95f;  // 1.0 -> 0.95: softer torque step at each reversal  // 0.90 -> 1.0 matched with FF  // step 1: match VELOCITY_FF=0.90 so the feedforward is not eaten  // 0:03 baseline  // matched with FF  // 0 -> 1.0: matches VELOCITY_FF=1.0, adds damping against overshoot
constexpr float FOLLOW_PITCH_ANGLE_INTEGRAL_LIMIT =
  0.60f;  // reverted  // 0.60 -> 0.15: tighter anti-windup  // 0.30 -> 0.60: room for the gravity torque
constexpr float FOLLOW_PITCH_SPEED_KP =
  0.20f;  // 0.28 -> 0.24: 0.28 buzzed (jagged curve), 0.24 keeps most of the score gain  // 0:03 baseline  // keep the damping that fixed the overshoot  // 0.11 -> 0.22: inner-loop damping is what stops the overshoot  // 0.08 -> 0.15: heavy axis has more inertia, tolerates more gain  // pitch 惯量小，内环适当降低（yaw 用 0.13）
constexpr float FOLLOW_PITCH_SPEED_KI =
  0.15f;  // 0.30 -> 0.22: integral also feeds the buzz  // 0:03 baseline  // 0.40 -> 0.30: less integral in the speed loop
constexpr float FOLLOW_PITCH_SPEED_KD = 0.0f;
constexpr float FOLLOW_PITCH_SPEED_INTEGRAL_LIMIT = 0.25f;
constexpr float FOLLOW_PITCH_ANGLE_MIN_RAD = 0.0f;
constexpr float FOLLOW_PITCH_ANGLE_MAX_RAD = 0.0f;
constexpr float FOLLOW_PITCH_MAX_SPEED_RAD_S = 15.0f;  // 10 -> 15    // 原始周期峰值 8.31
constexpr float FOLLOW_PITCH_MAX_ACCEL_RAD_S2 =
  650.0f;  // 450 -> 650: rounds off the peak, needs sharper accel  // 250 -> 450  // 0:03 baseline  // 250 -> 600: 2x speed needs 4x accel  // 130 -> 250  // 原始周期峰值 69.0
constexpr float FOLLOW_PITCH_MAX_TORQUE_NM =
  2.0f;  // 1.50 -> 2.00: peak error needs 1.80 Nm, was clipped (GM6020 max 2.22)  // 1.20 -> 1.50: headroom for the peak  // 0.80 -> 1.20  // 0:03 baseline  // 0.80 -> 1.80: 2x speed needs 2x torque, was saturating  // reverted  // 0.80 -> 0.30: load is only 0.10 Nm, huge gain slammed to the limit  // 1.80 -> 0.80: load is only 0.1 Nm, keep headroom but not excessive  // 0.40 -> 1.80: THIS was the stall (GM6020 max 2.22)
constexpr float FOLLOW_PITCH_VELOCITY_FF =
  0.95f;  // 1.0 -> 0.95: matched with KD  // 0.90 -> 1.0: full feedforward kills the lag  // 0:03 baseline  // back to 1.0: must match ANGLE_KD  // 1.0 -> 0.90: reduce lead, keep some
constexpr bool FOLLOW_PITCH_CONFIGURED = true;

// 默认微分滤波时间常数；双轴跟随 yaw 位置环单独配置。
constexpr float FOLLOW_D_FILTER_S = 0.05f;
constexpr float FOLLOW_BOTH_YAW_ANGLE_D_FILTER_S =
  0.015f;  // Step 3: 15 ms, yaw position loop only.

// ---- 上电找零（撞挡块 → 回退固定角度 → 该位置作为起始角）----
// 流程：恒力矩朝一个方向慢转 → 角速度≈0 持续 STALL_MS 判定顶到挡块
//       → 反向走 BACKOFF_RAD → 停稳 SETTLE_MS → 捕捉参考零点并进入跟随
// 第一次方向超时会自动换反方向重试；两个方向都超时 → 报故障、不出力矩。
constexpr bool FOLLOW_HOMING_ENABLED = true;
// 单轴找零开关：yaw 用绝对编码器/相对起点，不需要撞挡块；只给 pitch 找零
constexpr bool FOLLOW_YAW_HOMING_ENABLED = false;
constexpr bool FOLLOW_PITCH_HOMING_ENABLED = true;
constexpr float FOLLOW_HOMING_TORQUE_NM = 0.15f;          // yaw 试探力矩
constexpr float FOLLOW_HOMING_PITCH_TORQUE_NM = 0.30f;    // pitch 试探力矩需大于重力矩
constexpr float FOLLOW_HOMING_STALL_SPEED_RAD_S = 0.06f;  // 判定"停转"的速度阈值
// 找零采用"限速爬行"而不是恒力矩：恒力矩在自由轴上会一直加速，0.1 秒就能冲到超速。
constexpr float FOLLOW_HOMING_SEEK_SPEED_RAD_S = 0.60f;  // 找零爬行目标转速
constexpr float FOLLOW_HOMING_SEEK_KP = 1.0f;            // 找零速度环增益 N·m/(rad/s)
constexpr unsigned FOLLOW_HOMING_STALL_MS = 200U;        // 停转持续多久算顶到挡块
constexpr unsigned FOLLOW_HOMING_TIMEOUT_MS = 4000U;     // 单个方向最长找零时间
constexpr float FOLLOW_HOMING_BACKOFF_RAD = 0.5236f;     // 顶到挡块后回退的角度（30°）
// 回退段力矩 = 启动力矩前馈 + 位置P - 速度D。启动力矩必须大于云台的静摩擦+重力，
// 否则会像之前那样：顶到挡块后 0.09 N·m 推不动，卡在原地不动。
constexpr float FOLLOW_HOMING_BACKOFF_FF_NM = 0.20f;  // 启动力矩前馈（脱离挡块用）
constexpr float FOLLOW_HOMING_POS_KP = 1.5f;          // 回退段位置增益 N·m/rad
constexpr float FOLLOW_HOMING_POS_KD = 0.10f;  // 回退段阻尼 N·m/(rad/s)（30 度行程需更大阻尼）
constexpr float FOLLOW_HOMING_MAX_TORQUE_NM = 0.45f;          // 回退段力矩上限
constexpr float FOLLOW_HOMING_TOLERANCE_RAD = 0.01f;          // 回退到位判定
constexpr unsigned FOLLOW_HOMING_SETTLE_MS = 100U;            // 到位后稳定确认时间
constexpr unsigned FOLLOW_HOMING_BACKOFF_TIMEOUT_MS = 3000U;  // 回退超时（推不动就报错）

// ---- 绘图 ----
// 0 = 底盘功率（实测 / 期望 / 限幅系数），1 = 跟随曲线（单yaw：目标/实际/分数，双轴按下方配置），
// 2 = 原始参考，3 = 底盘诊断（FL 目标速度 / 实际速度 / 力矩 / 静止释放标志），
// 4 = 功率模型标定（Στω / Στ²×100 / Σω²÷20 / 实测功率）
constexpr unsigned FOLLOW_PLOT_MODE = 1U;  // follow view
constexpr bool FOLLOW_YAW_DIAGNOSTICS =
  false;  // false = plot curves, not frame counters  // 当前：双轴反馈帧数诊断（查 pitch 电机是否在线）
// 模式 1 画哪根轴：0 = yaw（目标/实际），1 = pitch（目标/实际）
constexpr unsigned FOLLOW_PLOT_AXIS =
  0U;  // 0 -> takes the dual 4-channel branch  // show pitch  // 0 -> 1: show the pitch axis

static_assert(
  FOLLOW_YAW_MOTOR_ID >= 1U && FOLLOW_YAW_MOTOR_ID <= 4U && FOLLOW_PITCH_MOTOR_ID >= 1U &&
    FOLLOW_PITCH_MOTOR_ID <= 4U && FOLLOW_YAW_MOTOR_ID != FOLLOW_PITCH_MOTOR_ID,
  "GM6020 current mode: IDs 1-4 share frame 0x1FE and must be different");
static_assert(
  FOLLOW_CAN_TX_INTERVAL_MS > 0U && FOLLOW_COMMAND_TIMEOUT_MS > FOLLOW_CAN_TX_INTERVAL_MS,
  "Invalid CAN timing");
static_assert(FOLLOW_PERIOD_MS > 0U && FOLLOW_TRACKING_ERROR_MS > 0U, "Invalid control timing");
static_assert(
  (FOLLOW_YAW_DIRECTION == 1.0f || FOLLOW_YAW_DIRECTION == -1.0f) &&
    (FOLLOW_PITCH_DIRECTION == 1.0f || FOLLOW_PITCH_DIRECTION == -1.0f),
  "Joint directions must be +1 or -1");
static_assert(
  FOLLOW_TRAJECTORY_TIME_SCALE > 0U && FOLLOW_TRAJECTORY_TIME_SCALE <= 100U,
  "Invalid trajectory time scale");
static_assert(FOLLOW_PLOT_MODE <= 5U, "Invalid plot mode");
static_assert(
  FOLLOW_MAX_TRACKING_ERROR_RAD > 0.0f && FOLLOW_OVERSPEED_FACTOR > 1.0f,
  "Protection thresholds must be positive and above 1");
static_assert(
  FOLLOW_D_FILTER_S >= 0.0f && FOLLOW_D_FILTER_S <= 1.0f,
  "Invalid derivative filter time constant");
static_assert(
  FOLLOW_BOTH_YAW_ANGLE_D_FILTER_S >= 0.0f && FOLLOW_BOTH_YAW_ANGLE_D_FILTER_S <= 1.0f,
  "Invalid dual-axis yaw angle derivative filter time constant");

struct FollowPidConfig
{
  float kp;
  float ki;
  float kd;
  float integral_limit;  // 积分项输出的绝对值上限（角度环 rad/s，速度环 N·m）
};

struct FollowAxisConfig
{
  FollowPidConfig angle_pid;  // 输入关节角 rad，输出目标转速 rad/s
  FollowPidConfig speed_pid;  // 输入转速 rad/s，输出力矩 N·m
  float angle_min_rad;        // 机械行程（不是轨迹范围）
  float angle_max_rad;
  float max_speed_rad_s;
  float max_accel_rad_s2;  // 目标转速斜坡，必须 ≥ 轨迹峰值加速度
  float max_output;        // 对称力矩限幅 N·m
  float velocity_feedforward_gain;
  float accel_feedforward_j;     // 加速度前馈惯量 kg·m^2，0 = 关闭
  float gravity_coefficient_nm;  // 重力矩系数 m·g·r，0 = 关闭
  float friction_nm;             // 库仑摩擦前馈 N·m，0 = 关闭
  bool configured;
};

constexpr FollowAxisConfig FOLLOW_YAW = {
  {FOLLOW_YAW_ANGLE_KP, FOLLOW_YAW_ANGLE_KI, FOLLOW_YAW_ANGLE_KD, FOLLOW_YAW_ANGLE_INTEGRAL_LIMIT},
  {FOLLOW_YAW_SPEED_KP, FOLLOW_YAW_SPEED_KI, FOLLOW_YAW_SPEED_KD, FOLLOW_YAW_SPEED_INTEGRAL_LIMIT},
  FOLLOW_YAW_ANGLE_MIN_RAD,
  FOLLOW_YAW_ANGLE_MAX_RAD,
  FOLLOW_YAW_MAX_SPEED_RAD_S,
  FOLLOW_YAW_MAX_ACCEL_RAD_S2,
  FOLLOW_YAW_MAX_TORQUE_NM,
  FOLLOW_YAW_VELOCITY_FF,
  FOLLOW_YAW_ACCEL_FF_J,
  0.0f,
  FOLLOW_YAW_FRICTION_NM,
  FOLLOW_YAW_CONFIGURED};

constexpr FollowAxisConfig FOLLOW_PITCH = {
  {FOLLOW_PITCH_ANGLE_KP, FOLLOW_PITCH_ANGLE_KI, FOLLOW_PITCH_ANGLE_KD,
   FOLLOW_PITCH_ANGLE_INTEGRAL_LIMIT},
  {FOLLOW_PITCH_SPEED_KP, FOLLOW_PITCH_SPEED_KI, FOLLOW_PITCH_SPEED_KD,
   FOLLOW_PITCH_SPEED_INTEGRAL_LIMIT},
  FOLLOW_PITCH_ANGLE_MIN_RAD,
  FOLLOW_PITCH_ANGLE_MAX_RAD,
  FOLLOW_PITCH_MAX_SPEED_RAD_S,
  FOLLOW_PITCH_MAX_ACCEL_RAD_S2,
  FOLLOW_PITCH_MAX_TORQUE_NM,
  FOLLOW_PITCH_VELOCITY_FF,
  FOLLOW_PITCH_ACCEL_FF_J,
  FOLLOW_PITCH_GRAVITY_NM,
  FOLLOW_PITCH_FRICTION_NM,
  FOLLOW_PITCH_CONFIGURED};

}  // namespace control_constants

#endif  // CONTROL_CONSTANTS_HPP
