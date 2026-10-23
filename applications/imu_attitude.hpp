#ifndef IMU_ATTITUDE_HPP
#define IMU_ATTITUDE_HPP

#include <cmath>

namespace imu_attitude_detail {
constexpr float pi = 3.14159265359f;
constexpr float delta(float current, float previous) {
  const float d = current - previous;
  return d > pi ? d - 2.0f*pi : (d < -pi ? d + 2.0f*pi : d);
}
struct GimbalProjection {float heading_x, heading_y, forward_x, forward_y, forward_z;};
// Body-to-world rotation. Correct the constant mounting roll measured at boot.
// The pitch axis (corrected body Y) remains horizontal under ideal yaw/pitch
// motion, so it gives heading even when the forward axis points vertically.
constexpr GimbalProjection project(const float q[4], float sin_roll, float cos_roll) {
  const float w=q[0], x=q[1], y=q[2], z=q[3];
  const float axis_x=2.0f*(x*y-w*z)*cos_roll-2.0f*(x*z+w*y)*sin_roll;
  const float axis_y=(1.0f-2.0f*(x*x+z*z))*cos_roll-2.0f*(y*z-w*x)*sin_roll;
  return {-axis_x,axis_y,1.0f-2.0f*(y*y+z*z),2.0f*(x*y+w*z),2.0f*(x*z-w*y)};
}
// For a two-axis yaw/pitch gimbal, world-vertical angular velocity is yaw rate.
// Body gyro Z alone is not yaw rate once the board tilts.
constexpr float vertical_rate(const float q[4], const float gyro[3]) {
  return 2.0f*(q[1]*q[3]-q[0]*q[2])*gyro[0] +
    2.0f*(q[2]*q[3]+q[0]*q[1])*gyro[1] +
    (1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]))*gyro[2];
}
}

// Body-to-world quaternion, ENU-like axes with gravity measured along +Z.
// No encoder or trajectory inputs are used by this estimator.
class ImuAttitude {
public:
  bool begin(const float a[3], bool gimbal_projection=false) {
    const float n = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (!std::isfinite(n) || n < 1.0f) return false;
    const float roll = std::atan2(a[1], a[2]);
    projected_gimbal_=gimbal_projection;
    mount_sin_roll_=std::sin(roll); mount_cos_roll_=std::cos(roll);
    const float pitch = std::atan2(-a[0], std::sqrt(a[1]*a[1] + a[2]*a[2]));
    const float cr = std::cos(roll*0.5f), sr = std::sin(roll*0.5f);
    const float cp = std::cos(pitch*0.5f), sp = std::sin(pitch*0.5f);
    q_[0]=cr*cp; q_[1]=sr*cp; q_[2]=cr*sp; q_[3]=-sr*sp;
    previous_yaw_ = 0.0f;
    previous_pitch_ = 0.0f;
    yaw = 0.0f;
    return angles(false);
  }

  bool update(const float a[3], const float w[3], float dt, float kp) {
    if (!std::isfinite(dt) || dt <= 0.0f || dt > 0.01f) return false;
    float wx=w[0], wy=w[1], wz=w[2];
    const float n = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    // Strong linear acceleration is not a reliable gravity observation.
    if (std::isfinite(n) && std::fabs(n-9.80665f) < 1.5f) {
      const float ax=a[0]/n, ay=a[1]/n, az=a[2]/n;
      const float vx=2.0f*(q_[1]*q_[3]-q_[0]*q_[2]);
      const float vy=2.0f*(q_[0]*q_[1]+q_[2]*q_[3]);
      const float vz=q_[0]*q_[0]-q_[1]*q_[1]-q_[2]*q_[2]+q_[3]*q_[3];
      wx += kp*(ay*vz-az*vy);
      wy += kp*(az*vx-ax*vz);
      wz += kp*(ax*vy-ay*vx);
    }
    wx*=0.5f*dt; wy*=0.5f*dt; wz*=0.5f*dt;
    const float q0=q_[0]-q_[1]*wx-q_[2]*wy-q_[3]*wz;
    const float q1=q_[1]+q_[0]*wx+q_[2]*wz-q_[3]*wy;
    const float q2=q_[2]+q_[0]*wy-q_[1]*wz+q_[3]*wx;
    const float q3=q_[3]+q_[0]*wz+q_[1]*wy-q_[2]*wx;
    const float norm=std::sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
    if (!std::isfinite(norm) || norm < 0.1f) return false;
    q_[0]=q0/norm; q_[1]=q1/norm; q_[2]=q2/norm; q_[3]=q3/norm;
    return angles(true);
  }

  float yaw = 0.0f;    // Unwrapped; never jumps at +/-pi.
  float pitch = 0.0f;
  float world_vertical_rate(const float gyro[3]) const {
    return imu_attitude_detail::vertical_rate(q_,gyro);
  }
private:
  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float previous_yaw_ = 0.0f;
  float previous_pitch_ = 0.0f;
  float mount_sin_roll_=0.0f, mount_cos_roll_=1.0f;
  bool projected_gimbal_=false;
  bool angles(bool accumulate) {
    if (projected_gimbal_) {
      const auto p=imu_attitude_detail::project(q_,mount_sin_roll_,mount_cos_roll_);
      const float n=std::sqrt(p.heading_x*p.heading_x+p.heading_y*p.heading_y);
      if (!std::isfinite(n) || n<0.1f) return false;
      const float heading=std::atan2(p.heading_x,p.heading_y);
      const float forward=(p.forward_x*p.heading_y+p.forward_y*p.heading_x)/n;
      const float tilt=std::atan2(-p.forward_z,forward);
      if (accumulate) {
        yaw+=imu_attitude_detail::delta(heading,previous_yaw_);
        pitch+=imu_attitude_detail::delta(tilt,previous_pitch_);
      } else {
        pitch=tilt;
      }
      previous_yaw_=heading; previous_pitch_=tilt;
      return std::isfinite(yaw) && std::isfinite(pitch);
    }
    constexpr float pi=3.14159265359f;
    const float wrapped=std::atan2(2.0f*(q_[0]*q_[3]+q_[1]*q_[2]),
      1.0f-2.0f*(q_[2]*q_[2]+q_[3]*q_[3]));
    float sine=2.0f*(q_[0]*q_[2]-q_[1]*q_[3]);
    if (sine > 1.0f) sine=1.0f;
    if (sine < -1.0f) sine=-1.0f;
    pitch=std::asin(sine);
    float delta=wrapped-previous_yaw_;
    if (delta > pi) delta-=2.0f*pi;
    if (delta < -pi) delta+=2.0f*pi;
    if (accumulate) yaw+=delta;
    previous_yaw_=wrapped;
    return std::isfinite(yaw) && std::isfinite(pitch);
  }
};

#endif
