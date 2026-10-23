#ifndef SHARED_DATA_HPP
#define SHARED_DATA_HPP

#include <cstdint>

struct SharedData
{
  // 遥控器
  uint8_t sw_r;
  uint8_t sw_l;
  float ch_lv;
  float ch_lh;
  float ch_lu;

  // 红外测距
  uint16_t distance_cm;
  uint8_t infrared_valid;
  uint32_t infrared_last_rx_ms;
  uint32_t infrared_timer_ms;  // Elapsed from first valid <=40cm event; never reset by distance/loss.

  // 自主任务
  uint8_t auto_state;    // 见 AutoState 枚举
  uint8_t auto_trigger;  // 1=自主运行, 0=不运行
  float auto_vx;         // 自主前进速度 m/s
  float auto_vy;         // 自主横向速度 m/s（一般 0）
  float auto_omega;      // 自主旋转角速度 rad/s（一般 0）
};

extern SharedData g_data;

#endif  // SHARED_DATA_HPP