#ifndef INFRARED_STATUS_HPP
#define INFRARED_STATUS_HPP
#include <cstdint>
namespace infrared
{
struct Status
{
  float distance_cm;
  uint32_t received_bytes;
  uint32_t valid_frames;
  uint32_t checksum_errors;
  uint32_t uart_errors;
  bool valid;
  uint32_t baud_rate;
  uint8_t last_payload_length;
  uint8_t recent_bytes[16];  // Raw RX ring, inspect in debugger if no frames validate.
  uint8_t recent_write_index;
  uint16_t raw_distance;
  uint8_t payload_byte2;
  uint8_t payload_byte3;
  bool timer_started;
  uint32_t trigger_stamp_ms;
};
Status snapshot();
}  // namespace infrared
#endif
