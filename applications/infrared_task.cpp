#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "control_constants.hpp"
#include "infrared_status.hpp"
#include "shared_data.hpp"
#include "usart.h"

namespace {
uint8_t rx_byte;
uint8_t frame[13] = {};
uint8_t used = 0;
uint32_t last_byte_ms = 0, last_frame_ms = 0;
uint32_t byte_count = 0, frame_count = 0, checksum_errors = 0, uart_errors = 0;
float distance_cm = 0.0f;
bool have_frame = false;
bool timer_started = false;
uint32_t timer_start_ms = 0;
uint32_t current_baud = control_constants::INFRARED_UART_BAUD;
uint8_t last_payload_length = 0;
uint16_t last_raw_distance = 0;
uint8_t last_payload_byte2 = 0, last_payload_byte3 = 0;
uint8_t recent_bytes[16] = {};
uint8_t recent_write_index = 0;

void consume(uint8_t byte, uint32_t now)
{
  if (used && now - last_byte_ms > 50U) used = 0;
  last_byte_ms = now;
  ++byte_count;
  recent_bytes[recent_write_index] = byte;
  recent_write_index = (recent_write_index + 1U) % 16U;
  if (used == 0) { if (byte == 0x5A) frame[used++] = byte; return; }
  if (used == 1) {
    if (byte == 0x5A) frame[used++] = byte;
    else used = 0;
    return;
  }
  if (used == 2 && byte != 0x15) {
    // Preserve an overlapping 5A 5A header.
    used = byte == 0x5A ? 2 : 0;
    return;
  }
  const unsigned required_length = control_constants::INFRARED_FRAME_DATA_LENGTH;
  if (used == 3 && (byte < 2U || byte > 8U ||
      (required_length == 0U ? (byte != 3U && byte != 4U) : byte != required_length))) {
    used = byte == 0x5A ? 1 : 0;
    return;
  }
  frame[used++] = byte;
  if (used < 4 || used != static_cast<unsigned>(frame[3]) + 5U) return;
  uint8_t sum = 0;
  for (unsigned i = 0; i + 1 < used; ++i) sum += frame[i];
  if (sum == frame[used - 1]) {
    const uint16_t raw = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];
    // 0/FFFF are treated as invalid; status bytes require the exact module manual.
    distance_cm = raw * control_constants::INFRARED_RAW_TO_CM;
    last_frame_ms = now;
    have_frame = raw != 0U && raw != 0xFFFFU && distance_cm <= 65535.0f;
    last_payload_length = frame[3];
    last_raw_distance = raw;
    last_payload_byte2 = frame[3] >= 3U ? frame[6] : 0U;
    last_payload_byte3 = frame[3] >= 4U ? frame[7] : 0U;
    ++frame_count;
    g_data.distance_cm = have_frame ? static_cast<uint16_t>(distance_cm) : 0U;
    g_data.infrared_last_rx_ms = now;  // Last checksum-correct frame, not arbitrary byte.
    g_data.infrared_valid = have_frame ? 1U : 0U;
    // Timestamp the FIRST valid near reading, never arbitrary bytes or invalid ranges.
    if (have_frame && !timer_started && distance_cm <= control_constants::INFRARED_AUTO_TRIGGER_CM) {
      timer_started = true;
      timer_start_ms = now;
    }
  }
  else ++checksum_errors;
  used = 0;
}
}

namespace infrared {
Status snapshot()
{
  taskENTER_CRITICAL();
  Status result = {distance_cm, byte_count, frame_count, checksum_errors, uart_errors,
    have_frame && (osKernelSysTick() - last_frame_ms <
                  control_constants::INFRARED_VALID_TIMEOUT_MS),
    current_baud, last_payload_length, {}, recent_write_index,
    last_raw_distance, last_payload_byte2, last_payload_byte3,timer_started,timer_start_ms};
  for (unsigned i = 0; i < 16U; ++i) result.recent_bytes[i] = recent_bytes[i];
  taskEXIT_CRITICAL();
  return result;
}
}

extern "C" void infrared_task()
{
  g_data.infrared_valid = 0;
  g_data.infrared_timer_ms = 0;
  timer_started = false;
  timer_start_ms = 0;
  // USART6 is dedicated to the sensor; USART1 plot baud remains unchanged.
  HAL_UART_AbortReceive(&huart6);
  huart6.Init.BaudRate = control_constants::INFRARED_UART_BAUD;
  if (HAL_UART_Init(&huart6) != HAL_OK) { Error_Handler(); }
  HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
  uint32_t probe_start_ms = osKernelSysTick(), probe_start_frames = 0;
  bool baud_locked = false;
  for (;;) {
    // Require two checksum-correct frames at the SAME baud before locking.
    taskENTER_CRITICAL();
    const bool enough_frames = frame_count - probe_start_frames >= 2U;
    taskEXIT_CRITICAL();
    if (enough_frames) baud_locked = true;
    if (control_constants::INFRARED_AUTO_BAUD && !baud_locked &&
        osKernelSysTick() - probe_start_ms >= control_constants::INFRARED_BAUD_PROBE_MS) {
      HAL_UART_AbortReceive(&huart6);
      taskENTER_CRITICAL();
      used = 0;
      have_frame = false;
      g_data.infrared_valid = 0;
      probe_start_frames = frame_count;
      current_baud = current_baud == 9600U ? 115200U : 9600U;
      taskEXIT_CRITICAL();
      huart6.Init.BaudRate = current_baud;
      if (HAL_UART_Init(&huart6) != HAL_OK) { Error_Handler(); }
      HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
      probe_start_ms = osKernelSysTick();
    }
    taskENTER_CRITICAL();
    if (!have_frame || osKernelSysTick() - last_frame_ms >=
                       control_constants::INFRARED_VALID_TIMEOUT_MS)
      g_data.infrared_valid = 0;
    if (timer_started) g_data.infrared_timer_ms = osKernelSysTick() - timer_start_ms;
    // Existing HAL error callback belongs to rc_task; recover USART6 here.
    if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
      ++uart_errors;
      used = 0;
      have_frame = false;
      g_data.infrared_valid = 0;
      HAL_UART_AbortReceive(&huart6);
    }
    if (huart6.RxState == HAL_UART_STATE_READY)
      HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
    taskEXIT_CRITICAL();
    osDelay(10);
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart != &huart6) return;
  consume(rx_byte, osKernelSysTick());
  HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
}

