#include "gimbal_can.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "homing.hpp"
#include "infrared_pitch_auto.hpp"
#include "pitch_start.hpp"
#include "trajectory_follow.hpp"

namespace gimbal_can {
namespace {
enum class InitState { Idle, Starting, Ready };
InitState state=InitState::Idle;
}

bool start() {
  taskENTER_CRITICAL();
  if (state==InitState::Ready) {taskEXIT_CRITICAL();return true;}
  if (state==InitState::Starting) {taskEXIT_CRITICAL();return false;}
  state=InitState::Starting;
  taskEXIT_CRITICAL();

  // HAL timeouts must run with interrupts enabled. No wait inside a critical section.
  CAN_FilterTypeDef filter={};
  filter.FilterBank=0;
  filter.FilterMode=CAN_FILTERMODE_IDMASK;
  filter.FilterScale=CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment=CAN_RX_FIFO0;
  filter.FilterActivation=ENABLE;
  filter.SlaveStartFilterBank=14;
  HAL_StatusTypeDef status=HAL_CAN_ConfigFilter(&hcan1,&filter);
  bool bus_started=false;
  if (status==HAL_OK) {
    status=HAL_CAN_Start(&hcan1);
    bus_started=(status==HAL_OK);
  }
  if (status==HAL_OK)
    status=HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  if (status!=HAL_OK && bus_started) (void)HAL_CAN_Stop(&hcan1);

  taskENTER_CRITICAL();
  state=(status==HAL_OK) ? InitState::Ready : InitState::Idle;
  taskEXIT_CRITICAL();
  return status==HAL_OK;
}
}  // namespace gimbal_can

// One bus dispatcher shared by independent programs; disabled programs consume nothing.
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (hcan!=&hcan1) return;
  CAN_RxHeaderTypeDef header={};
  uint8_t data[8]={};
  while (HAL_CAN_GetRxFifoFillLevel(hcan,CAN_RX_FIFO0)>0U) {
    if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&header,data)!=HAL_OK) break;
    if (header.IDE!=CAN_ID_STD || header.RTR!=CAN_RTR_DATA || header.DLC!=8) continue;
    const uint32_t now=xTaskGetTickCountFromISR();
    infrared_pitch_auto::receive_can1(header,data,now);  // Observer; never consumes legacy feedback.
    if (homing::receive_can1(header,data,now)) continue;
    if (pitch_start::receive_can1(header,data,now)) continue;
    trajectory_follow::receive_can1(header,data,now);
  }
}
