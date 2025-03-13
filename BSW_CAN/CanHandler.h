/**
 * @file CanHandler.h
 * @brief Header file for CAN message processing and signal extraction.
 * @version 1.0
 * @date 2025-01-18
 * @author Lee Chaeun
 */

#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

/* Include necessary headers */
#include <BSW_CAN/CanDefinitions.h>
#include <BSW_CAN/Mycan_init.h>
#include "BSW_Car_State_Manager/car_state_manager.h"
#include "IfxMultican.h"
#include "BSW_PID/STM_Interrupt.h"
#include <string.h>

/* CAN 작업 플래그 */
extern bool canTaskFlag; // CAN 작업 플래그
extern sint16 heading_angle_error;
extern sint16 RightOffset;
extern sint16 LeftOffset;
extern uint8_t newCautionLv;

/* 함수 프로토타입 */
sint16 getHeadingAngleError(void);

void processCanTasks(void);

uint32_t extractSignalValue(const uint8_t *data, uint8_t startBit, uint8_t length, bool isBigEndian);

void packSignalValue(uint8_t *data, uint8_t startBit, uint8_t length, uint32_t value, bool isBigEndian);

void processCanMessage(uint32_t messageId, const uint8_t *data, CanDataManager *canData);

void transmitCanMessage(uint32_t messageId);

#endif /* CAN_HANDLER_H_ */
