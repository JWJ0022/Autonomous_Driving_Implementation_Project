/**
 * @file CanHandler.c
 * @brief Implementation for processing CAN messages and extracting signals
 * @version 1.0
 * @date 2025-01-18
 * Made by Lee Chaeun *^^*
 * Made by L.SH ㅡㅡ!
 */
#include <BSW_CAN/CanDefinitions.h>
#include <BSW_CAN/CanHandler.h>
#include <stdbool.h>
#include <stdint.h>

sint16 heading_angle_error = 0;
sint16 RightOffset = 0;
sint16 LeftOffset = 0;
uint8_t newCautionLv = 0;

/**
 * @brief Extracts a signal value from CAN message data.
 */

sint16 getHeadingAngleError(void)
{
    return heading_angle_error;
}

uint32_t extractSignalValue (const uint8_t *data, uint8_t startBit, uint8_t length, bool isBigEndian)
{
    uint32_t signalValue = 0;
    uint8_t byteOffset = startBit / 8;
    uint8_t bitOffset = startBit % 8;

    for (uint8_t i = 0; i < ((bitOffset + length + 7) / 8); i++)
    {
        uint8_t byte = data[byteOffset + i];
        if (isBigEndian)
        {
            signalValue = (signalValue << 8) | (uint32_t) byte;
        }
        else
        {
            signalValue |= (uint32_t) byte << (i * 8);
        }
    }

    uint8_t shift = (isBigEndian) ? (uint8_t) ((8 - ((startBit + length) % 8)) % 8) : bitOffset;
    signalValue = (signalValue >> shift) & ((1U << length) - 1U);

    return signalValue;
}

int32_t extractSignedSignalValue (const uint8_t *data, uint8_t startBit, uint8_t length, bool isBigEndian)
{
    uint32_t signalValue = extractSignalValue(data, startBit, length, isBigEndian);

    // 부호 확장 적용
    if ((signalValue >> (length - 1)) & 1)  // MSB(최상위 비트)가 1이면 음수
    {
        signalValue |= ~((1U << length) - 1U);  // 상위 비트를 1로 채워서 부호 확장
    }

    return (int32_t) signalValue;
}

/**
 * @brief Packs a signal value into CAN message data.
 */

void packSignalValue (uint8_t *data, uint8_t startBit, uint8_t length, uint32_t value, bool isBigEndian)
{
    uint32_t mask = (1U << length) - 1;
    value &= mask;

    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t bitPosition = startBit + i;
        uint8_t byteIndex = bitPosition / 8;
        uint8_t bitInByte = bitPosition % 8;

        if (value & (1U << i))
        {
            data[byteIndex] |= (1U << bitInByte);
        }
        else
        {
            data[byteIndex] &= ~(1U << bitInByte);
        }
    }
}

/**
 * @brief Processes a received CAN message.
 */

void processCanMessage (uint32_t messageId, const uint8_t *data, CanDataManager *canData)
{
    switch (messageId)
    {
        case DMS_DrvCautionLv :
        {
            newCautionLv = extractSignalValue(data, 0, 3, false);

            if (canData->rxData->DrvCautionLv != newCautionLv)
            {
                canData->rxData->DrvCautionLv = newCautionLv;
                canData->flags->DrvCautionLv_flag = true;
            }
        }
            break;

        case LKAS_HeadAngle :

            heading_angle_error = extractSignedSignalValue(data, 0, 16, false);
            canData->rxData->HeadAngle = heading_angle_error;
            break;

        case LKAS_Right :

            RightOffset = extractSignedSignalValue(data, 0, 16, false);
            canData->rxData->Right = RightOffset;
            break;

        case LKAS_Left :

            LeftOffset = extractSignedSignalValue(data, 0, 16, false);
            canData->rxData->Left = LeftOffset;
            break;

        default :
            break;
    }
}

void processCanMessages (CanDataManager *canData)
{
    while (canQueueHead != canQueueTail) // ✅ 큐가 비어있지 않으면 처리
    {
        uint32_t messageId = canMessageQueue[canQueueHead].messageId;
        uint8_t *data = canMessageQueue[canQueueHead].data;

        // ✅ 수신된 메시지 처리 (함수 인자 추가)
        processCanMessage(messageId, data, canData);

        // ✅ 큐 포인터 이동 (순환 큐 방식)
        canQueueHead = (canQueueHead + 1) % CAN_QUEUE_SIZE;
    }

// ✅ 모든 메시지를 처리한 후 플래그 초기화
    canDataUpdated = false;
}

/**
 * @brief Prepares and transmits a CAN message.
 */

void transmitCanMessage (uint32_t messageId)
{
    uint8_t data[8] = {0};

    switch (messageId)
    {
        case Motor_BrakeSta :
        {
            if (carState.stop == TRUE && carState.emergencyBrake == FALSE)
            {
                canDataManager.txData->BrakeSta = 1;
            }
            else if(carState.emergencyBrake == TRUE)
            {
                canDataManager.txData->BrakeSta = 2;
            }
            else
            {
                canDataManager.txData->BrakeSta = 3;
            }
            packSignalValue(data, 0, 2, canDataManager.txData->BrakeSta, false);
        }
            break;
        case Motor_EmergencyAlarm :
        {
            packSignalValue(data, 0, 1, canDataManager.txData->EmergencyAlarm, false);
            break;
        }
        case Motor_WheelAngle :
        {
            canDataManager.txData->WheelAngle = (uint8_t)RPM_CMD2 - (uint8_t)RPM_CMD1;
            packSignalValue(data, 0, 8, canDataManager.txData->WheelAngle, false);
        }
            break;
        case Motor_LaneChange :
        {
//            if (carState.lanefunc == LANE_CHANGE && carState.mode == DRIVING_TURNING_RIGHT)
//            {
//                canDataManager.txData->LaneChange = 1;
//            }
            if (carState.lanefunc == LANE_CHANGE && carState.mode == DRIVING_TURNING_LEFT)
            {
                canDataManager.txData->LaneChange = 2;
            }
            else if (carState.emergencyBrake == TRUE)
            {
                canDataManager.txData->LaneChange = 3;
            }
            packSignalValue(data, 0, 2, canDataManager.txData->LaneChange, false);
        }
            break;
        default :
            break;
    }

// CAN 메시지 송신
    IfxMultican_Message_init(&g_multican.txMsg, messageId, *(uint32_t*) &data[0], *(uint32_t*) &data[4], 8);
    while (IfxMultican_Status_notSentBusy
            == IfxMultican_Can_MsgObj_sendMessage(&g_multican.canSrcMsgObj, &g_multican.txMsg))
    {
    }
}
