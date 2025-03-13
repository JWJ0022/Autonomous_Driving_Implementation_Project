#ifndef CAN_DEFINITIONS_H_
#define CAN_DEFINITIONS_H_

#include <stdint.h>
#include <stdbool.h>
#include "Platform_Types.h"

#define CAN_QUEUE_SIZE 10

typedef struct
{
    uint32_t messageId;
    uint8_t data[8];
} CanQueueItem;

/* CAN 메시지 정의 구조체 */
typedef struct
{
    uint32_t messageId; /**< CAN 메시지 ID */
    uint8_t dlc; /**< 데이터 길이 */
    const char *name; /**< 메시지 이름 */
} CanMessage;

/* CAN 신호 정의 구조체 */
typedef struct
{
    const char *name; /**< 신호 이름 */
    uint8_t startBit; /**< 신호 시작 비트 */
    uint8_t length; /**< 신호 길이 */
    bool isBigEndian; /**< Big Endian 여부 */
    bool isSigned; /**< Signed 여부 */
} CanSignal;

#define LKAS_HeadAngle 0x50
#define LKAS_Left 0x51
#define LKAS_Right 0x52

#define DMS_DrvCautionLv 0x100

#define Motor_BrakeSta 0x102 // 0: Default, 1: 그냥 브레이크, 2: 급브레이크
#define Motor_EmergencyAlarm 0x104 // 갓길에 안전정차 완료화면 쏘라 to do 15:00
#define Motor_WheelAngle 0x105 // 250222 21:00 100ms 쏘라
#define Motor_LaneChange 0x108 // 0: default, 1: 우깜. 2: 좌깜

#define Motor_VehicleSpeed 0x110

/* CAN 메시지 정의 */
extern const CanMessage canMessages_Tx[];
extern const CanMessage canMessages_Rx[];

/* CAN 신호 정의 */
extern const CanSignal canSignals_Tx[];
extern const CanSignal canSignals_Rx[];

typedef struct
{
    bool DrvCautionLv_flag;
    bool BrakeSta_flag;
    bool EmergencyAlarm_flag;
    bool WheelAngle_flag;
    bool LaneChange_flag;
    bool VehicleSpeed_flag;

    bool LKAS_HeadAngle_flag;
    bool LAKS_Right_flag;
    bool LAKS_Left_flag;
} CanSignalFlags;

typedef struct
{

    uint8_t BrakeSta;
    uint8_t EmergencyAlarm;
    uint8_t WheelAngle;
    uint8_t LaneChange;
    uint32_t VehicleSpeed;
} CanSignalData_Tx;

typedef struct
{
    uint8_t DrvCautionLv;
    sint16 HeadAngle;
    sint16 Right;
    sint16 Left;
} CanSignalData_Rx;

typedef struct
{
    CanSignalData_Rx *rxData;
    CanSignalData_Tx *txData;
    CanSignalFlags *flags;
} CanDataManager;

/* 전역 데이터 구조체 포인터 */
extern CanDataManager canDataManager;

/* CAN 메시지 큐 */
extern CanQueueItem canMessageQueue[CAN_QUEUE_SIZE];
extern uint8_t canQueueHead;
extern uint8_t canQueueTail;
extern volatile bool canDataUpdated;

/* 함수 프로토타입 */
void processCanMessage (uint32_t messageId, const uint8_t *data, CanDataManager *canData);
void packSignalValue (uint8_t *data, uint8_t startBit, uint8_t length, uint32_t value, bool isBigEndian);
uint32_t extractSignalValue (const uint8_t *data, uint8_t startBit, uint8_t length, bool isBigEndian);

#endif /* CAN_DEFINITIONS_H_ */
