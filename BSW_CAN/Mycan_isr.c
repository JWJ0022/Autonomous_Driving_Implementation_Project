#include <BSW_CAN/CanDefinitions.h>
#include <BSW_CAN/Mycan_init.h>
#include <BSW_CAN/Mycan_isr.h>
#include "Bsp.h"
#include <stdint.h>

IFX_INTERRUPT(canIsrTxHandler, 0, ISR_PRIORITY_CAN_TX);
IFX_INTERRUPT(canIsrRxHandler, 0, ISR_PRIORITY_CAN_RX);

CanQueueItem canMessageQueue[CAN_QUEUE_SIZE];
uint8_t canQueueHead = 0;
uint8_t canQueueTail = 0;
volatile bool canDataUpdated = false; // ✅ 데이터 수신 플래그

void canIsrTxHandler (void)
{
    
}

void canIsrRxHandler (void)
{
    //IfxPort_togglePin(&MODULE_P10,2);

    IfxMultican_Status readStatus;
    uint8_t data[8];

    // CAN 메시지 읽기
    readStatus = IfxMultican_Can_MsgObj_readMessage(&g_multican.canDstMsgObj, &g_multican.rxMsg);

    if (readStatus & IfxMultican_Status_newData)
    {
        // ✅ 큐에 메시지 추가
        canMessageQueue[canQueueTail].messageId = g_multican.rxMsg.id;
        memcpy(canMessageQueue[canQueueTail].data, g_multican.rxMsg.data, 8);

        // ✅ 큐 포인터 이동 (순환 큐 방식)
        canQueueTail = (canQueueTail + 1) % CAN_QUEUE_SIZE;
        if (canQueueTail == canQueueHead)
        {
            // 큐가 가득 차면, 가장 오래된 데이터를 덮어씀
            canQueueHead = (canQueueHead + 1) % CAN_QUEUE_SIZE;
        }

        // ✅ 데이터 수신 플래그 설정
        canDataUpdated = true;
    }
}
