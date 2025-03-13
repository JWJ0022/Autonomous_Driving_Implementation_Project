#ifndef MYCAN_INIT_H_
#define MYCAN_INIT_H_

#include "IfxMultican_Can.h"
#include "IfxMultican.h"
#include "Ifx_Types.h"
#include "IfxPort.h"

#define SRC_MESSAGE_OBJECT_ID       (IfxMultican_MsgObjId)0     /* Source message object ID                          */
#define DST_MESSAGE_OBJECT_ID       (IfxMultican_MsgObjId)1     /* Destination message object ID                     */
#define TX_INTERRUPT_SRC_ID         IfxMultican_SrcId_0         /* Transmit interrupt service request ID             */
#define RX_INTERRUPT_SRC_ID         IfxMultican_SrcId_1         /* Receive interrupt service request ID              */
#define ISR_PRIORITY_CAN_TX         8                           /* Define the CAN TX interrupt priority              */
#define ISR_PRIORITY_CAN_RX         7                           /* Define the CAN RX interrupt priority              */

typedef struct
{
        IfxMultican_Can                 can;                   /* CAN module handle to HW module SFR set                 */
        IfxMultican_Can_Config          canConfig;             /* CAN module configuration structure                     */
        IfxMultican_Can_Node            canTxNode;            /* CAN source node handle data structure                  */
        IfxMultican_Can_Node            canRxNode;            /* CAN destination node handle data structure             */
        IfxMultican_Can_NodeConfig      canNodeConfig;         /* CAN node configuration structure                       */
        IfxMultican_Can_MsgObj          canSrcMsgObj;          /* CAN source message object handle data structure        */
        IfxMultican_Can_MsgObj          canDstMsgObj;          /* CAN destination message object handle data structure   */
        IfxMultican_Can_MsgObjConfig    canMsgObjConfig;       /* CAN message object configuration structure             */
        IfxMultican_Message             txMsg;                 /* Transmitted CAN message structure                      */
        IfxMultican_Message             rxMsg;                 /* Received CAN message structure                         */
} AppMulticanType;

extern AppMulticanType g_multican;

void initMultican(void);

#endif /* MYCAN_INIT_H_ */
