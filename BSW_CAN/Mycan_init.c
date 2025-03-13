#include <BSW_CAN/Mycan_init.h>

AppMulticanType g_multican;  // Global MULTICAN configuration structure

void initMultican (void)
{
    /* CAN 모듈 설정 */
    IfxMultican_Can_initModuleConfig(&g_multican.canConfig, &MODULE_CAN);

    g_multican.canConfig.nodePointer[TX_INTERRUPT_SRC_ID].priority = ISR_PRIORITY_CAN_TX;
    g_multican.canConfig.nodePointer[RX_INTERRUPT_SRC_ID].priority = ISR_PRIORITY_CAN_RX;

    IfxMultican_Can_initModule(&g_multican.can, &g_multican.canConfig);

    /* 노드 설정 Tx */
    IfxMultican_Can_Node_initConfig(&g_multican.canNodeConfig, &g_multican.can);
    g_multican.canNodeConfig.loopBackMode = FALSE;
    g_multican.canNodeConfig.nodeId = IfxMultican_NodeId_0; // NodeId_0 설정

    /* 포트 설정 */
    /* Rx Pin 세팅 */
    g_multican.canNodeConfig.rxPin = &IfxMultican_RXD0B_P20_7_IN;
    g_multican.canNodeConfig.rxPinMode = IfxPort_InputMode_pullUp;

    /* Tx Pin 세팅 */
    g_multican.canNodeConfig.txPin = &IfxMultican_TXD0_P20_8_OUT;
    g_multican.canNodeConfig.txPinMode = IfxPort_OutputMode_pushPull;
    g_multican.canNodeConfig.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed2;

    g_multican.canNodeConfig.baudrate = 500000;

    IfxMultican_Can_Node_init(&g_multican.canTxNode, &g_multican.canNodeConfig);

    /* 노드 설정 Rx */
    IfxMultican_Can_Node_initConfig(&g_multican.canNodeConfig, &g_multican.can);

    g_multican.canNodeConfig.loopBackMode = FALSE;
    g_multican.canNodeConfig.nodeId = IfxMultican_NodeId_0;

    IfxMultican_Can_Node_init(&g_multican.canRxNode, &g_multican.canNodeConfig);

    /* Rx, Tx Node는 같은 걸 써도 메세지객체 ID 는 송신/수신 분리해야됨 */
    /* 송신 메시지 객체 설정 */

    IfxMultican_Can_MsgObj_initConfig(&g_multican.canMsgObjConfig, &g_multican.canTxNode);

    g_multican.canMsgObjConfig.msgObjId = SRC_MESSAGE_OBJECT_ID; // 송신 메시지 객체 ID
    g_multican.canMsgObjConfig.frame = IfxMultican_Frame_transmit;
    g_multican.canMsgObjConfig.txInterrupt.enabled = TRUE; // 송신 완료 인터럽트 활성화
    g_multican.canMsgObjConfig.txInterrupt.srcId = TX_INTERRUPT_SRC_ID;

    IfxMultican_Can_MsgObj_init(&g_multican.canSrcMsgObj, &g_multican.canMsgObjConfig);

    /* 수신 메시지 객체 설정 */
    IfxMultican_Can_MsgObj_initConfig(&g_multican.canMsgObjConfig, &g_multican.canRxNode);

    g_multican.canMsgObjConfig.messageId = 0x0;
    g_multican.canMsgObjConfig.acceptanceMask = 0x0;

    g_multican.canMsgObjConfig.msgObjId = DST_MESSAGE_OBJECT_ID; // 수신 메시지 객체 ID
    g_multican.canMsgObjConfig.frame = IfxMultican_Frame_receive;
    g_multican.canMsgObjConfig.rxInterrupt.enabled = TRUE; // 송신 완료 인터럽트 활성화
    g_multican.canMsgObjConfig.rxInterrupt.srcId = RX_INTERRUPT_SRC_ID;

    IfxMultican_Can_MsgObj_init(&g_multican.canDstMsgObj, &g_multican.canMsgObjConfig);
}
