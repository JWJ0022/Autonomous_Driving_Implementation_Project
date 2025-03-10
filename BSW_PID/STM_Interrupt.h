#ifndef BSW_PID_STM_INTERRUPT_H_
#define BSW_PID_STM_INTERRUPT_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"
#include "Bsp.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "IfxGpt12_IncrEnc.h"

#include <BSW_PID/Encoder.h>
#include <BSW_PID/GTM_ATOM_PWM.h>
#include <BSW_PID/PID_CON.h>
#include <BSW_PID/STM_Interrupt.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
extern float32 RPM_CMD1;
extern float32 RPM_CMD2;
extern float32 Maximum_RPM_multiplied_by_reduction_ratio;
extern float32 Minimum_RPM_multiplied_by_reduction_ratio;
extern float32 motor_speed_rpm;
extern float32 Motor_B_motor_speed_rpm;
extern IfxGpt12_IncrEnc_Config gpt12Config;
extern IfxGpt12_IncrEnc_Config Motor_B_gpt12Config;
extern uint8 CPR;
extern boolean STM_IT_FLAG;
extern PIDREG3 speed_pid;
extern PIDREG3 Motor_B_speed_pid;
extern IfxStm_CompareConfig g_STMConf;
extern Ifx_TickTime g_ticksFor5ms;

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void RPM_cal(void);
void PI_const_update(void);
void PI_Speed_con(void);
void isrSTM(void);
void initSTM(void);
void initPeripherals(void);
void PI_Speed_con_B(void);


#endif /* BSW_PID_STM_INTERRUPT_H_ */
