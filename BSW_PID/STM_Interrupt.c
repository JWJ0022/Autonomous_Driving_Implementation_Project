/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include <BSW_PID/STM_Interrupt.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define ISR_PRIORITY_STM        70                              /* Priority for interrupt ISR                       */
#define TIMER_INT_TIME          7                             /* Time between interrupts in ms                    */
#define STM                     &MODULE_STM0                    /* STM0 is used in this example                     */
#define GEAR_RATIO              30
#define Maximum_RPM             300
#define Minimum_RPM             -300
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
float32 s_T_samp= 0.001*TIMER_INT_TIME;
float32 Maximum_RPM_multiplied_by_reduction_ratio = Maximum_RPM * GEAR_RATIO;
float32 Minimum_RPM_multiplied_by_reduction_ratio = Minimum_RPM * GEAR_RATIO;
float32 RPM_CMD1 = 0;
float32 RPM_CMD2 = 0;
sint32 Enc_count_new = 0;
sint32 Motor_B_Enc_count_new = 0;
sint32 Enc_count_old = 0;
sint32 Motor_B_Enc_count_old = 0;
float32 Enc_count_diff = 0;
float32 Motor_B_Enc_count_diff = 0;
float32 motor_speed_rpm=0;
float32 Motor_B_motor_speed_rpm=0;
sint32 i1 = 0;
PIDREG3 speed_pid = PIDREG3_DEFAULTS;
PIDREG3 Motor_B_speed_pid = PIDREG3_DEFAULTS;
volatile float32 Kp_s=1.0,Ki_s=0.31004,Kd_s=0;
boolean STM_IT_FLAG = FALSE;
IfxStm_CompareConfig g_STMConf;

Ifx_TickTime g_ticksFor5ms;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
IFX_INTERRUPT(isrSTM, 0, ISR_PRIORITY_STM);

void RPM_cal(void)
{
    Encoder_update();   //지워도 되는지 확인할것

    Enc_count_new = gpt12Config.module->T2.U;
    Motor_B_Enc_count_new = Motor_B_gpt12Config.module->T3.U;

    Enc_count_diff = (float32)(Enc_count_new - Enc_count_old);
    Motor_B_Enc_count_diff = (float32)(Motor_B_Enc_count_new - Motor_B_Enc_count_old);

    motor_speed_rpm = Enc_count_diff/(float32)CPR/(float32)(TIMER_INT_TIME*0.001)*60.0f;
    Motor_B_motor_speed_rpm = Motor_B_Enc_count_diff/(float32)CPR/(float32)(TIMER_INT_TIME*0.001)*60.0f;

    Enc_count_old = Enc_count_new;
    Motor_B_Enc_count_old = Motor_B_Enc_count_new;
}

void PI_const_update(void)
{
    speed_pid.Kp = Kp_s;
    speed_pid.Ki = Ki_s;
    speed_pid.Kd = Kd_s;
    speed_pid.Kc = 1/Kp_s;
    speed_pid.T_samp = s_T_samp;
    speed_pid.OutMax = Maximum_RPM_multiplied_by_reduction_ratio;
    speed_pid.OutMin = Minimum_RPM_multiplied_by_reduction_ratio;

    Motor_B_speed_pid.Kp = Kp_s;
    Motor_B_speed_pid.Ki = Ki_s;
    Motor_B_speed_pid.Kd = Kd_s;
    Motor_B_speed_pid.Kc = 1/Kp_s;
    Motor_B_speed_pid.T_samp = s_T_samp;
    Motor_B_speed_pid.OutMax = Maximum_RPM_multiplied_by_reduction_ratio;
    Motor_B_speed_pid.OutMin = Minimum_RPM_multiplied_by_reduction_ratio;
}

void PI_Speed_con(void)
{
    PI_const_update();
    if(RPM_CMD1==0)
    {
        speed_pid.reset((void *)&speed_pid);
        //Motor_B_speed_pid.reset((void *)&Motor_B_speed_pid); //!원본

        setMotorControl(0,0);
    }
    else
    {
        speed_pid.Ref= - RPM_CMD1*GEAR_RATIO;   // speed reference
        speed_pid.Fdb= motor_speed_rpm; // speed measured by ENC
        speed_pid.calc((void *)&speed_pid);   // Calculate speed PID Controller
    }
}

void PI_Speed_con_B(void)
{
    PI_const_update();
    if(RPM_CMD2==0)
    {
        Motor_B_speed_pid.reset((void *)&Motor_B_speed_pid);

        setMotor_B_Control(0,0);
    }
    else
    {
        Motor_B_speed_pid.Ref=RPM_CMD2*GEAR_RATIO;   // speed reference
        Motor_B_speed_pid.Fdb= Motor_B_motor_speed_rpm; // speed measured by ENC
        Motor_B_speed_pid.calc((void *)&Motor_B_speed_pid);   // Calculate speed PID Controller
    }
}

void isrSTM(void)
{
    STM_IT_FLAG = TRUE;
}

void initSTM(void)
{
    IfxStm_initCompareConfig(&g_STMConf);

    g_STMConf.triggerPriority = ISR_PRIORITY_STM;
    g_STMConf.typeOfService = IfxSrc_Tos_cpu0;
    g_STMConf.ticks = (uint32)g_ticksFor5ms;
    IfxStm_initCompare(STM, &g_STMConf);
}

void initPeripherals(void)
{
    g_ticksFor5ms = IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, TIMER_INT_TIME);
    initSTM();
}
