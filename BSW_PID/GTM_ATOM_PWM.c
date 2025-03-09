/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include <BSW_PID/GTM_ATOM_PWM.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define ISR_PRIORITY_ATOM  20
#define Motor_A_PWM                IfxGtm_ATOM1_1_TOUT1_P02_1_OUT
#define Motor_B_PWM        IfxGtm_ATOM1_3_TOUT105_P10_3_OUT
#define PWM_OUT         IfxGtm_ATOM1_2_TOUT12_P00_3_OUT
#define PWM_PERIOD         1000
#define CLK_FREQ           1000000.0f
#define PWMA_PIN &MODULE_P02,1   // Motor_A PWM 핀 (P2.1)
#define BRAKEA_PIN &MODULE_P02,7 // Motor_A 브레이크 핀 (P2.7)
#define DIRA_PIN &MODULE_P10,1   // Motor_A 방향 제어 핀 (P10.1)
#define Motor_B_PWMA_PIN &MODULE_P10,3 //Motor_B PWM 핀(P10,3)
#define Motor_B_BRAKE_PIN &MODULE_P02,6 //Motor_B 브레이크 핀(P2.6)
#define Motor_B_DIR_PIN &MODULE_P10,2   //Motor_B 방향 제어 핀(P10.2)

#define ULT_PWM_PERIOD  100000
#define DUTY_CYCLE      ULT_PWM_PERIOD/2000

#define PWM_IN          IfxGtm_TIM2_3_TIN13_P00_4_IN     /* Input port pin for the PWM signal                        */
#define SIDE_PWM_IN     IfxGtm_TIM2_4_TIN14_P00_5_IN

#define secTous 1000000

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxGtm_Atom_Pwm_Config g_atomConfig;                            /* Timer configuration structure                    */
IfxGtm_Atom_Pwm_Config Motor_B_g_atomConfig;
IfxGtm_Atom_Pwm_Driver g_atomDriver;                            /* Timer Driver structure                           */
IfxGtm_Atom_Pwm_Driver Motor_B_g_atomDriver;
IfxGtm_Atom_Pwm_Config u_atomConfig;
IfxGtm_Atom_Pwm_Driver u_atomDriver;
uint32 g_PWMValue = 0;                                         /* Initialization of the fade value                 */



IfxGtm_Tim_In_Config configTIM;
IfxGtm_Tim_In_Config sideTIM;

float32 g_measuredPwmDutyCycle = 0.0;                   /* Global variable for duty cycle of generated PWM signal   */
float32 g_measuredPwmFreq_Hz = 0.0;                     /* Global variable for frequency calculation of PWM signal  */
float32 g_measuredPwmPeriod = 0.0;                      /* Global variable for period calculation of PWM signal     */
IfxGtm_Tim_In g_driverTIM;                              /* TIM driver structure                                     */
boolean g_dataCoherent = FALSE;                         /* Boolean to know if the measured data is coherent         */
float32 b_pwm_period_us;
float32 b_pwm_sec_us;

float32 s_measuredPwmDutyCycle = 0.0;
float32 s_measuredPwmFreq_Hz = 0.0;
float32 s_measuredPwmPeriod = 0.0;
IfxGtm_Tim_In s_driverTIM;
boolean s_dataCoherent = FALSE;
float32 s_pwm_period_us;
float32 s_pwm_sec_us;


/*********************************************************************************************************************/
/*-----------------------------------------------Function Prototypes-------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Function Implementations-----------------------------------------------*/
/*********************************************************************************************************************/
/* This function initializes the ATOM */
void initGtmATomPwm(void)
{
    IfxGtm_enable(&MODULE_GTM); /* Enable GTM */

    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, CLK_FREQ);        /* Set the CMU clock 0 frequency    */
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);                /* Enable the CMU clock 0           */

    IfxGtm_Atom_Pwm_initConfig(&g_atomConfig, &MODULE_GTM);                     /* Initialize default parameters    */
    IfxGtm_Atom_Pwm_initConfig(&Motor_B_g_atomConfig, &MODULE_GTM);
    IfxGtm_Atom_Pwm_initConfig(&u_atomConfig, &MODULE_GTM);

    IfxGtm_Tim_In_initConfig(&configTIM, &MODULE_GTM);
    IfxGtm_Tim_In_initConfig(&sideTIM, &MODULE_GTM);

    g_atomConfig.atom = Motor_A_PWM.atom;                                       /* Select the ATOM depending on the LED     */
    g_atomConfig.atomChannel = Motor_A_PWM.channel;                             /* Select the channel depending on the LED  */
    g_atomConfig.period = PWM_PERIOD;                                   /* Set timer period                         */
    g_atomConfig.pin.outputPin = &Motor_A_PWM;                                  /* Set LED as output                        */
    g_atomConfig.synchronousUpdateEnabled = TRUE;                       /* Enable synchronous update                */

    Motor_B_g_atomConfig.atom = Motor_B_PWM.atom;
    Motor_B_g_atomConfig.atomChannel = Motor_B_PWM.channel;
    Motor_B_g_atomConfig.period = PWM_PERIOD;
    Motor_B_g_atomConfig.pin.outputPin = &Motor_B_PWM;
    Motor_B_g_atomConfig.synchronousUpdateEnabled = TRUE;

    u_atomConfig.atom = PWM_OUT.atom;                                         /* Select the ATOM depending on the LED     */
    u_atomConfig.atomChannel = PWM_OUT.channel;                               /* Select the channel depending on the LED  */
    u_atomConfig.period = ULT_PWM_PERIOD;                                 /* Set timer period                         */
    u_atomConfig.pin.outputPin = &PWM_OUT;                                    /* Set LED as output                        */
    u_atomConfig.synchronousUpdateEnabled = TRUE;                           /* Enable synchronous update                */
    u_atomConfig.dutyCycle = DUTY_CYCLE;

    configTIM.filter.inputPin = &PWM_IN;
    configTIM.filter.inputPinMode = IfxPort_InputMode_pullDown;

    sideTIM.filter.inputPin = &SIDE_PWM_IN;
    sideTIM.filter.inputPinMode = IfxPort_InputMode_pullDown;


    IfxGtm_Atom_Pwm_init(&g_atomDriver, &g_atomConfig);
    IfxGtm_Atom_Pwm_start(&g_atomDriver, TRUE);

    IfxGtm_Atom_Pwm_init(&Motor_B_g_atomDriver, &Motor_B_g_atomConfig);
    IfxGtm_Atom_Pwm_start(&Motor_B_g_atomDriver, TRUE);

    IfxGtm_Atom_Pwm_init(&u_atomDriver, &u_atomConfig);
    IfxGtm_Atom_Pwm_start(&u_atomDriver, TRUE);

    IfxGtm_Tim_In_init(&g_driverTIM, &configTIM);
    IfxGtm_Tim_In_init(&s_driverTIM, &sideTIM);
}

void PWM_set(uint32 g_PWMValue)
{
    if(g_PWMValue >= PWM_PERIOD)
    {
        g_PWMValue = PWM_PERIOD; /* Set the direction of the fade */
    }
    else if(g_PWMValue <= 0)
    {
        g_PWMValue = 0;  /* Set the direction of the fade */
    }
    setDutyCycle(g_PWMValue); /* Call the function which is setting the duty cycle of the PWM */
}

void PWM_Motor_B_set(uint32 g_PWMValue)
{
    if(g_PWMValue >= PWM_PERIOD)
    {
        g_PWMValue = PWM_PERIOD; /* Set the direction of the fade */
    }
    else if(g_PWMValue <= 0)
    {
        g_PWMValue = 0;  /* Set the direction of the fade */
    }
    setMotor_B_DutyCycle(g_PWMValue); /* Call the function which is setting the duty cycle of the PWM */
}

/* This function sets the duty cycle of the PWM */
void setDutyCycle(uint32 dutyCycle)
{
    g_atomConfig.dutyCycle = dutyCycle;                 /* Set duty cycle        */
    IfxGtm_Atom_Pwm_init(&g_atomDriver, &g_atomConfig); /* Re-initialize the PWM */
}

void setMotor_B_DutyCycle(uint32 dutyCycle)
{
    Motor_B_g_atomConfig.dutyCycle = dutyCycle;
    IfxGtm_Atom_Pwm_init(&Motor_B_g_atomDriver, &Motor_B_g_atomConfig);
}

// 모터 제어 함수
void setMotorControl(uint8 direction, uint8 enable)
{
    // 브레이크 설정
    if (enable == 0)
    {
        IfxPort_setPinState(BRAKEA_PIN, IfxPort_State_high); // 브레이크 활성화

        // PWM 출력 중지
        GTM_ATOM1_AGC_GLB_CTRL.B.UPEN_CTRL1 = 0;

        return;
    }
    else
    {
        IfxPort_setPinState(BRAKEA_PIN, IfxPort_State_low); // 브레이크 비활성화

        //GTM_TOM0_TGC0_GLB_CTRL.B.UPEN_CTRL1 = 2;
        GTM_ATOM1_AGC_GLB_CTRL.B.UPEN_CTRL1 = 2;
    }

    // 방향 설정
    if (direction == 0)
    {
        IfxPort_setPinState(DIRA_PIN, IfxPort_State_low); // 정방향
    }
    else
    {
        IfxPort_setPinState(DIRA_PIN, IfxPort_State_high); // 역방향
    }
}

void setMotor_B_Control(uint8 direction, uint8 enable)
{
    // 브레이크 설정
    if (enable == 0)
    {
        IfxPort_setPinState(Motor_B_BRAKE_PIN, IfxPort_State_high); // 브레이크 활성화

        // PWM 출력 중지
        GTM_ATOM1_AGC_GLB_CTRL.B.UPEN_CTRL3 = 0;

        return;
    }
    else
    {
        IfxPort_setPinState(Motor_B_BRAKE_PIN, IfxPort_State_low); // 브레이크 비활성화

        //GTM_TOM0_TGC0_GLB_CTRL.B.UPEN_CTRL1 = 2;
        GTM_ATOM1_AGC_GLB_CTRL.B.UPEN_CTRL3 = 2;
    }

    // 방향 설정
    if (direction == 0)
    {
        IfxPort_setPinState(Motor_B_DIR_PIN, IfxPort_State_low); // 정방향
    }
    else
    {
        IfxPort_setPinState(Motor_B_DIR_PIN, IfxPort_State_high); // 역방향
    }
}

void initPins(void)
{
    // 방향 핀 초기화
    IfxPort_setPinMode(DIRA_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(DIRA_PIN, IfxPort_State_low);

    IfxPort_setPinMode(Motor_B_DIR_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(Motor_B_DIR_PIN, IfxPort_State_low);

    // 브레이크 핀 초기화
    IfxPort_setPinMode(BRAKEA_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(BRAKEA_PIN, IfxPort_State_low);

    IfxPort_setPinMode(Motor_B_BRAKE_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(Motor_B_BRAKE_PIN, IfxPort_State_low);
}

float32 measure_PWM(void)
{
    IfxGtm_Tim_In_update(&g_driverTIM);                                         /* Update the measured data         */
    g_measuredPwmPeriod = IfxGtm_Tim_In_getPeriodSecond(&g_driverTIM);          /* Get the period of the PWM signal */
    g_measuredPwmFreq_Hz = 1 / g_measuredPwmPeriod;                             /* Calculate the frequency          */
    g_measuredPwmDutyCycle = IfxGtm_Tim_In_getDutyPercent(&g_driverTIM, &g_dataCoherent); /* Get the duty cycle     */

    b_pwm_period_us = g_measuredPwmPeriod * secTous;
    b_pwm_sec_us = b_pwm_period_us * (g_measuredPwmDutyCycle / 100);

    return b_pwm_sec_us;
}

float32 side_PWM(void) {
    IfxGtm_Tim_In_update(&s_driverTIM);                                         /* Update the measured data         */
    s_measuredPwmPeriod = IfxGtm_Tim_In_getPeriodSecond(&s_driverTIM);          /* Get the period of the PWM signal */
    s_measuredPwmFreq_Hz = 1 / s_measuredPwmPeriod;                             /* Calculate the frequency          */
    s_measuredPwmDutyCycle = IfxGtm_Tim_In_getDutyPercent(&s_driverTIM, &s_dataCoherent); /* Get the duty cycle     */

    s_pwm_period_us = s_measuredPwmPeriod * secTous;
    s_pwm_sec_us = s_pwm_period_us * (s_measuredPwmDutyCycle / 100);

    return s_pwm_sec_us;
}
