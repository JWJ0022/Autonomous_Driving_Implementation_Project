/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "BSW_PID/Encoder.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
// 엔코더 설정
#define PULSES_PER_REV 330     // 한 채널당 펄스 수
const uint32 CPR = (PULSES_PER_REV * 4);  // 4체배시 한바퀴 펄스 수

// Interrupt priority definitions
#define ISR_PRIORITY_INCRENC_ZERO 6
#define Motor_B_ISR_PRIORITY_INCRENC_ZERO 7

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxGpt12_IncrEnc_Config gpt12Config;
IfxGpt12_IncrEnc gpt12;

IfxGpt12_IncrEnc_Config Motor_B_gpt12Config;
IfxGpt12_IncrEnc Motor_B_gpt12;

float32 speed;
sint32 rawPosition;
IfxStdIf_Pos_Dir direction;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
void initIncrEnc(void)
{
    // Initialize global clocks
    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_8);
    IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt2BlockPrescaler_4);

    // Create module config
//    IfxGpt12_IncrEnc_Config gpt12Config;
    IfxGpt12_IncrEnc_initConfig(&gpt12Config, &MODULE_GPT120);
    IfxGpt12_IncrEnc_initConfig(&Motor_B_gpt12Config, &MODULE_GPT120);

    // Configure Motor_A encoder parameters
    gpt12Config.base.offset               = 100;                    // Initial position offset
    gpt12Config.base.reversed             = FALSE;               // Count direction not reversed
    gpt12Config.base.resolution           = PULSES_PER_REV;                // Encoder resolution
    gpt12Config.base.periodPerRotation    = 1;                   // Number of periods per rotation
    gpt12Config.base.resolutionFactor     = IfxStdIf_Pos_ResolutionFactor_fourFold;  // Quadrature mode
    gpt12Config.base.updatePeriod         = 0.001;              // 1ms update period
    gpt12Config.base.speedModeThreshold   = 100;                // Threshold for speed calculation mode
    gpt12Config.base.minSpeed             = 10;                 // Minimum speed in rpm
    gpt12Config.base.maxSpeed             = 5000;                // Maximum speed in rpm

    // Configure Motor_B encoder parameters
    Motor_B_gpt12Config.base.offset               = 100;                    // Initial position offset
    Motor_B_gpt12Config.base.reversed             = FALSE;               // Count direction not reversed
    Motor_B_gpt12Config.base.resolution           = PULSES_PER_REV;                // Encoder resolution
    Motor_B_gpt12Config.base.periodPerRotation    = 1;                   // Number of periods per rotation
    Motor_B_gpt12Config.base.resolutionFactor     = IfxStdIf_Pos_ResolutionFactor_fourFold;  // Quadrature mode
    Motor_B_gpt12Config.base.updatePeriod         = 0.001;              // 1ms update period
    Motor_B_gpt12Config.base.speedModeThreshold   = 100;                // Threshold for speed calculation mode
    Motor_B_gpt12Config.base.minSpeed             = 10;                 // Minimum speed in rpm
    Motor_B_gpt12Config.base.maxSpeed             = 5000;                // Maximum speed in rpm

    // Configure Motor_A pins
    gpt12Config.pinA = &IfxGpt120_T2INA_P00_7_IN;     // Encoder A signal -> T3IN
    gpt12Config.pinB = &IfxGpt120_T2EUDA_P00_8_IN;    // Encoder B signal -> T3EUD
    gpt12Config.pinZ = NULL;                          // No Z signal used
    gpt12Config.pinMode = IfxPort_InputMode_pullDown;   // Use internal pullup

    // Configure Motor_B pins
    Motor_B_gpt12Config.pinA = &IfxGpt120_T3INB_P10_4_IN;     // Encoder A signal -> T3IN
    Motor_B_gpt12Config.pinB = &IfxGpt120_T3EUDB_P10_7_IN;;    // Encoder B signal -> T3EUD
    Motor_B_gpt12Config.pinZ = NULL;                          // No Z signal used
    Motor_B_gpt12Config.pinMode = IfxPort_InputMode_pullDown;   // Use internal pullup

    // Configure Motor_A interrupts
    gpt12Config.zeroIsrPriority = ISR_PRIORITY_INCRENC_ZERO;
    gpt12Config.zeroIsrProvider = IfxSrc_Tos_cpu0;

    // Configure Motor_B interrupts
    Motor_B_gpt12Config.zeroIsrPriority = Motor_B_ISR_PRIORITY_INCRENC_ZERO;
    Motor_B_gpt12Config.zeroIsrProvider = IfxSrc_Tos_cpu0;

    // Enable Motor_A speed filter
    gpt12Config.base.speedFilterEnabled = TRUE;
    gpt12Config.base.speedFilerCutOffFrequency = gpt12Config.base.maxSpeed / 2 * IFX_PI * 2;

    // Enable Motor_B speed filter
    Motor_B_gpt12Config.base.speedFilterEnabled = TRUE;
    Motor_B_gpt12Config.base.speedFilerCutOffFrequency = gpt12Config.base.maxSpeed / 2 * IFX_PI * 2;

    // Initialize Motor_A module
    IfxGpt12_IncrEnc_init(&gpt12, &gpt12Config);

    // Initialize Motor_B module
    IfxGpt12_IncrEnc_init(&Motor_B_gpt12, &Motor_B_gpt12Config);
}

void Encoder_update(void)
{
    IfxGpt12_IncrEnc_update(&gpt12);
    IfxGpt12_IncrEnc_update(&Motor_B_gpt12);
    //    speed = IfxGpt12_IncrEnc_getSpeed(&gpt12);
//    rawPosition = IfxGpt12_IncrEnc_getRawPosition(&gpt12);
//    direction = IfxGpt12_IncrEnc_getDirection(&gpt12);
}
