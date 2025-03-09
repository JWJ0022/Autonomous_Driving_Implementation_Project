/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "ASW_FCA/FCA.h"
/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define STM1                           &MODULE_STM1
#define ISR_PRIORITY_FCA_STM           50
#define FCA_TIME_INTERVAL              100 //단위 : ms
#define Safety_Distance                150 //단위 : mm, 감속 시작 거리 450
#define STRAIGHT_MAX_SPEED             150
#define LANE_CHANGE_MAX_SPEED          300
#define LANE_KEEP_MAX_SPEED            250
#define MIN_SPEED                      40
#define SUDDEN_STOP_THRESHOLD          150 //단위 : mm

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxStm_CompareConfig FCA_STMConf;
Ifx_TickTime g_ticksFor50ms;
boolean FCA_IT_FLAG = FALSE;
boolean FCA_ENABLE_FLAG = TRUE;
boolean EMERGENCY_STOP_FLAG = FALSE;
boolean complete_EDSS = FALSE;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
IFX_INTERRUPT(isrFCA, 0, ISR_PRIORITY_FCA_STM);

void isrFCA(void)
{
    FCA_IT_FLAG = TRUE;
}

void initFCASTM(void)
{
    IfxStm_initCompareConfig(&FCA_STMConf);
    g_ticksFor50ms = IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, FCA_TIME_INTERVAL);

    FCA_STMConf.triggerPriority = ISR_PRIORITY_FCA_STM;
    FCA_STMConf.typeOfService = IfxSrc_Tos_cpu0;
    FCA_STMConf.ticks = (uint32)g_ticksFor50ms;
    IfxStm_initCompare(STM1, &FCA_STMConf);
}

void performFCA(void)
{
    FCA_IT_FLAG = FALSE;

    float32 Kp_FCA = 1.5f;
    float32 targetSpeed = 0;
    static uint32_t preDistance = 0;
    uint32 distanceChange = 0;

    //거리 계산
    uint32_t Front_Distance = get_kalman_val();
    distanceChange = Front_Distance - preDistance;

    //급정거
    if(distanceChange < -SUDDEN_STOP_THRESHOLD && Front_Distance < Safety_Distance)
    {
        carState.stop = TRUE;
        carState.emergencyBrake = TRUE;
        RPM_CMD1 = -RPM_CMD1;
        RPM_CMD2 = -RPM_CMD2;
    }
    //정지
    else if(Front_Distance <= Safety_Distance)
    {
        carState.stop = TRUE;
        targetSpeed = 0;
    }
    //거리에 따른 속도 비례 제어
    else if(Front_Distance > Safety_Distance)
    {
        if(complete_EDSS == FALSE)
        {
            carState.stop = FALSE;
        }
        carState.emergencyBrake = FALSE;
        targetSpeed = Kp_FCA * (Front_Distance - Safety_Distance);
    }

    //주행 모드에 따라 최대 속도 제한
    if(carState.mode == DRIVING_STRAIGHT)
    {
        if((carState.stop == FALSE) && (targetSpeed > STRAIGHT_MAX_SPEED))
        {
            targetSpeed = STRAIGHT_MAX_SPEED;
        }
        else if((carState.stop == FALSE) && (targetSpeed < MIN_SPEED))
        {
            targetSpeed = MIN_SPEED;
        }
    }
    else if(carState.lanefunc == LANE_KEEPING && carState.mode != DRIVING_STRAIGHT)
    {
        if((carState.stop == FALSE) && (targetSpeed > LANE_KEEP_MAX_SPEED))
        {
            targetSpeed = LANE_KEEP_MAX_SPEED;
        }
        else if((carState.stop == FALSE) && (targetSpeed < MIN_SPEED))
        {
            targetSpeed = MIN_SPEED;
        }
    }
    else if(carState.lanefunc == LANE_CHANGE && carState.mode != DRIVING_STRAIGHT)
    {
        if((carState.stop == FALSE) && (targetSpeed > LANE_CHANGE_MAX_SPEED))
        {
            targetSpeed = LANE_CHANGE_MAX_SPEED;
        }
        else if((carState.stop == FALSE) && (targetSpeed < MIN_SPEED))
        {
            targetSpeed = MIN_SPEED;
        }
    }

    //target speed 세팅
    //정지 상태면 0,0
    if(carState.stop == TRUE)
    {
        setTargetSpeed(0,0);
    }
    //직진 상태면 targetspeed, targetspeed
    else if(carState.mode == DRIVING_STRAIGHT)
    {
        setTargetSpeed(targetSpeed, targetSpeed);
    }
    //우회전 상태면 targetSpeed, targetSpeed * speedRatio
    else if(carState.mode == DRIVING_TURNING_RIGHT)
    {
        float32 speedRatio = (carState.leftWheelSpeed > 0) ? (carState.rightWheelSpeed / carState.leftWheelSpeed) : 0.8f;
        float32 leftSpeed = targetSpeed;
        float32 rightSpeed = targetSpeed * speedRatio;
        setTargetSpeed(leftSpeed, rightSpeed);
    }
    //좌회전 상태면 targetSpeed * speedRatio, targetSpeed
    else if(carState.mode == DRIVING_TURNING_LEFT)
    {
        float32 speedRatio = (carState.rightWheelSpeed > 0) ? (carState.leftWheelSpeed / carState.rightWheelSpeed) : 0.8f;
        float32 leftSpeed = targetSpeed * speedRatio;
        float32 rightSpeed = targetSpeed;
        setTargetSpeed(leftSpeed, rightSpeed);
    }

    preDistance = Front_Distance;

    IfxStm_increaseCompare(STM1, FCA_STMConf.comparator, (uint32)g_ticksFor50ms);
}
