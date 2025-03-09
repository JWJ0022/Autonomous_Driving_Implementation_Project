/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "ASW_Lane_Change/lane_change.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define ISR_PRIORITY_LC_T1_STM      55
#define ISR_PRIORITY_LC_T2_STM      57
#define ISR_PRIORITY_LC_T3_STM      59
#define LANE_WIDTH                  250

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxStm_CompareConfig LC_T1_STMConf;
IfxStm_CompareConfig LC_T2_STMConf;
IfxStm_CompareConfig LC_T3_STMConf;

static const float32 TARGET_ANGLE_DEGREE = 70.0f; //단위 : 도
static const float32 TARGET_ANGLE_RADIAN = 70.0f * (IFX_PI/180); //단위 : 라디안
static const float32 ANGULAR_VELOCITY_DEGREE = 30.0f; //단위 : 도/초
static const float32 ANGULAR_VELOCITY_RADIAN = 30.0f * (IFX_PI/180); //단위 : 라디안/초

uint32 rotation_time_T1_ticks;
float32 turn_radius = 0;

float32 rotation_time_T1;
float32 travel_time_T2 = 0;

float32 origin_left_wheelspeed = 0;
float32 origin_right_wheelspeed = 0;

int it_dist_FLAG = 0;
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
void T1_rotation_Right(void)
{
    carState.lanefunc = LANE_CHANGE;
    carState.mode = DRIVING_TURNING_RIGHT;

    origin_left_wheelspeed = carState.leftWheelSpeed;
    origin_right_wheelspeed = carState.rightWheelSpeed;

    float32 Base_Speed = getCurrentSpeed();
    carState.baseSpeed = Base_Speed;

    set_turn(Right, ANGULAR_VELOCITY_RADIAN);
}

// 전역 변수 (플래그 추가)
boolean RightOffsetFlag1 = FALSE;  // RightOffset이 -100 이하로 떨어지면 ON
void checkRightOffsetChange(void)
{
    //차선 변경 중일 때만 실행
    if (carState.lanefunc == LANE_CHANGE)
    {
        //RightOffset이 -100 이하로 떨어지면 flag1을 ON
        if (RightOffset < -100)
        {
            RightOffsetFlag1 = TRUE;
        }

        //flag1이 ON 상태에서 RightOffset이 양수가 되면 차선 변경 완료
        if (RightOffsetFlag1 == TRUE && RightOffset > 0)
        {
            carState.lanefunc = LANE_KEEPING;  // 🚗 차선 유지 모드로 변경
            RightOffsetFlag1 = FALSE;  // 플래그 초기화
        }
    }
}

