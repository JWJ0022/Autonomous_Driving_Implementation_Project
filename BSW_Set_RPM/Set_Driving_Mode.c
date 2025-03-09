/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include <BSW_Set_RPM/Set_Driving_Mode.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define STM                     &MODULE_STM1
#define WHEEL_RADIUS            32.5f  // 바퀴 반지름 (단위: mm)
#define WHEEL_BASE              180.0f //휠 베이스 : 190mm
#define MIN_RADIUS              WHEEL_BASE*1.2f //최소회전반경 : 휠베이스 * 1.2
#define MAX_RADIUS              WHEEL_BASE*5.0f //최대회전반경 : 휠베이스 * 5
#define Maximum_RPM             300
#define Minimum_RPM             50
#define GEAR_RATIO              30
#define MAX_WHEEL_SPEED         150
#define MIN_WHEEL_SPEED         20


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
float32 CurrentTargetSpeed = 0;
float32 Target_A_Motor_Speed = 0;
float32 Target_B_Motor_Speed = 0;
static const float32 CAR_LENGTH = 135.0f;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/


/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
void Set_Stop(void)
{
    while(RPM_CMD1 > 0 || RPM_CMD2 > 0)
    {
        RPM_CMD1 -= 5;
        RPM_CMD2 -= 5;

        if(RPM_CMD1 < 0)
        {
            RPM_CMD1 = 0;

        }
        if(RPM_CMD2 < 0)
        {
            RPM_CMD2 = 0;
        }
    }

    setMotorControl(0, 0);
    setMotor_B_Control(0, 0);
}

void setTargetSpeed(float32 A_left_motor_speed, float32 B_right_motor_speed)
{
    Target_A_Motor_Speed = A_left_motor_speed;
    carState.leftWheelSpeed = Target_A_Motor_Speed;
    Target_B_Motor_Speed = B_right_motor_speed;
    carState.rightWheelSpeed = Target_B_Motor_Speed;
}

void UpdateRPMBasedOnSpeed(void)//단위 : mm/s
{
    //목표 RPM 계산
    float32 Target_A_Motor_cal_RPM = (Target_A_Motor_Speed * 60.0f)/(2.0f * IFX_PI * WHEEL_RADIUS);
    float32 Target_B_Motor_cal_RPM = (Target_B_Motor_Speed * 60.0f)/(2.0f * IFX_PI * WHEEL_RADIUS);
    float32 rampRate = 5.0f;

    //모터 A의 목표 RPM을 점진적으로 변경
    if (RPM_CMD1 < Target_A_Motor_cal_RPM)
    {
        RPM_CMD1 += rampRate;
        if (RPM_CMD1 > Target_A_Motor_cal_RPM) // 목표치를 초과하지 않도록 보정
            RPM_CMD1 = Target_A_Motor_cal_RPM;
    }
    else if (RPM_CMD1 > Target_A_Motor_cal_RPM)
    {
        RPM_CMD1 -= rampRate;
        if (RPM_CMD1 < Target_A_Motor_cal_RPM) // 목표치를 초과하지 않도록 보정
            RPM_CMD1 = Target_A_Motor_cal_RPM;
    }

    //모터 B의 목표 RPM을 점진적으로 변경
    if (RPM_CMD2 < Target_B_Motor_cal_RPM)
    {
        RPM_CMD2 += rampRate;
        if (RPM_CMD2 > Target_B_Motor_cal_RPM) // 목표치를 초과하지 않도록 보정
            RPM_CMD2 = Target_B_Motor_cal_RPM;
    }
    else if (RPM_CMD2 > Target_B_Motor_cal_RPM)
    {
        RPM_CMD2 -= rampRate;
        if (RPM_CMD2 < Target_B_Motor_cal_RPM) // 목표치를 초과하지 않도록 보정
            RPM_CMD2 = Target_B_Motor_cal_RPM;
    }
}

float32 set_turn(Direction direction, float32 target_angle_radian)
{
    float32 turn_radius = 0;
    float32 Base_Speed = 0;
    float32 innerWheelSpeed = 0;
    float32 outerWheelSpeed = 0;

    //RPM_CMD1, RPM_CMD2의 평균으로 현재 속도를 계산
    Base_Speed = getCurrentSpeed();

    //최대 회전각 제한
    if(target_angle_radian > IFX_PI/3)
    {
        target_angle_radian = IFX_PI/3;
    }

    //회전 반경 계산
    turn_radius = CAR_LENGTH / tanf(target_angle_radian);

    //두 모터의 휠 스피드 비율 계산
    float32 wheelSpeedRatio = WHEEL_BASE / (2 * turn_radius);
    if (wheelSpeedRatio > 1.0f)
    {
        //최대 1로 제한하여 속도가 음수가 되는 문제 방지
        wheelSpeedRatio = 1.0f;
    }

    //두 모터에 계산된 휠 스피드 반영
    innerWheelSpeed = Base_Speed * (1 - wheelSpeedRatio);
    outerWheelSpeed = Base_Speed * (1 + wheelSpeedRatio);

    //회전 방향에 따라 다르게 모터 스피드 인가
    if(direction == Right)
    {
        carState.mode = DRIVING_TURNING_RIGHT;
        setTargetSpeed(outerWheelSpeed, innerWheelSpeed);
        carState.leftWheelSpeed = outerWheelSpeed;
        carState.rightWheelSpeed = innerWheelSpeed;
    }
    else
    {
        carState.mode = DRIVING_TURNING_LEFT;
        setTargetSpeed(innerWheelSpeed, outerWheelSpeed);
        carState.leftWheelSpeed = innerWheelSpeed;
        carState.rightWheelSpeed = outerWheelSpeed;
    }

    return turn_radius;
}

float32 getCurrentSpeed(void)
{
    float32 MotorA_Current_RPM_Multiplied_By_Ratio = - motor_speed_rpm;
    float32 MotorB_Current_RPM_Multiplied_By_Ratio = Motor_B_motor_speed_rpm;

    float32 MotorA_Current_RPM = MotorA_Current_RPM_Multiplied_By_Ratio / GEAR_RATIO;
    float32 MotorB_Current_RPM = MotorB_Current_RPM_Multiplied_By_Ratio / GEAR_RATIO;

    float32 Current_RPM_AVG = (MotorA_Current_RPM + MotorB_Current_RPM) / 2.0f;
    float32 BASE_SPEED = (Current_RPM_AVG * 2 * 3.14159 * WHEEL_RADIUS) / 60.0f;

    return BASE_SPEED;
}
