#include <BSW_CAN/CanDefinitions.h>

// ✅ 기존 전역 변수를 구조체 포인터로 변경
static CanSignalData_Rx g_canSignalData_Rx = {0};
static CanSignalData_Tx g_canSignalData_Tx = {0};
static CanSignalFlags g_canSignalFlags = {0};

// ✅ 구조체 포인터를 통한 데이터 접근
CanDataManager canDataManager = {
    .rxData = &g_canSignalData_Rx,
    .txData = &g_canSignalData_Tx,
    .flags = &g_canSignalFlags
};


const CanMessage canMessages_Tx[] = {
        {Motor_BrakeSta, 8, "Motor_BrakeSta"},
        {Motor_EmergencyAlarm, 8, "Motor_EmergencyAlarm"},
        {Motor_WheelAngle, 8, "Motor_WheelAngle"},
        {Motor_LaneChange, 8, "Motor_LaneChange"},
        {Motor_VehicleSpeed, 8, "Motor_VehicleSpeed"},
};

const CanMessage canMessages_Rx[] = {
    {DMS_DrvCautionLv, 8, "DMS_DrvCautinoLv"},
    {LKAS_HeadAngle, 8, "LKAS_HeadAngle"},
    {LKAS_Right, 8, "LAKS_Right"},
    {LKAS_Left, 8, "LAKS_Left"},
};

// ✅ CAN 신호 정의
const CanSignal canSignals_Tx[] = {
        {"BrakeSta", 0, 2, false, false},
        {"Motor_EmergencyAlarm", 0, 1, false, false},
        {"WheelAngle", 0, 8, false, false},
        {"LaneChange", 0, 2, false, false},


        // to do
        {"VehicleSpeed", 0, 8, false, false},
};

const CanSignal canSignals_Rx[] = {
    {"DrvCautionLv", 0, 3, false, false},
    {"HeadAngle", 0, 16, false, true},
    {"Left", 0, 16, false, true},
    {"Right", 0, 16, false, true},
};

