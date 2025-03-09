/* =================================================================================
File name:       PID_REG3_2LEVEL.H

Originator: Digital Control Systems Group
            Texas Instruments

Description:
Header file containing constants, data type definitions, and
function prototypes for the PIDREG3.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005 Version 3.20
------------------------------------------------------------------------------*/
#include "Ifx_Types.h"


//#include "PARAMETERS_PMSM_Control.h"


typedef struct {  float32  Ref;           // Input: Reference input
                  float32  Fdb;           // Input: Feedback input
                  float32  Err;           // Variable: Error (k)
                  float32  Err_1;         // Variable: previous Error(k-1)
                  float32  Err_2;         // Variable: previous Error(k-2)
                  float32  ErrSum;        // Summation of Err
                  float32  Kp;            // Parameter: Proportional gain
                  float32  Up;            // Variable: Proportional output
                  float32  Up1;           // Variable: Proportional output
                  float32  Ui;            // Variable: Integral output
                  float32  Ui_1;          // Variable: Integral output
                  float32  Ud;            // Variable: Derivative output
                  float32  OutPreSat;     // Variable: Pre-saturated output
                  float32  OutMax;        // Parameter: Maximum output
                  float32  OutMin;        // Parameter: Minimum output
                  float32  Out;           // Output: PID output
                  float32  Out_1;         // Output: previous PID output
                  float32  SatErr;        // Variable: Saturated difference
                  float32  Ki;            // Parameter: Integral gain
                  float32  Kc;            // Parameter: Integral correction gain
                  float32  Kd;            // Parameter: Derivative gain
                  float32  Ka;            // Parameter : Accelator gain
                  float32  T_samp;
                  uint32    Current_mode; // PID mode for Current Control
                  void  (*calc)();      // Pointer to calculation function
                  void  (*reset)();     // Pointer to calculation function
                 } PIDREG3;

typedef PIDREG3 *PIDREG3_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/                     
#define PIDREG3_DEFAULTS { 0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                          (void (*)(unsigned int))pid_reg3_calc, \
                          (void (*)(unsigned int))pid_reset }

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pid_reg3_calc(PIDREG3_handle);
void pid_reset(PIDREG3_handle);
