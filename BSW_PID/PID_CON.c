/*=====================================================================================
 File name:        PID_REG3_2level.C                   
                    
 Originator:    Digital Control Systems Group
            Texas Instruments

 Description:  PID controller with anti-windup          

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005 Version 3.20
-------------------------------------------------------------------------------------*/

#include <BSW_PID/PID_CON.h>

#define ERR_SUM_MAX 1000
#define ERR_SUM_MIN -1000

void pid_reg3_calc(PIDREG3 *v)
{

    v->Err = v->Ref - v->Fdb;

    // Discrete PID Controller
    v->Up = v->Kp * v->Err ;

    v->ErrSum += v->Err * v->T_samp;
    if (v->ErrSum > ERR_SUM_MAX) {
        v->ErrSum = ERR_SUM_MAX;
    } else if (v->ErrSum < ERR_SUM_MIN) {
        v->ErrSum = ERR_SUM_MIN;
    }

    v->Ui = v->Ki * v->ErrSum;


    v->Ud = v->Kd / v->T_samp * (v->Err - v->Err_1) ;

    //    v->Ui = v->Ki * v->T_samp * (0.5 * v->Err + v->ErrSum) - v->Kc * v->SatErr ;

    // Compute the pre-saturated output
    v->Out = v->Up + v->Ui;


     if (v->Out > v->OutMax)
      v->Out =  v->OutMax;

    else if (v->Out < v->OutMin)
      v->Out =  v->OutMin;

    v->Err_1 = v->Err;
}

// for reset all the PI value

void pid_reset(PIDREG3 *v)
{
    // reset the error
    v->Err = 0;
    v->Err_1 = 0;
    v->Err_2 = 0;

    // reset the p, I, d output
    v->Up = 0;     v->Ui = 0;    v->Ud = 0;

    // reset the pre-saturated output
    v->OutPreSat = 0;
    
     // reset the previous proportional output
    v->Out = 0;
    v->Out_1 = 0;
//  v->ErrSum = 0;
    v->Fdb = 0;

    // reset the saturate difference
    v->SatErr = 0;     
    
    // reset the previous proportional output 
   // v->Up1 = 0;
  
}
