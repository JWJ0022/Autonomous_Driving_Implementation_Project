/**********************************************************************************************************************
 * \file Ultrasonic.c
 * \copyright Copyright (C) Infineon Technologies AG 2019
 *
 * Use of this file is subject to the terms of use agreed between (i) you or the company in which ordinary course of
 * business you are acting and (ii) Infineon Technologies AG or its licensees. If and as long as no such terms of use
 * are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization obtaining a copy of the software and
 * accompanying documentation covered by this license (the "Software") to use, reproduce, display, distribute, execute,
 * and transmit the Software, and to prepare derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including the above license grant, this restriction
 * and the following disclaimer, must be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are solely in the form of
 * machine-executable object code generated by a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

#include <BSW_Filter/ULTRA_FILT.h>
#include <BSW_GTM/GTM_TIM_Capture.h>
#include <BSW_Sensor/Ultrasonic.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
UKFilter ukf;

static float32 rearcheck = 0.0;
static float32 sidecheck = 0.0;

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void Init_Ultrasonics(void)
{
    //init_TIM();
}

double ReadUltrasonic_noFilt(void)
{
    float32 b_duration;
    float32 b_distance;

    /* Calculate Distance */
    b_duration = measure_PWM();
    b_distance = 0.343 * b_duration / 2.0;
    if(!isnan(b_distance) && b_distance > 0.0) {
        if(b_distance < 50.0){b_distance = 50.0;}

        median_filter(b_distance, b_duration);
    }

    rearcheck = b_distance;

    return b_distance;
}

double SideUltrasonic_noFilt(void) {
    float32 s_duration;
    float32 s_distance;

    s_duration = side_PWM();
    s_distance = 0.343 * s_duration / 2.0;

    sidecheck = s_distance;

    return s_distance;
}

