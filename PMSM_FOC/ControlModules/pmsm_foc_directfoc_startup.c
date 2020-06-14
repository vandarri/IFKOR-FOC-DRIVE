/**
 * @file pmsm_foc_directfoc_startup.c
 * @date 2015-12-23
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015-2016, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ***********************************************************************************************************************
 *
 * @file pmsm_foc_directfoc_startup.c
 * @date 23 Dec, 2015
 * @version 1.0.0
 *
 * @brief ACMP low level driver API prototype definition for XMC1 <br>
 *
 * <b>Detailed description of file</b> <br>
 * APIs provided in this file mainly cover the following functionality:
 * ---- Filter, Hysterisis, Output inversion
 *
 * History
 *
 * 23 Dec 2015 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "pmsm_foc_directfoc_startup.h"

/*********************************************************************************************************************
 * MACROS
 *********************************************************************************************************************/

#define IV_ADC_BIAS   (2045)
#define IW_ADC_BIAS   (2048)
#define VREF_INC_STEP   (10U)                        /* Step that voltage increases. */
#define PRE_POSIT_VOLT ((VQ_VF_OFFSET * 2U) >> 1)   /* Voltage for rotor preposition/alignment. */
#define PI_SPEED_IK_DEFAULT (1U << 10)
/*********************************************************************************************************************
 * GLOBAL DATA
**********************************************************************************************************************/
extern ADCType ADC;                             /* ADC results, trigger positions. */
extern SVMType SVM;                             /* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
extern CurrentType Current;                     /* Motor current and current space vector. */
extern MotorControlType Motor;                  /* Motor control information */
extern FOCOutputType FOCOutput;                 /* Output for FOC LIB. */
extern FOCInputType FOCInput;                   /* Parameters input for FOC LIB. */
extern ClarkeTransformType Clarke_Transform;
extern Car2PolType Car2Polar;

extern PI_Coefs_Type PI_Speed;                  /* Speed PI controller. */
extern PI_Coefs_Type PI_Torque;                 /* Torque / Iq PI controller. */
extern PI_Coefs_Type PI_Flux;                   /* Flux /Id PI controller. */
extern PI_Coefs_Type PI_PLL;                    /* PLL rotor speed PI controller. */


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/* API to brake the motor, charge gate driver bootstrap capacitors */
void DirectFOC_StartUp_Brake_Motor_Bootstrap_Charge(void)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
   ADC.ADC_Bias = (uint32_t) ((ADC.ADC_Bias * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + ADC.ADC_ResultTz1)
                                 >> SHIFT_BIAS_LPF;
#else
  Get_ADCPhaseCurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);


  /* Read Iu ADC bias */
  ADC.ADC_Bias_Iu = (uint32_t) ((ADC.ADC_Bias_Iu * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + ADC.ADC_Iu)
                                 >> SHIFT_BIAS_LPF;
  /* Read Iv ADC bias */
  ADC.ADC_Bias_Iv = (uint32_t) ((ADC.ADC_Bias_Iv * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + ADC.ADC_Iv)
                                 >> SHIFT_BIAS_LPF;
  /* Read Iw ADC bias */
  ADC.ADC_Bias_Iw = (uint32_t) ((ADC.ADC_Bias_Iw * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + ADC.ADC_Iw)
                                 >> SHIFT_BIAS_LPF;
#endif
  Motor.Counter++;


  if (ADC.ADC_POT < TH_POT_ADC)
  {
    /* If system is idle, i.e.: PWM duty cycle or POT ADC too low. */
    Motor.Counter = 0U; /* cannot go to ramp up, keep motor braking. */
  }

  /*Timer decides when to stop motor braking. */
  if (Motor.Counter > BRAKE_TIME)
  {
    /* Next, go to rotor initial preposition/alignment. */
    Motor.State = (uint32_t) PRE_POSITIONING;

    /* Motor in transition mode. */
    Motor.Mode_Flag = MOTOR_TRANSITION;
    /* Clear counters. */
    Motor.Counter = 0U;
    Motor.Ramp_Counter = 0U;

    Current.I_U = 0;
    Current.I_V = 0;
    Current.I_W = 0;

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    ADC.ADC3Trig_Point = 0;
    ADC.ADC4Trig_Point = 0;
    ADC.ADC_Result1 = 0;
    ADC.ADC_Result2 = 0;
    ADC.Result_Flag = RESULTS_ADCTZ12;
    /* Set ADC trigger for ADCTz1/Tz2, for single-shunt current sensing only. */
    ADCTZ12_TriggerSetting ();
#endif
  }

}
/* End of DirectFOC_StartUp_Brake_Motor_Bootstrap_Charge () */

/* API to set the Rotor initial preposition/alignment */
#define MIN_TORQUE_TOSTART  (USER_IQ_REF_HIGH_LIMIT * 0.3)
void DirectFOCRotor_Pre_Positioning(void)
{
  uint32_t Ik_unsigned;

  /* Single-shunt or 2or3-shunt 3-phase current reconstruction, to get Iu and Iv. */
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  Current_Reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);
#else
  Current_Reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
#endif
  ClarkeTransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

  /* Rotor preposition/alignment counter ++. */
  Motor.Alignment_Counter++;
  if (Motor.Alignment_Counter < ALIGNMENT_TIME)
  {

    if (Car2Polar.SVM_Vref16 < PRE_POSIT_VOLT)
    {
      /* Vref increases gradually. */
      Car2Polar.SVM_Vref16 += VREF_INC_STEP;
    }
  }
  else
  {
    /* Next, directly to FOC closed-loop for faster startup. */
    Motor.State = (uint32_t) FOC_CLOSED_LOOP;
    /* Motor in transition mode. */
    Motor.Mode_Flag = MOTOR_TRANSITION;
    /* Clear counter. */
    Motor.Alignment_Counter = 0U;
    /* Init FOC rotor speed ωr = PI_PLL.Uk, needed for ωL|I|, ωLId, ωLIq,
     *  * and FG frequency calculation. */
    PI_PLL.Uk = (int32_t) Motor.Speed ;
    /* Motor reference speed of FOC. */
    Motor.Ref_Speed = (int32_t) Motor.Speed;
    Ik_unsigned = (uint32_t) Car2Polar.SVM_Vref16 << PI_TORQUE_SCALE_KPKI;
    PI_Torque.Ik = (int32_t) Ik_unsigned << 0;
    /* Init Vd's Ik = 0 of flux / Id PI controller. */
    PI_Flux.Ik = 0;
#if(MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC)
    /* no V/f or MET. */
    Motor.Ramp_Up_Rate = (RAMPUP_RATE << 2U);
    /* Slower ramp up for S-curve profile of FOC. */
    PI_Speed.Ik = PI_SPEED_IK_DEFAULT << PI_SPEED_SCALE_KPKI;

#endif
  }

  /* Update SVM PWM. */
  PWMSVM01_Update(Car2Polar.SVM_Vref16, Car2Polar.SVM_Angle16);
  /* Record SVM reference vector magnitude (32-bit) of last PWM cycle. */
  Car2Polar.Vref32_Previous = Car2Polar.Vref32;
  Car2Polar.Vref32 = (uint32_t) Car2Polar.SVM_Vref16 << CORDIC_SHIFT;

  /* Init for smooth transition from V/f to FOC closed-loop. */
  Init_Smooth_Transition_To_FOC(Motor.Speed);

  if (Motor.State == (uint32_t) FOC_CLOSED_LOOP)
  {
    /* Init parameters of FOC LIB. Init once only. */
    FOC_SystemParameters_Init_OnceOnly();
  }

}
/* End of DirectFOCRotor_Pre_Positioning () */

