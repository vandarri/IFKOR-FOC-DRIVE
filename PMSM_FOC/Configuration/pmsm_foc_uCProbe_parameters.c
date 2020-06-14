/**
 * @file pmsm_foc_uCProbe_parameters.c
 * @date 2015-12-16
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes with
 * Infineon Technologies AG dave@infineon.com).
 *********************************************************************************************************************
 *
 * @file pmsm_foc_uCProbe_parameters.c
 * @date 17 Dec, 2015
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
 * 17 Dec 2015 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include <XMC1300.h>
#include "..\PMSM_FOC\ControlModules\pmsm_foc_functions.h"
#include "..\PMSM_FOC\MCUInit\adc.h"
#include "..\PMSM_FOC\MIDSys\pmsm_foc_current_threeshunt.h"
#include "pmsm_foc_uCProbe_parameters.h"
#include "pmsm_foc_user_mcuhwconfig.h"
#include "pmsm_foc_user_parameter.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/

/* Global variables: */
extern ADCType ADC; /* ADC results, trigger positions. */
MotorControlType Motor; /* Motor control information */
extern CurrentType Current; /* Motor current and current space vector. */
extern Car2PolType Car2Polar;

extern TripType Trip; /* For trip / over-current detection, and protection. */
extern StallType Stall; /* For motor startup lock / fail / stall detection, and protection. */
extern HallType Hall;
extern SVMType SVM; /* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
/* extern PLL_EstimatorType PLL_Estimator; */
/* extern PI_Coefs_Type PI_PLL;          // PLL rotor speed PI controller. */

FOCInputType FOCInput; /* Parameters input for FOC LIB. */
FOCOutputType FOCOutput; /* Output for FOC LIB. */


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/*###* Init for motor control ####
	 * ---------------------------*/
void MotorControl_Init (void)
{


  Motor.State = BRAKE_BOOTSTRAP;/*
                                 * First brake the motor before motor startup.
                                 * Charge gate driver bootstrap capacitors (if any).
                                 */

  Motor.Rotation_Dir = DIRECTION_INC; /* Motor rotation direction - rotor angle increasing. */

	Variables_Init ();								/* Init variables. */
	Get_Current_Bias();

} /* End of MotorControl_Init () */


/* Init variables */

void Variables_Init (void)
{
  Motor.Mode_Flag = MOTOR_TRANSITION;				// Motor in transition mode.

  Motor.L_METPLL = L_OMEGALI;						// Motor inductance per phase
                                     //Using L_OMEGALI instead of Motor.L_METPLL in multiplication saves one MCU clock.

  Motor.Counter = 0;								// Init counters.
  Motor.Ramp_Counter = 0;
  Motor.Alignment_Counter = 0;
  Motor.Non_RealTime_Counter = 1;
  Motor.UART_Counter = 0;
  Motor.UART_Debug_Counter = 0;

  Motor.Speed = DEFAULT_SPEED_STARTUP;			// Init for V/f ramp-up.
  Motor.FG_Speed = Motor.Speed;					// Motor speed for Frequency Generation (FG) only.
  Motor.Ref_Speed = 0;
  Motor.Speed_by_POT_PWM = SPEED_LOW_LIMIT;

  Motor.Ramp_Up_Rate = RAMPUP_RATE << RATIO_S;	// Slower ramp up and ramp down for S-curve profile.
  Motor.Ramp_Dn_Rate = RAMPDOWN_RATE << (RATIO_S - 1);


  Motor.PWM_DutyCycle = 0;
  Motor.PWM_Speed_Raw = 0;
  Motor.PWM_Freq = 20;							// Init PWM frequency 20Hz.

  Car2Polar.SVM_Angle16 = (DEGREE_X >> 16U);		// Init Vref angle θ = X°.

  Car2Polar.Vref_AngleQ31 = Car2Polar.SVM_Angle16 << 16U;
  Car2Polar.Vref_AngleQ31_Previous = Car2Polar.Vref_AngleQ31;


  Car2Polar.SVM_Vref16 = 0;

  ADC.ADCTrig_Point = (uint32_t)(PERIOD_REG) >> 1;			// For ADC trigger for 2or3-shunt current sensing.

  ADC.ADC_DCLink = ADC_DCLINK_IDEAL;

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  ADC.ADC3Trig_Point = 0;           /* For ADC 3 trigger, of single-shunt current sensing.*/
  ADC.ADC4Trig_Point = 0;           /* For ADC 4 trigger. */

  ADC.ADC_Result1 = 0;
  ADC.ADC_Result2 = 0;
  ADC.ADC_ResultTz1 = 0;
  ADC.ADC_ResultTz2 = 0;
  ADC.ADC_Result3 = 0;
  ADC.ADC_Result4 = 0;
  SVM.SVM_Flag = SVM_USE_PZV;        /* Init using SVM with Pseudo Zero Vectors (PZV). */
  ADC.Result_Flag = RESULTS_ADCTZ12;
#endif

  /* Init motor phase currents */
  Current.I_U = 0;
  Current.I_V = 0;
  Current.I_W = 0;

  SVM.PreviousSectorNo = 0;						// Init SVM sector No.

  SVM.Flag_3or2_ADC = USE_ALL_ADC;				// Init to use all (e.g.: three) ADC samplings for current reconstruction, for 2or3-shunt.

  PI_controller_Init();							// Init parameters (Kp / Ki, limits) of PI controllers.


  FOC_SystemParameters_Init_OnceOnly();

}	// End of Variables_Init ()



