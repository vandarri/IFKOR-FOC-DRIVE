/**
 * @file pmsm_foc_functions.c
 * @date 2015-11-23
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
 * @file pmsm_foc_functions.c
 * @date 23 Nov, 2015
 * @version 1.0.1
 *
 * @brief Sensorless FOC with 3-shunt <br>
 *
 * <b>Detailed description of file</b> <br>
 *  IDE: Infineon DAVE 4, Version: 4.1.2, Installer build : 2015-06-15
 *  HW:  Infineon PMSM-LV-15W, or XMC 750 Watt Motor Control Application Kit.
 *  MCU: Infineon XMC1302.
 *
 * History
 *
 * 29 Jun 2015 Version 1.0.0 <br>:
 *      Initial version
 *
 * 23 Nov 2015 Version 1.0.1 <br>:
 *      To support single-shunt current sensing (need to comment out SHUNT_2_OR_3 at define.h).
 * @endcond
 *
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include <XMC1300.h>                      /* SFR declarations of the selected device */
#include "pmsm_foc_functions.h"

ClarkeTransformType Clarke_Transform;
ParkTransformType Park_Transform;

Car2PolType Car2Polar;
/* ADC results, trigger positions. */
ADCType ADC;
/* Motor current and current space vector. */
CurrentType Current;
/* Motor control information */
extern MotorControlType Motor;

/* For trip / over-current detection, and protection. */
TripType Trip;
/* For motor startup lock / fail / stall detection, and protection. */
StallType Stall;
/* For over/under-voltage detection, and protection. */
OverUnderVoltType OverUnderVolt;
/* For Hall signal processing. */
/* |I|, magnitude of current space vector */
uint32_t I_Mag;
/* Angle γ (1Q23 << 8) of current space vector, from last PWM cycle */
int32_t I_AngleQ31;
/* |Vref|cos(γ-θ) = |Vref|cos(γ-θ) */
int32_t Vref_CosDelta;
int32_t Vref_SinDelta;

HallType Hall;
/* Parameters input for FOC LIB. */
extern FOCInputType FOCInput;
/* Sine LUT used for SVM calculations, array size 256 or 1024.*/
extern const uint16_t Sinus60_tab[];
/* Speed PI controller.*/
extern PI_Coefs_Type PI_Speed;
/*  Torque / Iq PI controller.*/
extern PI_Coefs_Type PI_Torque;
/*  Flux /Id PI controller.*/
extern PI_Coefs_Type PI_Flux;
/* PLL rotor speed PI controller.*/
extern PI_Coefs_Type PI_PLL;
extern PLL_EstimatorType PLL_Estimator;
extern int32_t VrefxSinDelta;
/* |I|, magnitude of current space vector */
extern uint32_t Current_I_Mag;
extern int32_t Delta_IV;

#define SHIFT_TO_1Q15		(3U)
#define MET_VREF_STEP		(1U)
/**
  * @brief	3-Shunt 3-Phase Current Reconstruction, ADC values are from last PWM cycle
  * 		ADCs of 2or3-Shunt are triggered by CCU83 CR1S
  *
  * @param	VADC_G1_RES_0
  * 		VADC_G0_RES_0
  * 		VADC_G1_RES_1
  *
  *@retval 	Current.I_U
  * 		Current.I_V
  * 		Current.I_W
  */

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
RAM_ATTRIBUTE void Current_Reconstruction (uint16_t Previous_SVM_SectorNo,int32_t ADC_result1, int32_t ADC_result2, CurrentType * const HandlePtr)
{
   switch (Previous_SVM_SectorNo)
  {
    case 0:  /* Sector A */
      HandlePtr->I_U = ADC_result1;
      HandlePtr->I_V = ADC_result2 - ADC_result1;
      break;

    case 1:  /* Sector B */
      HandlePtr->I_U = ADC_result2 - ADC_result1;
      HandlePtr->I_V = ADC_result1;
      break;

    case 2:  /* Sector C */
      HandlePtr->I_U = -ADC_result2;
      HandlePtr->I_V = ADC_result1;
      break;

    case 3:  /* Sector D */
      HandlePtr->I_U = -ADC_result2;
      HandlePtr->I_V = ADC_result2 - ADC_result1;
      break;

    case 4:  /* Sector E */
      HandlePtr->I_U = ADC_result2 - ADC_result1;
      HandlePtr->I_V = -ADC_result2;
      break;

    default:  /* Sector F */
      HandlePtr->I_U = ADC_result1;
      HandlePtr->I_V = -ADC_result2;
      break;
  }
  CCU4_Debug3Output (HandlePtr->I_U, 1,13,HandlePtr->I_V ,1,13);
}
#else
RAM_ATTRIBUTE void Current_Reconstruction (int32_t ADC_Iu, int32_t ADC_Iv, int32_t ADC_Iw, CurrentType * const HandlePtr)
{

	/* Motor phase current, Iu, Iv, Iw*/
	HandlePtr->I_U = ((int16_t)(ADC.ADC_Bias_Iu - ADC_Iu)) << 3;
	HandlePtr->I_V = ((int16_t)(ADC.ADC_Bias_Iv - ADC_Iv)) << 3;
	HandlePtr->I_W = ((int16_t)(ADC.ADC_Bias_Iw - ADC_Iw)) << 3;

	//CCU4_Debug3Output (HandlePtr->I_V, 1,11,HandlePtr->I_U ,1,11);

}
#endif

#if((MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_ONLY)||(MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC)||(MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC))

	/*###* FOC controller LIB, calculated once in each PWM cycle ####
		 * ------------------------------------------------------------*/
RAM_ATTRIBUTE void FOC_Speed_Controller (void)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  Current_Reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

#else
	Get_ADCPhaseCurrent(FOCOutput.Previous_SVM_SectorNo, FOCOutput.New_SVM_SectorNo, &ADC);

	Current_Reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
#endif
	Motor.Speed = PLL_Estimator.RotorSpeed_In;

  #if(OVERCURRENT_PROTECTION == ENABLED)
    Over_current_Protection_check(ADC.ADC_IDCLink, Park_Transform.Iq, &FOCInput.overcurrent_factor);
    FOCInput.Ref_Speed = (Motor.Ref_Speed * FOCInput.overcurrent_factor) >> 12;        // Motor reference speed.
  #else
    FOCInput.Ref_Speed = Motor.Ref_Speed;        // Motor reference speed.
  #endif

	ClarkeTransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

	ParkTransform(Clarke_Transform.I_Alpha_1Q31, Clarke_Transform.I_Beta_1Q31, PLL_Estimator.RotorAngleQ31);

	FOCOutput.Previous_SVM_SectorNo = FOCOutput.New_SVM_SectorNo;	// Record previous SVM sector number.

	/* PI Controller #1 - Speed / Speed PI controller of FOC */
	PI_controller_anti_windup(FOCInput.Ref_Speed,PLL_Estimator.RotorSpeed_In, &PI_Speed);


	ParkTransform_GetResult(&Park_Transform);

	PLL_Imag(Car2Polar.Vref_AngleQ31, Clarke_Transform.I_Alpha_1Q31,Clarke_Transform.I_Beta_1Q31);

 /* PI Controller #2 - Torque / Iq PI controller of FOC */
	PI_controller(PI_Speed.Uk, Park_Transform.Iq, &PI_Torque);

	Car2Polar.Torque_Vq = PI_Torque.Uk;

	PLL_Imag_GetResult(&PLL_Estimator);

	PLL_Vref(PLL_Estimator.Delta_IV, Car2Polar.Vref32, PI_PLL.Uk, FOCInput.Phase_L,&PLL_Estimator);

	/* PI Controller #3 - Flux / Id PI controller of FOC */
	PI_controller(FOCInput.Ref_Id, Park_Transform.Id, &PI_Flux);

	Car2Polar.Flux_Vd = PI_Flux.Uk;

	PLL_Vref_GetResult(&PLL_Estimator);

	Cart2Polar(Car2Polar.Torque_Vq, Car2Polar.Flux_Vd,PLL_Estimator.RotorAngleQ31);

	PLL_GetPosSpd(&PLL_Estimator);

	Car2Pol_GetResult(&Car2Polar);

	uint32_t SVM_Vref16;
	SVM_Vref16 = Car2Polar.Vref32 >>CORDIC_SHIFT;

	SVM_Vref16 = (SVM_Vref16 * 311) >> 8;
	Car2Polar.Vref32 = SVM_Vref16 << CORDIC_SHIFT;


}
#endif



/* To init rotor angle 1Q31 for first FOC PWM cycle, Lag/lead current angle Î³ by a 90Â° angle */
void Init_FOC_RotorAngle(void)
{
  PLL_Estimator.RotorAngleQ31 = (I_AngleQ31 - DEGREE_90) + (FOCInput.Ref_Speed << (16U - FOCInput.Res_Inc));

} /* End of Init_FOC_RotorAngle () */

/* To init PI controllers' integral terms (Ik) for first FOC PWM cycle */
void Init_FOC_PI_Iks(void)
{
  PI_Speed.Ik = I_Mag; /* Init PI integral terms for smooth transition to FOC. */
  PI_Torque.Ik = (Vref_CosDelta * 256) >> 8; /*
                                              * Init Vq of torque / Iq PI controller,
                                              * |Vref|cos(Î³-Î¸) = |Vref|cos(Î¸-Î³).
                                              */
  PI_Flux.Ik = (Vref_SinDelta * 256) >> 8; /* Init Vd of flux / Id PI controller, |Vref|sin(Î³-Î¸) < 0 typically. */

  PI_PLL.Ik = FOCInput.Ref_Speed; /* Init rotor speed Ï‰r of PLL Observer PI controller. */

  PI_Speed.Ik <<= PI_Speed.Scale_KpKi; /* All PI integral terms left shift by PI_data->Scale_KpKi. */
  PI_Torque.Ik <<= PI_Torque.Scale_KpKi;
  PI_Flux.Ik <<= PI_Flux.Scale_KpKi;
  PI_PLL.Ik <<= PI_PLL.Scale_KpKi;

	}	// End of Init_FOC_PI_Iks ()



/*
 * To update angle θ (16-bit) of SVM reference vector Vref
 * Digital implementation θ[k] = θ[k-1] + ω[k]
 */
void Update_Vref_Angle (int32_t Speed)
{
	Car2Polar.Vref_AngleQ31_Previous =  Car2Polar.Vref_AngleQ31;		// Record Vref angle θ of last PWM cycle.

  if (Motor.Rotation_Dir == DIRECTION_INC)
  {
    /* If motor rotation direction - angle increasing. θ[k] = θ[k-1] + ω[k]. */
    Car2Polar.Vref_AngleQ31 += (Speed << (16U - RES_INC)); /* θ[k] = θ[k-1] + ω[k]. */
  }
  else
  {
		Car2Polar.Vref_AngleQ31 -= (Speed << (16U-RES_INC));		// θ[k] = θ[k-1] - ω[k].
	}

	Car2Polar.SVM_Angle16 = Car2Polar.Vref_AngleQ31 >> 16U;			// SVM Vref angle θ (16-bit).
}	// End of Update_Vref_Angle ()

/* Direct FOC startup. Motor startup to FOC closed-loop directly, no V/f or MET.*/
#if(MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC)
/* Initial ramp up rate in FOC transition mode.*/
  #define RAMP_RATE_INIT_FOC    (RAMPUP_RATE << 0U)
#else
  #define RAMP_RATE_INIT_FOC    (RAMPUP_RATE << 0U)
#endif
/* (SPEED_LOW_LIMIT), (DEFAULT_SPEED_REFERENCE). Minimum startup speed for FOC. */
#define MIN_STARTUP_SPEED_FOC   (SPEED_LOW_LIMIT << 1)
/* Threshold speed to exit FOC. */
#define FOC_EXIT_SPEED  (SPEED_LOW_LIMIT * 3)



	/** Miscellaneous works in FOC, such as ramp up, speed adjustment, stop motor, etc
	 ** Do NOT add any CORDIC calculations in this function.
		 * -----------------------------------------------------------------------------------*/
RAM_ATTRIBUTE void Misc_Works_of_FOC (void)
{
	  uint16_t DCLink_adc_result;
      uint16_t IDCLink_adc_result;
      XMC_VADC_GLOBAL_BackgroundTriggerConversion(VADC);
      /* DC link ADC LPF. Read RES5 for ADC result (Previous ADC result) */
	  DCLink_adc_result = VADC_VDC_GROUP->RES[VADC_VDC_RESULT_REG];
		ADC.ADC_DCLink = (ADC.ADC_DCLink * ((1<<ADCLPF)-1) + DCLink_adc_result) >> ADCLPF;

		/* IDC link ADC LPF.  */
	   IDCLink_adc_result = VADC_IDC_GROUP->RES[VADC_IDC_RESULT_REG];
	   ADC.ADC_IDCLink = (ADC.ADC_IDCLink * ((1 << ADCLPF) - 1) + IDCLink_adc_result) >> ADCLPF;

    #if(MY_FOC_CONTROL_SCHEME == CONSTANT_TORQUE_DIRECT_FOC)
		Linear_Torque_Ramp_Generator(Motor.Speed_by_POT_PWM,USER_IQ_RAMPUP, USER_IQ_RAMPDOWN, &FOCInput);
		if((SYSTEM_BE_IDLE) && (FOCInput.Ref_Iq <= Motor.Speed_by_POT_PWM) && Motor.Speed < FOC_EXIT_SPEED)
		{
		  /* Clear counters.*/
      Motor.Counter = 0;
      /* Next, go to Motor Stop (may brake motor and cause vibration).*/
      Motor.State = STOP_MOTOR;
		}
    #endif

    #if(MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC)

		Linear_VQ_Ramp_Generator(Motor.Speed_by_POT_PWM ,USER_VQ_RAMPUP, USER_VQ_RAMPDOWN, &FOCInput);

		if((SYSTEM_BE_IDLE) && (FOCInput.Vq <= Motor.Speed_by_POT_PWM) && Motor.Speed < FOC_EXIT_SPEED)
		{
		  Motor.Counter = 0;
		  Motor.State = STOP_MOTOR;
		}
    #endif

		if (Motor.Mode_Flag == MOTOR_TRANSITION)
		{
		  /* Motor in transition mode. Motor goes to higher speed first, important for startup. */
        #if((MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC))
		    Linear_Ramp_Generator(MIN_STARTUP_SPEED_FOC,RAMP_RATE_INIT_FOC,RAMPDOWN_RATE,SPEEDRAMPSTEP,&Motor.Ref_Speed);
        #endif
	  }
		else   /* else, FOC is in stable mode.*/
		{
		    /* Over/under voltage protection */
        #if(VDC_UNDER_OVERVOLTAGE_PROTECTION == ENABLED)

          if(VADC_VDC_GROUP->CEFLAG & (1 << VADC_VDC_CHANNEL))
          {
            if (ADC.ADC_DCLink > VDC_OVER_LIMIT)
            {
              /* Motor.error_status = PMSM_FOC_EID_OVER_VOLT;*/
              Motor.State = DCLINK_OVER_UNDER_VOLTAGE;
              /* Disable gate driver. */
              XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, DISABLE_LEVEL);
            }
            else if(ADC.ADC_DCLink < VDC_MIN_LIMIT)
            {
              /* Motor.error_status =  PMSM_FOC_EID_UNDER_VOLT; */
              Motor.State = DCLINK_OVER_UNDER_VOLTAGE;
              /* Disable gate driver. */
              XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, DISABLE_LEVEL);
            }
          VADC_VDC_GROUP->CEFCLR |= (1 << VADC_VDC_CHANNEL);

          }
        #endif

        #if((MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC))
          SCurve_Ramp_Generator(Motor.Speed_by_POT_PWM, RAMPUP_RATE, RAMPDOWN_RATE, SPEEDRAMPSTEP,&Motor.Ref_Speed);
          Adjust_FOC_Parameters ();							// Adjust parameters, e.g.: for PI controllers.
          if ((SYSTEM_BE_IDLE) && (Motor.Ref_Speed <= Motor.Speed_by_POT_PWM) && (Motor.Speed < FOC_EXIT_SPEED))
          {	// If PWM duty cycle or POT ADC too low, and motor ref speed reached PWM set speed,
          //if ((SYSTEM_BE_IDLE) && (Motor.Ref_Speed <= Motor.Speed_by_POT_PWM) && (Motor.Speed < FOC_EXIT_SPEED)) {
          //if (SYSTEM_BE_IDLE) {									// If PWM duty cycle or POT ADC too low,
              Motor.Counter = 0;								/* Clear counters. */
              Motor.Ramp_Counter = 0;
              Motor.State = STOP_MOTOR;						/* Next, go to Motor Stop (may brake motor and cause vibration). */
          }
        #endif
		}
}


#define PI_SPEED_IK_LIMIT_FINAL		((((1<<15) * 6U) >> 3U) << PI_SPEED_SCALE_KPKI)
/* Step for parameter Ik change. */
#define SPEED_IK_ADJUST_STEP		(1<<14)

/** Adjust parameters, e.g.: for PI controllers, in FOC stable state
** Scheduling - using different parameters in different operating regions.
** Execution time: ?us (O3 - Optimize most).
 * ----------------------------------------------------------------------*/
RAM_ATTRIBUTE void Adjust_FOC_Parameters (void)
{
     /* Parameter adjustment not finished yet. */
    if (Motor.Adjust_Para_Flag != ADJUST_DONE)
    {
      /* 1). Ik limit scheduling for Speed PI controller:*/
      if (PI_Speed.Ik_limit_max < PI_SPEED_IK_LIMIT_FINAL)
      {
        /* Parameter adjusted gradually and regularly every PWM cycle. */
        PI_Speed.Ik_limit_max += SPEED_IK_ADJUST_STEP;
        PI_Speed.Ik_limit_min = - PI_Speed.Ik_limit_max;
      }
      else
      {
        /* To indicate that adjustment of this parameter is done. */
        Motor.Adjust_Para_Flag = ADJUST_DONE;
      }
    }
}


#if(MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC)
/* ~0.5s, max time that MET control takes before considered stable, x PWM period. */
#define TIME_OF_MET 		(7500U)

/** Miscellaneous works in MET, such as ramp up, speed adjustment, transition from MET to FOC, etc
* ---------------------------------------------------------------------------------------------------*/
void Misc_Works_of_MET (void)
{
    if (Motor.Mode_Flag != MOTOR_TRANSITION)
    {					// Motor in transition mode.
       Transition_to_FOC();                // Transition from MET to FOC, using 3-step motor start-up V/f->MET->FOC.
    }

    if ((SYSTEM_BE_IDLE) && (Motor.Speed <= Motor.Speed_by_POT_PWM))
    {
      /* If PWM duty cycle or POT ADC too low, and speed reached PWM set speed */
      Motor.Counter = 0;
      Motor.Ramp_Counter = 0;
      /* Set low motor speed, so motor stop and resume can be fast.*/
      Motor.Speed = SPEED_LOW_LIMIT >> 5;
       /* Next, go to Motor Stop. */
      Motor.State = STOP_MOTOR;
  }

} /* End of Misc_Works_of_MET () */

/*
 * Init variables for transition to FOC, e.g. from MET to FOC: MET -> FOC
 * Execution time: ?us (O3 - Optimize most)
 * 3-step motor start-up: V/f open-loop -> MET closed-loop -> FOC closed-loop.
 */
void Transition_to_FOC(void)
{
  Motor.Counter = TIME_OF_MET + 1; /* Go to FOC immediately once it find |ε| <= ε_Th. */

  if (Motor.Counter > TIME_OF_MET)
  {
    Motor.State = FOC_CLOSED_LOOP; /* Next, go to FOC closed-loop. */

    Init_FOC_RotorAngle(); /* Init rotor angle for first FOC PWM cycle, Lag/lead current angle γ by a 90° angle. */
    Init_FOC_PI_Iks(); /* To init PI controllers' Ik for first FOC PWM cycle. */

    PI_PLL.Uk = Motor.Speed; /*
                              * Init FOC rotor speed ωr = PI_PLL.Uk, needed for ωL|I|, ωLId, ωLIq,
                              * and FG frequency calculation.
                              */
    Motor.Ref_Speed = Motor.Speed; /* Motor reference speed of FOC. */

    FOC_SystemParameters_Init_OnceOnly(); /* Init parameters of FOC LIB. Init once only. */

    Motor.Mode_Flag = MOTOR_TRANSITION; /* Motor in transition mode. */
    Motor.Counter = 0; /* Clear counters. */
    Motor.Ramp_Counter = 0;
    Motor.Ramp_Up_Rate = RAMPUP_RATE << RATIO_S; /* Slower ramp up and ramp down for S-curve profile. */
  }
} /* End of Transition_to_FOC () */
#endif

/* Init parameters of LIB. Init once only */

void FOC_SystemParameters_Init_OnceOnly (void)
{

  /* Init below once only before go to FOC: */
  FOCInput.Phase_L = L_OMEGALI;
  FOCInput.Phase_L_Scale = SCALE_L;

  FOCInput.Res_Inc = RES_INC; /* Resolution increase, use (16 + Res_Inc) bit to represent 360 deg. */
  FOCInput.LPF_N_BEMF = SHIFT_MET_PLL;

  FOCInput.CCU8_Period = (uint32_t) PERIOD_REG;

  FOCInput.Ref_Id = 0;

  FOCInput.Vq = 0;
  FOCInput.Vq_Flag = 0; /* FOC Vq from Iq PI controller. */

  FOCInput.Iq_PI_Flag = 0; /* Reference of Iq PI controller from speed PI output. */

  FOCInput.RotorSpeed_In = 0;

  FOCInput.SVM_5_Segment_Flag = 0; /* 7-segment SVM. For 3-shunt current sensing only. */

  FOCInput.Flag_State = 0;

    #if(OVERCURRENT_PROTECTION == ENABLED)
     FOCInput.overcurrent_factor = 4096;                /* */
    #endif
  PLL_Estimator.RotorSpeed_In = Motor.Speed;
  FOCInput.Ref_Speed = Motor.Speed; /* Motor reference speed. */

  PI_PLL.Ik = PLL_Estimator.RotorSpeed_In << PI_PLL.Scale_KpKi;

  FOCOutput.New_SVM_SectorNo = SVM.CurrentSectorNo;

  FOCOutput.Previous_SVM_SectorNo = 0;
  FOCOutput.New_SVM_SectorNo = 0; /* Use default SVM sector. */

} /* End of FOC_SystemParameters_Init_OnceOnly () */
















