/**
 * @file pmsm_foc_interface.c
 * @date 2015-12-16
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
 * @file pmsm_foc_interface.c
 * @date 16 Dec, 2015
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
 * 16 Dec 2015 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <XMC1300.h>                      // SFR declarations of the selected device
#include "pmsm_foc_interface.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/* Global variables:*/
extern ClarkeTransformType Clarke_Transform;
extern Car2PolType Car2Polar;
extern PLL_EstimatorType PLL_Estimator;
/* Angle γ (1Q23 << 8) of current space vector, from last PWM cycle */
extern int32_t I_AngleQ31;
/* ε = |Vref|sin(γ-θ)+ωL|I| */
int32_t Epsilon;
/* ADC results, trigger positions. */
extern ADCType ADC;
/* Motor control information */
extern MotorControlType Motor;
/* Motor current and current space vector. */
extern CurrentType Current;
/* For trip / over-current detection, and protection. */
extern TripType Trip;
/* For motor startup lock / fail / stall detection, and protection. */
extern StallType Stall;
/* For Hall signal processing. */
extern HallType Hall;
/* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
extern SVMType SVM;
/* Output for FOC LIB. */
extern FOCOutputType FOCOutput;
/* Parameters input for FOC LIB. */
extern FOCInputType FOCInput;
/* Speed PI controller. */
extern PI_Coefs_Type PI_Speed;
/* Torque / Iq PI controller. */
extern PI_Coefs_Type PI_Torque;
/* Flux /Id PI controller. */
extern PI_Coefs_Type PI_Flux;
/* PLL rotor speed PI controller. */
extern PI_Coefs_Type PI_PLL;

/* Data Structure initialization */



/* Global variable. MCU Reset Status Information, reason of last reset. */
extern uint32_t * NEW_SHS0_CALOC1;
extern uint32_t g_mcu_reset_status;

/* 0.05s, time that motor keeps in Stop Mode, x PWM period. */
#define TIME_OF_STOP 	(200U)

#define TIME_OF_VFRAMP		(750U)
/* Step that voltage increases. */
#define ALIGNMENT_STEP		(32U)
/* Voltage for rotor preposition/alignment. */
#define ALIGNMENT_VOLT		((VQ_VF_OFFSET * 2) >> 1)
/* Ratio for ramp up slowly in V/f open loop.*/
#define RAMP_RATIO_VF		(1U)
/* Indicates a running CORDIC calculation if MATH->STATC[0] (i.e.: BSY) = 1. */
#define CORDIC_IS_BSY (MATH->STATC & 0x01)

#if(uCPROBE_GUI == ENABLED)
extern uint32_t Update_KpKi;                      /* uCProbe Save KPKi to flash button variable */
uint32_t Speed_in_rpm;                     /* uC_Probe variable */
uint32_t Real_Speed_in_rpm;                /* uC_Probe variable */
#endif


/* V/f Only, with rotor initial preposition/alignment
 * ----------------------------------------------------------*/
  void VF_ONLY_OpenLoop_RampUp (void)
  {
    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      Current_Reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

    #else
      Get_ADCPhaseCurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);

      Current_Reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif
    /* To get I_Alpha and I_Beta of last PWM cycle, scale down I_Mag (i.e.: |I|) by 2/3. */
    ClarkeTransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

    /* Update V/f voltage amplitude, Vref = Offset + Kω. */
    Car2Polar.SVM_Vref16 = VQ_VF_OFFSET + (VQ_VF_SLEW * (Motor.Speed >> RES_INC));

    /* To update angle θ (16-bit) of SVM reference vector Vref. */
    Update_Vref_Angle (Motor.Speed);

    if (Motor.Mode_Flag == MOTOR_TRANSITION)
    {       /* Motor is in transition mode */
      if (Motor.Speed < VF_TRANSITION_SPEED)
      {
      /* Motor speed not reach V/f open-loop to MET/FOC transition speed */
      /* Speed ramp counter ++. */
      Motor.Ramp_Counter ++;
        if (Motor.Ramp_Counter > Motor.Ramp_Up_Rate)
        {
            /* Ramp up slowly in V/f.*/
            if (Motor.Ramp_Up_Rate > (RAMPUP_RATE << RAMP_RATIO_VF))
            {
              /* Increase acceleration step by step.*/
              Motor.Ramp_Up_Rate --;
            }
            /* Motor speed ++. */
            Motor.Speed += SPEEDRAMPSTEP;
            /* Clear ramp counter.*/
            Motor.Ramp_Counter = 0;
        }
      }
      else
      {
        /* Motor run at V/f constant speed for a while.*/
        Motor.Counter ++;
        if (Motor.Counter > TIME_OF_VFRAMP)
        {
          /* Change flag: motor in stable mode of V/f ramp-up. */
          Motor.Mode_Flag = MOTOR_STABLE;
          Motor.Counter = 0;
        }
      }
    }
    else
    {
      if (SYSTEM_BE_IDLE)
      { /* If PWM duty cycle or POT ADC too low, */
        Motor.Counter = 0;                /* Clear counters. */
        Motor.Ramp_Counter = 0;
        Motor.State = STOP_MOTOR;         /* Next, go to Motor Stop. */
      }
    }

    /* Limit of |Vref| (16-bit). */
    #define SVM_VREF16_MAX    (32767U)
    if (Car2Polar.SVM_Vref16 > SVM_VREF16_MAX)
    {
      /*  Limit |Vref| maximum value.*/
      Car2Polar.SVM_Vref16 = SVM_VREF16_MAX;
    }

    /*  Update SVM PWM. */
    PWMSVM01_Update(Car2Polar.SVM_Vref16, Car2Polar.SVM_Angle16);

    /* Record SVM reference vector magnitude (32-bit) of last PWM cycle.*/
    Car2Polar.Vref32_Previous = Car2Polar.Vref32;
    Car2Polar.Vref32 = Car2Polar.SVM_Vref16 << CORDIC_SHIFT;

    /* Init for smooth transition from V/f to FOC closed-loop.*/
    Init_Smooth_Transition_To_FOC (Motor.Speed);
    Car2Polar.Vref_AngleQ31 = Car2Polar.SVM_Angle16 << 16;
}


/** V/f Ramp-Up, with rotor initial preposition/alignment ####
** Execution time: ?us (O3 - Optimize most).
* ----------------------------------------------------------*/
	void VF_FOC_OpenLoop_RampUp (void)
{
    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    //  Get_ADC_SingleShuntCurrent(&ADC);
    Current_Reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

    #else
    Get_ADCPhaseCurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);
    Current_Reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif

    /* To get I_Alpha and I_Beta of last PWM cycle, scale down I_Mag (i.e.: |I|) by 2/3.*/
    ClarkeTransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

    /* Update V/f voltage amplitude, Vref = Offset + Kω. */
    Car2Polar.SVM_Vref16 = VQ_VF_OFFSET + (VQ_VF_SLEW * (Motor.Speed >> RES_INC));

    /* To update angle θ (16-bit) of SVM reference vector Vref.*/
    Update_Vref_Angle (Motor.Speed);

    /* Motor is in transition mode,*/
    if (Motor.Mode_Flag == MOTOR_TRANSITION)
    {
      if (Motor.Speed < VF_TRANSITION_SPEED)
      {			/* Motor speed not reach V/f open-loop to MET/FOC transition speed.*/
        Motor.Ramp_Counter ++;
        if (Motor.Ramp_Counter > Motor.Ramp_Up_Rate)
        {	  /* Ramp up slowly in V/f.*/
          if (Motor.Ramp_Up_Rate > (RAMPUP_RATE << RAMP_RATIO_VF))
          {
            Motor.Ramp_Up_Rate --;
          }
          /* Motor speed ++.*/
          Motor.Speed += SPEEDRAMPSTEP;
          /* Clear ramp counter.*/
          Motor.Ramp_Counter = 0;
        }
      }
      else
      {
        /* Motor run at V/f constant speed for a while.*/
        Motor.Counter ++;
        if (Motor.Counter > TIME_OF_VFRAMP)
        {
          /* Change flag: motor in stable mode of V/f ramp-up.*/
          Motor.Mode_Flag = MOTOR_STABLE;
          Motor.Counter = 0;
        }
      }
    }
    else
    {
      /* Next, go to MET (Maximum Efficiency Tracking) closed-loop control. */
      Motor.State = MET_FOC;

      FOCInput.BEMF1 = 0;
      FOCInput.BEMF2 = 0;

      /* MET loop unlocked, Init ε Threshold LOW. (Initially, ε_Th = ε_Th_L.) */
      FOCInput.Threshold_LOW = THRESHOLD_LOW;
      FOCInput.Threshold_HIGH = THRESHOLD_HIGH;
      FOCInput.Threshold = (Motor.Speed * THRESHOLD_LOW) >> RES_INC;

      FOCInput.Phase_L = L_OMEGALI;
      FOCInput.Phase_L_Scale = SCALE_L;

      /* Resolution increase, use (16 + Res_Inc) bit to represent 360 deg. */
      FOCInput.Res_Inc = RES_INC;
      FOCInput.LPF_N_BEMF = SHIFT_MET_PLL;

      /* Motor in transition mode.*/
      Motor.Mode_Flag = MOTOR_TRANSITION;
      Motor.Counter = 0;
      Motor.Ramp_Counter = 0;
      Motor.Alignment_Counter = 0;
      /* Slower ramp up and ramp down for S-curve profile.*/
      Motor.Ramp_Up_Rate = RAMPUP_RATE << RATIO_S;
    }

    /* Limit of |Vref| (16-bit). */
    #define SVM_VREF16_MAX		(32767U)
    if (Car2Polar.SVM_Vref16 > SVM_VREF16_MAX)
    {
      /* Limit |Vref| maximum value.*/
      Car2Polar.SVM_Vref16 = SVM_VREF16_MAX;
    }

    /* Update SVM PWM.*/
    PWMSVM01_Update(Car2Polar.SVM_Vref16, Car2Polar.SVM_Angle16);

    /* Record SVM reference vector magnitude (32-bit) of last PWM cycle. */
    Car2Polar.Vref32_Previous = Car2Polar.Vref32;
    Car2Polar.Vref32 = Car2Polar.SVM_Vref16 << CORDIC_SHIFT;

    /* Init for smooth transition from V/f to FOC closed-loop.*/
    Init_Smooth_Transition_To_FOC (Motor.Speed);
    Car2Polar.Vref_AngleQ31 = Car2Polar.SVM_Angle16 << 16;

}


/** To calculate |Vref|sin(γ-θ), wL|I|, and ε=|Vref|sin(γ-θ)+wL|I|, for MET (Maximum Efficiency Tracking) .
** Execution time: 5.9us - 6.3us (O3 - Optimize most).
* ----------------------------------------------------------------------------------------------------------*/
void Init_Smooth_Transition_To_FOC (uint32_t Omega_Speed)
{

    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    // Get_ADC_SingleShuntCurrent(&ADC);
    Current_Reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

    #else
    Get_ADCPhaseCurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);

    Current_Reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif

    ClarkeTransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

    PLL_Imag(Car2Polar.Vref_AngleQ31,Clarke_Transform.I_Alpha_1Q31,Clarke_Transform.I_Beta_1Q31);

    /* Wait if CORDIC is still running calculation. Omit if CCU4 outputs debug information.*/
    while (CORDIC_IS_BSY)
    {
      continue;
    }
    PLL_Imag_GetResult(&PLL_Estimator);

    PLL_Vref(PLL_Estimator.Delta_IV, Car2Polar.Vref32,PI_PLL.Uk, FOCInput.Phase_L,&PLL_Estimator);

    /* CPU computes the following simultaneously when CORDIC #7 */
    /* γ, used for smooth MET->FOC transition and motor startup lock / fail / stall detection.*/
    I_AngleQ31 = PLL_Estimator.Delta_IV + Car2Polar.Vref_AngleQ31;

    /* Results of CORDIC #7 - Vrefxsin(Î³-Î¸) and Vrefxcos(γ-θ) */
    /* Wait if CORDIC is still running calculation.*/
    while (CORDIC_IS_BSY)
    {
      continue;
    }
    PLL_Vref_GetResult(&PLL_Estimator);

    /* Shift to get real result (16-bit).*/
    PLL_Estimator.VrefxSinDelta >>= CORDIC_SHIFT;

    PLL_Estimator.VrefxSinDelta = (PLL_Estimator.VrefxSinDelta * 311) >> 8;

    /* Unity gain LPF: Y[n] = Y[n-1] + (X[n]-Y[n-1])>>FOCInput.LPF_N_BEMF */
    /* |Vref|sin(γ-θ) with LPF.*/
    FOCInput.BEMF1 = (FOCInput.BEMF1 * ((1 << FOCInput.LPF_N_BEMF) - 1) + PLL_Estimator.VrefxSinDelta)
    >> FOCInput.LPF_N_BEMF;

    /* Îµ = |Vref|sin(γ-θ) + wL|I|. Motor rotates in one direction only. Rotor angle always increasing.*/
    Epsilon = FOCInput.BEMF1 +  FOCInput.BEMF2;

}



#define MET_VREF_STEP			(1U)
/** For control strategy MET (Maximum Efficiency Tracking)
** Execution time: ?us (O3 - Optimize most).
* -----------------------------------------------------------*/
uint16_t VF_Smooth_Transition_To_FOC (void)
{
    int32_t Vref_Change_Step;
    /* Î”V, change (increase / decrease, if necessary) step for |Vref| of SVM */
    uint16_t Mode_Flag = MOTOR_TRANSITION;
    /* A temporary variable to indicate leading or lagging of I to V.*/
    int32_t Flag_Leading_Lagging;
    /* 2or3-shunt phase current sensing.*/
    FOCInput.Ref_Speed = Motor.Speed;

    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    //  Get_ADC_SingleShuntCurrent(&ADC);
    Current_Reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

    #else
    Get_ADCPhaseCurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);

    Current_Reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif
    ClarkeTransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

    /* Î¸[k] = Î¸[k-1] + Ï‰[k]. Motor rotates in one direction only. Rotor angle always increasing.*/
    Car2Polar.Vref_AngleQ31 += (FOCInput.Ref_Speed << (16U-FOCInput.Res_Inc));

    /* To calculate |Vref|sin(Î³-Î¸), Ï‰L|I|, and Îµ = |Vref|sin(Î³-Î¸)+ Ï‰L|I|. */
    Init_Smooth_Transition_To_FOC (FOCInput.Ref_Speed);

    /* Bang-bang controller (aka: hysteresis controller, on-off controller) of MET Controller
    * Execution time: 1.5us~2.1us (O3 - Optimize most).
    * ------------------------------------------------------------------------------------------ */
    /* (Î³-Î¸) >= zero (i.e.: I is leading V) is an urgent condition, to increase |Vref| immediately.*/
    Flag_Leading_Lagging = FOCInput.BEMF1;

    if (Flag_Leading_Lagging >= 0)
    {
      /* An urgent condition,*/
      /* Increase |Vref| immediately.*/
      Car2Polar.Vref32 += (MET_VREF_STEP << CORDIC_SHIFT);

      /* MET loop unlocked, Îµ_Th = Îµ_Th_L Threshold_LOW.*/
      FOCInput.Threshold = (FOCInput.Ref_Speed * FOCInput.Threshold_LOW) >> FOCInput.Res_Inc;
    }
    else
    {	/* (Î³-Î¸) < zero.*/
      if (Epsilon < 0)
      {
        Epsilon = -Epsilon;
        /* Reverse Îµ value, i.e.: find |Îµ|. */
        Vref_Change_Step = -MET_VREF_STEP;

        /* |Vref| should be decreased (or no change) if Îµ < 0. */
        #define	MIN_AMPLITUDE	(20U)
        /* Minimum value of |Vref| */
        if (Car2Polar.Vref32 <= (MIN_AMPLITUDE << CORDIC_SHIFT))
        {
          /* |Vref| cannot decrease further if it is too small.*/
          Vref_Change_Step = 0;
        }
      }
      else
      {
        /* |Vref| should be increased (or no change) if Îµ >= 0.*/
        Vref_Change_Step = MET_VREF_STEP;
      }

      if (Epsilon > FOCInput.Threshold)
      {
        /* If |Îµ| > Îµ_Th,*/
        /* |Vref| changes (increase or decrease) by a step.*/
        Car2Polar.Vref32 += (Vref_Change_Step << CORDIC_SHIFT);

        /* MET loop unlocked, Îµ_Th = Îµ_Th_L Threshold_LOW.*/
        FOCInput.Threshold = (FOCInput.Ref_Speed * FOCInput.Threshold_LOW) >> FOCInput.Res_Inc;
      }
      else
      {	/* If |Îµ| <= Îµ_Th, |Vref| no need change.*/
        /*  MET loop locked,  Îµ_Th = Îµ_Th_H Threshold_HIGH.*/
        FOCInput.Threshold = (FOCInput.Ref_Speed * FOCInput.Threshold_HIGH) >> FOCInput.Res_Inc;

        if (FOCInput.Flag_State == 0)
        {
          /* If use 3-step motor start-up V/f-> MET-> FOC,*/
          /*  Change flag: motor in MET stable mode once it finds |Îµ| <= Îµ_Th, try to jump to FOC immediately. */
          Mode_Flag = MOTOR_STABLE;
        }
      }
    }
    /* Update SVM PWM.*/
    PWMSVM01_Update((Car2Polar.Vref32 >> CORDIC_SHIFT), Car2Polar.Vref_AngleQ31>>16U);

    return (Mode_Flag);
}


/** Stop the motor, check PWM or POT ADC (for adjusting motor speed)
 ** Execution time: ?us (O3 - Optimize most).
	* ---------------------------------------------------------------------*/
void Stop_Motor (void)
{
    static uint32_t  local_counter = 0;      // General purpose counter

    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      Current_Reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);
    #else
      /* 2or3-shunt 3-phase current reconstruction, to get Iu and Iv */
      Current_Reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
    #endif
      ClarkeTransform(Current.I_U, Current.I_V,Current.I_W, &Clarke_Transform);
      local_counter ++;
#if(uCPROBE_GUI == ENABLED)
    if(Update_KpKi == UCPROBE_BUTTON_PRESSED)
    {
      /* Disable CCU8 interrupt to enable update of user parameter */
      NVIC_DisableIRQ(CCU80_0_IRQn);

      User_Para[1] = PI_Speed.Kp;
      User_Para[2] = PI_Speed.Ki;
      User_Para[3] = PI_Speed.Scale_KpKi;

      User_Para[4] = PI_Torque.Kp;
      User_Para[5] = PI_Torque.Ki;
      User_Para[6] = PI_Torque.Scale_KpKi;

      User_Para[7] = PI_Flux.Kp;
      User_Para[8] = PI_Flux.Ki;
      User_Para[9] = PI_Flux.Scale_KpKi;

      User_Para[10] = PI_PLL.Kp;
      User_Para[11] = PI_PLL.Ki;
      User_Para[12] = PI_PLL.Scale_KpKi;

      uint32_t *MotorCONF_Address = MotorConfig_Addr;
      uint32_t *UserConfig_Address = &User_Para[0];
      XMC_FLASH_ProgramVerifyPage(MotorCONF_Address,UserConfig_Address);

      NVIC_SystemReset();
    }
#endif

    if (SYSTEM_BE_IDLE)
    {
      /* If system is idle, i.e.: PWM duty cycle or POT ADC too low.*/
      /* Reset counter, local_counter < TIME_OF_STOP to prevent it from re-start of motor.*/
      local_counter = 0;
      Motor.Speed = 0;
    }


    if (local_counter > TIME_OF_STOP)
    {
      local_counter = 0;
      #if (CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      /* Init ADC, for current sensing, ADC of DC link Vdc (and POT). Do at later stage of the init */
      ADC_Init();
      ADC_DCLink_Init();
      ADC_Pot_Init();
      #endif
      CCU8_Init();
      Variables_Init ();
      CCUx_SynStart();

      /* Direct FOC startup. Motor startup to FOC closed-loop directly, no V/f or MET*/
      #if(MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC || MY_FOC_CONTROL_SCHEME == CONSTANT_TORQUE_DIRECT_FOC || MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC)
      /* Next, go to rotor initial preposition/alignment.*/
      Motor.State = PRE_POSITIONING;
      #elif(MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC)
      Motor.State = VFOPENLOOP_RAMP_UP;             // Next, go to V/f ramp-up and re-start the motor.
      #elif(MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_ONLY)
      Motor.State = BRAKE_BOOTSTRAP;
      #endif

      if (Motor.State == VFOPENLOOP_RAMP_UP)
      {
        Motor.Ramp_Up_Rate = (RAMPUP_RATE * 40); /* In V/f, much slower initial ramp up for S-curve profile. */
      }
    }
    else
    {
      /* To update angle θ (16-bit) of SVM reference vector Vref */
      Update_Vref_Angle (Motor.Speed);
      /* Update SVM PWM, brake motor.*/
      PWMSVM01_Update(0, Car2Polar.SVM_Angle16);
    }

}


/*  Shift times for unity gain LPF: Y[n] = Y[n-1] + (X[n]-Y[n-1])>>SHIFT_BIAS_LPF. */
/** To brake the motor, charge gate driver bootstrap capacitors (if any)
 ** Execution time: ?us (O3 - Optimize most).
	* -------------------------------------------------------------------------*/
void VF_FOC_Brake_Motor_Bootstrap_Charge (void)
{
    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
   // Get_ADC_SingleShuntCurrent(&ADC);
    ADC.ADC_Bias = (uint32_t) ((ADC.ADC_Bias * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + ADC.ADC_ResultTz1)
                                   >> SHIFT_BIAS_LPF;
    #else
		Get_ADCPhaseCurrent(SVM.PreviousSectorNo, SVM.CurrentSectorNo, &ADC);

		ADC.ADC_Bias_Iu = (ADC.ADC_Bias_Iu * ((1<<SHIFT_BIAS_LPF)-1) + ADC.ADC_Iu)>>SHIFT_BIAS_LPF;		// Read Iu ADC bias during motor brake.
		ADC.ADC_Bias_Iv = (ADC.ADC_Bias_Iv * ((1<<SHIFT_BIAS_LPF)-1) + ADC.ADC_Iv)>>SHIFT_BIAS_LPF;		// Read Iv ADC bias.
		ADC.ADC_Bias_Iw = (ADC.ADC_Bias_Iw * ((1<<SHIFT_BIAS_LPF)-1) + ADC.ADC_Iw)>>SHIFT_BIAS_LPF;		// Read Iw ADC bias.
    #endif

    #define IV_ADC_BIAS   (2045)
    #define IW_ADC_BIAS   (2048)

		/* If system is idle, i.e.: PWM duty cycle or POT ADC too low.*/
	  Motor.Counter ++;
	if (SYSTEM_BE_IDLE)
	{
	  /* cannot go to ramp up, keep motor braking.*/
		Motor.Counter = 0;
	}



	if (Motor.Counter > BRAKE_TIME)
	{
	  /* Timer decides when to stop motor braking.*/
		Motor.State = VFOPENLOOP_RAMP_UP;
		/* Next, go to V/f ramp-up.*/
		Motor.Mode_Flag = MOTOR_TRANSITION;
		/* Motor in transition mode.*/
		Motor.Counter = 0;
		/* Clear counters.*/
		Motor.Ramp_Counter = 0;
		if (Motor.State == VFOPENLOOP_RAMP_UP)
		{
		  /* In V/f, much slower initial ramp up for S-curve profile.*/
			Motor.Ramp_Up_Rate = (RAMPUP_RATE * 40);
		}
		Current.I_U = 0;
		Current.I_V = 0;
		Current.I_W = 0;
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
    ADC.ADC3Trig_Point = 0;
    ADC.ADC4Trig_Point = 0;
    ADC.ADC_Result1 = 0;
    ADC.ADC_Result2 = 0;
    ADC.Result_Flag = RESULTS_ADCTZ12;
    ADCTZ12_TriggerSetting ();              /* Set ADC trigger for ADCTz1/Tz2, for single-shunt current sensing only. */
#endif
	}


}

RAM_ATTRIBUTE void Linear_Ramp_Generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val)
{
  if (*reference_val != set_val)
  {
    /* in FOC, ωref not reach the target speed.*/
    if (*reference_val < set_val)
    {
      /* Speed ramp counter ++.*/
      Motor.Ramp_Counter ++;
      if (Motor.Ramp_Counter > Motor.Ramp_Up_Rate)
      {
        /* Ramp up slowly (if needed) at start of FOC.*/
        if (Motor.Ramp_Up_Rate > rampup_rate)
        {
          /* Increase acceleration step by step.*/
          Motor.Ramp_Up_Rate --;
        }

        /*  ωref ++.*/
        *reference_val += speedrampstep;
        /* Clear ramp counter. */
        Motor.Ramp_Counter = 0;
      }
    }
    else
    {
      /* Speed ramp counter ++.*/
      Motor.Ramp_Counter ++;
      if (Motor.Ramp_Counter > rampdown_rate)
      {
        /* ωref --.*/
        *reference_val -= speedrampstep;
        Motor.Ramp_Counter = 0;
      }
    }
  }
  else
  {
    /* ωref reach the target speed */
    /* Update counter */
    Motor.Counter ++;
    /*15, 150 or 1500. Time that FOC becomes stable, x PWM period.*/
    if (Motor.Counter > 2U)
    {
      /* Change flag: in FOC stable mode. */
      Motor.Mode_Flag = MOTOR_STABLE;
      Motor.Counter = 0;
      /* Clear counter */
      Motor.Ramp_Counter = 0;
    }
  }
}

/*  To use S-curve profile in motor ramp up / down. Comment out to use trapezoidal profile. */
#define S_CURVE_PROFILE   1
/* Speed threshold for entering second S-curve of ramp up / down. */
#define SPEED_TH_2ND_S    (SPEED_LOW_LIMIT >> 0U)

RAM_ATTRIBUTE void SCurve_Ramp_Generator(int32_t set_val, int32_t rampup_rate, int32_t rampdown_rate,
                                         int32_t speedrampstep, int32_t *reference_val)
{
  if (*reference_val == set_val)
  {
    /* For most of the time, motor ref speed = speed set by POT ADC or PWM. */
    Motor.Ramp_Counter = 0;
    Motor.Ramp_Up_Rate = rampup_rate << RATIO_S;

    /* Reset to slower ramp up and ramp down for S-curve profile. */
    Motor.Ramp_Dn_Rate = rampdown_rate << (RATIO_S - 1);
  }
  else if (*reference_val < set_val)
  {
    /* Motor ref speed lower than speed set by POT or PWM. */
    /* Speed ramp counter ++. */
    Motor.Ramp_Counter++;
    if (Motor.Ramp_Counter > Motor.Ramp_Up_Rate)
    {
      if ((set_val - *reference_val) > SPEED_TH_2ND_S)
      {
        /* First S-curve of ramp up, and constant acceleration. */
        if (Motor.Ramp_Up_Rate > rampup_rate)
        {
          /* Increase acceleration step by step. */
          Motor.Ramp_Up_Rate--;
        }
      }
      else
      {
        /* Second S-curve of ramp up. */
        if (Motor.Ramp_Up_Rate < (rampup_rate << RATIO_S))
        {
          Motor.Ramp_Up_Rate++;
        }
      }
      /* Motor ref speed ++. */
      *reference_val += speedrampstep;
      Motor.Ramp_Counter = 0;
    }
  }
  else
  {
    /* Motor ref speed higher than speed set by POT or PWM. */
    /* Speed ramp counter ++. */
    Motor.Ramp_Counter++;
    if (Motor.Ramp_Counter > Motor.Ramp_Dn_Rate)
    {
      if ((*reference_val - set_val) > SPEED_TH_2ND_S)
      {
        /* First S-curve of ramp down, and constant deceleration. */
        if (Motor.Ramp_Dn_Rate > rampdown_rate)
        {
          /* Increase deceleration step by step. */
          Motor.Ramp_Dn_Rate--;
        }
      }
      else
      {
        /* Second S-curve of ramp down. */
        if (Motor.Ramp_Dn_Rate < (rampdown_rate << (RATIO_S - 1)))
        {
          Motor.Ramp_Dn_Rate++;
        }
      }

        if (ADC.ADC_DCLink < VDC_MAX_LIMIT)
        {
          /* If DC link voltage Vdc is too high, stop ramp-down motor.*/
          /* Motor ref speed --.*/
          *reference_val -= speedrampstep;
        }
        Motor.Ramp_Counter = 0;
      }

    }

 }



/* Non-Real-Time Tasks Configuration */
/* 2 ~ 100, x CCU8 PWM period. For tasks that don't need real-time computing.*/
#define NON_REALTIME_RATE 64
#define POTADC_LPF    (5U)          // (5U). ADC uses LPF.
/** Miscellaneous works in CCU80_0_IRQHandler, such as tasks that don't need real-time computing
	* -------------------------------------------------------------------------------------------------*/
RAM_ATTRIBUTE void Misc_Works_of_IRQ (void)
{
  /* Handle tasks that don't need real-time computing:*/
  #if(UART_ENABLE == USIC_DISABLED_ALL)
      uint16_t pot_adc_result;
  #endif
  /* Counter ++. */
	Motor.Non_RealTime_Counter ++;
	if (Motor.Non_RealTime_Counter > NON_REALTIME_RATE)
	{
	    /* Reset counter.*/
		  Motor.Non_RealTime_Counter = 0;

      #if((MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC))
        /* POT ADC values 0 ~ 2^12 represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/
        Motor.Speed_by_POT_PWM = SPEED_LOW_LIMIT + (((SPEED_HIGH_LIMIT - SPEED_LOW_LIMIT) * ADC.ADC_POT) >> 12);
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        Motor.Speed_by_POT_PWM = MIN_MAX_LIMIT(Motor.Speed_by_POT_PWM, SPEED_HIGH_LIMIT, SPEED_LOW_LIMIT);
      #elif(MY_FOC_CONTROL_SCHEME == CONSTANT_TORQUE_DIRECT_FOC)
        /* POT ADC values 0 ~ 2^12 represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/


        Motor.Speed_by_POT_PWM = USER_IQ_REF_LOW_LIMIT + (((USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT) * ADC.ADC_POT) >> 12);
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        Motor.Speed_by_POT_PWM = MIN_MAX_LIMIT(Motor.Speed_by_POT_PWM, USER_IQ_REF_HIGH_LIMIT, USER_IQ_REF_LOW_LIMIT);

      #elif(MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC)
        /* POT ADC values 0 ~ 2^12 represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/
        Motor.Speed_by_POT_PWM = USER_VQ_REF_LOW_LIMIT + (((USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT) * ADC.ADC_POT) >> 12);
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        Motor.Speed_by_POT_PWM = MIN_MAX_LIMIT(Motor.Speed_by_POT_PWM, USER_VQ_REF_HIGH_LIMIT, USER_VQ_REF_LOW_LIMIT);
      #endif
			/* Service watchdog. Without WDT service regularly , it will reset system.*/
			XMC_WDT_Service();

			Motor.UART_Counter ++;
			if (Motor.UART_Counter > (3U))
			{
        /* Reset counter.*/
        Motor.UART_Counter = 0;
        #if(UART_ENABLE == USIC_DISABLED_ALL)
        pot_adc_result = XMC_VADC_GROUP_GetResult(VADC_POT_GROUP,VADC_POT_RESULT_REG);
        /* POT ADC LPF. Read RES7 for ADC result (Previous ADC result). */
        ADC.ADC_POT = (ADC.ADC_POT * ((1<<POTADC_LPF)-1) + pot_adc_result) >> POTADC_LPF;

        Speed_in_rpm = (Motor.Speed * SPEED_TO_RPM ) >> SCALE_SPEED_TO_RPM;

        #else
        /* Use UART to adjust POT ADC values, and hence motor speed.*/
        /* Use UART to set POT ADC, by polling.*/
        UART_Set_POT_ADC ();
        #endif
			}

	}

}
