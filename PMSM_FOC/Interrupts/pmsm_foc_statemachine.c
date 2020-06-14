/**
 * @file pmsm_foc_statemachine.c
 * @date 2015-12-17
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
 * @file pmsm_foc_statemachine.c
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

#include <XMC1300.h>                      /* SFR declarations of the selected device */
#include "pmsm_foc_statemachine.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define DEGREE_60       (10922U)                   /* 60° angle (0 ~ 2^16 represent electrical angle 0° ~ 360°). */
#define RATIO_T0_111    (2U)                       /* = 2 for standard SVM. */
#define PERIOD_OF_PWM   ((uint16_t)PERIOD_REG + 1U) /* Period of a CCU8 PWM. */
#define RATIO_T0_111    (2U)                       /* = 2 for standard SVM.*/
#define CCU8_PERIOD_2ND  (PERIOD_REG - TZ_PZVX2-1)   // Second CCU83 period, for ADC trigger
#define SPEED_SS_THRESHOLD ((SPEED_HIGH_LIMIT * 3U) >> 2U)
#define FOC_EXIT_SPEED  (SPEED_LOW_LIMIT * 3)
#define NON_REALTIME_RATE 64


/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/

extern MotorControlType Motor; /* Motor control information */
extern SVMType SVM; /* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon. */
extern FOCOutputType FOCOutput; /* Output for FOC LIB. */
extern FOCInputType FOCInput; /* Parameters input for FOC LIB. */
extern Car2PolType Car2Polar;
extern CurrentType Current; /* Motor current and current space vector. */
extern PLL_EstimatorType PLL_Estimator;
extern ADCType ADC;
extern const uint16_t Sinus60_tab[];  /* Sine LUT used for SVM calculations, array size 256 or 1024. */
/*********************************************************************************************************************
 * LOCAL ROUTINES
 ********************************************************************************************************************/

RAM_ATTRIBUTE void DirectFOCStartUp_CCU80_0_IRQHandler(void);
RAM_ATTRIBUTE void DirectFOCIQStartUp_CCU80_0_IRQHandler(void);
RAM_ATTRIBUTE void DirectFOCVQStartUp_CCU80_0_IRQHandler(void);
RAM_ATTRIBUTE void VF_FOC_CCU80_0_IRQHandler(void);
RAM_ATTRIBUTE void VF_ONLY_CCU80_0_IRQHandler(void);

#if (MY_FOC_CONTROL_SCHEME == CONSTANT_TORQUE_DIRECT_FOC)
/*
 * Periodic CCU80 Period Match Interrupt, function called every CCU8 PWM cycle
 * It is the state machine of Sensorless FOC
 */
RAM_ATTRIBUTE void DirectFOCIQStartUp_CCU80_0_IRQHandler(void)
{


  switch (Motor.State)
	{
    case FOC_CLOSED_LOOP:

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        ADC34_TriggerSetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }

#endif

      /* Execution time: 22us */
      //XMC_GPIO_SetOutputHigh(P1_2);
      FOC_Torque_Controller();                          /* Toggle LED in Main.c, CPU load = 28.2 us */
      //XMC_GPIO_SetOutputLow(P1_2);


      /********************************** Misc_Works_of_FOC ();**************************************************/
      Misc_Works_of_FOC();


/**********************************   PWMSVM01_Update((Car2Polar.Vref32 >> CORDIC_SHIFT), (Car2Polar.Vref_AngleQ31 >> 16U));**************************************************/

      PWMSVM01_Update(Car2Polar.Vref32 >> CORDIC_SHIFT, Car2Polar.Vref_AngleQ31 >> 16U);

      /* Record SVM sector information. */
      FOCOutput.New_SVM_SectorNo = SVM.CurrentSectorNo;

        break;
			case BRAKE_BOOTSTRAP:
			  /* Brake the motor before motor startup. Charge gate driver bootstrap capacitors (if any)*/
				DirectFOC_StartUp_Brake_Motor_Bootstrap_Charge ();
				break;

			case STOP_MOTOR:

				Stop_Motor ();
				break;

			case PRE_POSITIONING:

				DirectFOCRotor_Pre_Positioning ();
				break;

			default:
				/* For trap protection if CCU8_TRAP_ENABLE (CCU8 TRAP functionality enabled)*/
				Error_Handling ();

				break;
		}

  /* CCU4_Debug3Output(Current.I_U, 1, 11, Current.I_V, 1, 11); */

  /*
   * Miscellaneous works in CCU80_0_IRQHandler, such as tasks that don't need real-time computing.
   * Execution time: 1.65us
   */
  /**********************************   Misc_Works_of_IRQ ();**************************************************/

  Misc_Works_of_IRQ ();
  Motor.speed_in_rpm = (Motor.Speed * SPEED_TO_RPM ) >> SCALE_SPEED_TO_RPM;

}



#elif (MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC)

RAM_ATTRIBUTE void DirectFOCVQStartUp_CCU80_0_IRQHandler(void)
{
  /* SVM time T1, T2. */
  uint16_t T1, T2, T1nT2;
  /* T0, T0_111 for first [111], T0_111 + T1/2, T0_111 + T2/2, T0_111 + (T1+T2)/2. */
  uint16_t T0, T0_111, T0nHalfT1,T0nHalfT2, T0nHalfT1nT2;
  uint16_t AngleTemp, AmplitudeTemp,SectorAngle;
  uint16_t DCLink_adc_result;
  uint16_t IDCLink_adc_result;
  static uint32_t Vq_counter = 0;

  //XMC_GPIO_SetOutputHigh(P1_2);
  switch (Motor.State)
  {
    case FOC_CLOSED_LOOP:

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        ADC34_TriggerSetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }

#endif

      /* Execution time: 22us */
      //XMC_GPIO_SetOutputHigh(P1_2);
      FOC_VQ_Controller();                          /* Toggle LED in Main.c, CPU load = 28.2 us */
      //XMC_GPIO_SetOutputLow(P1_2);


      /********************************** Misc_Works_of_FOC ();**************************************************/
      VADC->BRSMR |= (uint32_t)VADC_BRSMR_LDEV_Msk;

      /* DC link ADC LPF. Read RES5 for ADC result (Previous ADC result) */
      DCLink_adc_result = VADC_VDC_GROUP->RES[VADC_VDC_RESULT_REG];
      ADC.ADC_DCLink = (ADC.ADC_DCLink * ((1<<ADCLPF)-1) + DCLink_adc_result) >> ADCLPF;

      /* IDC link ADC LPF.  */
      IDCLink_adc_result = VADC_IDC_GROUP->RES[VADC_IDC_RESULT_REG];
      ADC.ADC_IDCLink = (ADC.ADC_IDCLink * ((1 << ADCLPF) - 1) + IDCLink_adc_result) >> ADCLPF;


      /**********************************  Linear_Torque_Ramp_Generator(Motor.Speed_by_POT_PWM,USER_IQ_RAMPUP, USER_IQ_RAMPDOWN, &FOCInput);**************************************************/
      Vq_counter++;
      if(Vq_counter >= USER_VQ_RAMP_SLEWRATE)
      {
         Vq_counter = 0;
         if( FOCInput.Vq < Motor.Speed_by_POT_PWM)
         {
           FOCInput.Vq += USER_VQ_RAMPUP;
         }
         else if(FOCInput.Vq > Motor.Speed_by_POT_PWM)
         {
           if((FOCInput.Vq >= USER_VQ_RAMPDOWN )&& (ADC.ADC_DCLink < VDC_MAX_LIMIT))
           {
             FOCInput.Vq -=  USER_VQ_RAMPDOWN;
           }

         }
      }

      /* Limit protection for ref_iq, the max value is capped up to 1Q15*/
      if(FOCInput.Vq > USER_VQ_REF_HIGH_LIMIT)
       FOCInput.Vq = USER_VQ_REF_HIGH_LIMIT - 1;

      if((SYSTEM_BE_IDLE) && Motor.Speed < FOC_EXIT_SPEED)
      //if((SYSTEM_BE_IDLE) && (FOCInput.Vq <= Motor.Speed_by_POT_PWM) && Motor.Speed < FOC_EXIT_SPEED)
      {
         /* Clear counters.*/
         Motor.Counter = 0;
         /* Next, go to Motor Stop (may brake motor and cause vibration).*/
         Motor.State = STOP_MOTOR;
      }


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


/**********************************   PWMSVM01_Update((Car2Polar.Vref32 >> CORDIC_SHIFT), (Car2Polar.Vref_AngleQ31 >> 16U));**************************************************/
      AmplitudeTemp = (Car2Polar.Vref32 >> CORDIC_SHIFT);
      AngleTemp = (Car2Polar.Vref_AngleQ31 >> 16U);

      SVM.PreviousSectorNo = SVM.CurrentSectorNo;           /* Record sector information of last PWM cycle. */

      /* Angle: 0 ~ 2^16 represent electrical angle 0° ~ 360°. Or = (uint16_t)(Angle >> 8) * 6; */
      AngleTemp = (AngleTemp >> ANGLETEMP_SHIFT) * 6U;
      SectorAngle = AngleTemp & SECTOR_ANGLE_AND;         /* Relative angle θrel in each sector. */
      SVM.CurrentSectorNo = AngleTemp >> SECTOR_NO_SHIFT; /* Update new SVM sector number. Or = (AngleTemp >> 8) & 0x7U; */

      /* Calculate T1 / T2 by LUT. */
      T1 = (((AmplitudeTemp * Sinus60_tab[MAX_LUT_INDEX - SectorAngle]) >> 15) * SVM_LUT_SCALE) >> 15;
      T2 = (((AmplitudeTemp * Sinus60_tab[SectorAngle]) >> 15) * SVM_LUT_SCALE) >> 15;

      T1nT2 = T1 + T2;                        /* Temp variable for (T1+T2) <= PERIOD_REG. */

      if (T1nT2 > PERIOD_OF_PWM)
      {
        #define SHIFT_OVERMODULATION    (5U)
        MATH->DIVCON = (0x00008004 | (SHIFT_OVERMODULATION << 16U) | (SHIFT_OVERMODULATION << 8U));
        MATH->DVD = T1 * PERIOD_OF_PWM;
        MATH->DVS = T1nT2;

        T1nT2 = PERIOD_OF_PWM;

        while (MATH->DIVST)
        {
          /* CPU wait */
        }

        T1 = MATH->QUOT;
        T2 = PERIOD_OF_PWM - T1;

      }

      T0 = PERIOD_OF_PWM - T1nT2;

      if (T0 > T0_THRESHOLD)
      {
        SVM.Flag_3or2_ADC = USE_ALL_ADC;      /* To use all (e.g.: three) ADC samplings for current reconstruction. */
      }
      else
      {
        SVM.Flag_3or2_ADC = USE_2_ADC;        /* To use two ADC samplings for current reconstruction. */
      }


      T0_111 = T0 >> RATIO_T0_111;                        /* T0_111, time of first [111]. */
      T0nHalfT1 = (T0 + (uint16_t)(T1 << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;       /* T0_111 + T1/2. */
      T0nHalfT2 = (T0 + (uint16_t)(T2 << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;       /* T0_111 + T2/2. */
      T0nHalfT1nT2 = (T0 + (uint16_t)((T1 + T2) << (RATIO_T0_111 - 1U))) >> RATIO_T0_111; /* T0_111 + (T1+T2)/2. */

      /* Standard 7-segment symmetric PWM: */
      switch (SVM.CurrentSectorNo)
      {
        case 0:                           /* Sector A */
            CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0nHalfT1nT2;
            CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

            CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0nHalfT2;
            CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT2);

            CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0_111;
            CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

          break;
        case 1:                           /* Sector B */

            CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0nHalfT1;
            CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1);

            CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0nHalfT1nT2;
            CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

            CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0_111;
            CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

          break;
        case 2:                           /* Sector C */
            CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0_111;
            CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

            CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0nHalfT1nT2;
            CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

            CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0nHalfT2;
            CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT2);

          break;
        case 3:                           /* Sector D */

            CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0_111;
            CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

            CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0nHalfT1;
            CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1);

            CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0nHalfT1nT2;
            CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

          break;
        case 4:                           /* Sector E */
            CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0nHalfT2;
            CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT2);

            CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0_111;
            CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

            CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0nHalfT1nT2;
            CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

          break;
        default:                          /* Process for all other cases, Sector F = 5. */
            CCU8_MODULE_PHASE_U->CR1S = (uint32_t) T0nHalfT1nT2;
            CCU8_MODULE_PHASE_U->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1nT2);

            CCU8_MODULE_PHASE_V->CR1S = (uint32_t) T0_111;
            CCU8_MODULE_PHASE_V->CR2S = (uint32_t) (PERIOD_OF_PWM - T0_111);

            CCU8_MODULE_PHASE_W->CR1S = (uint32_t) T0nHalfT1;
            CCU8_MODULE_PHASE_W->CR2S = (uint32_t) (PERIOD_OF_PWM - T0nHalfT1);


          break;
      } /* End of switch (SVM.CurrentSectorNo), standard 7-segment symmetric PWM. */

      /* Enable shadow transfer for slice 0,1,2 for CCU80 Kernel. */
      CCU8_MODULE->GCSS |= (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0|XMC_CCU8_SHADOW_TRANSFER_SLICE_1|XMC_CCU8_SHADOW_TRANSFER_SLICE_2);
      /* ADC triggered always at centre of [000] (fixed centre position in one PWM). */
      /* Record SVM sector information. */
      FOCOutput.New_SVM_SectorNo = SVM.CurrentSectorNo;

        break;
      case BRAKE_BOOTSTRAP:
        /* Brake the motor before motor startup. Charge gate driver bootstrap capacitors (if any)*/
        DirectFOC_StartUp_Brake_Motor_Bootstrap_Charge ();
        break;

      case STOP_MOTOR:

        Stop_Motor ();
        break;

      case PRE_POSITIONING:

        DirectFOCRotor_Pre_Positioning ();
        break;

      default:
        /* For trap protection if CCU8_TRAP_ENABLE (CCU8 TRAP functionality enabled)*/
        Error_Handling ();

        break;
    }

  /* CCU4_Debug3Output(Current.I_U, 1, 11, Current.I_V, 1, 11); */

  /*
   * Miscellaneous works in CCU80_0_IRQHandler, such as tasks that don't need real-time computing.
   * Execution time: 1.65us
   */
  /**********************************   Misc_Works_of_IRQ ();**************************************************/

    /* Counter ++. */
    Motor.Non_RealTime_Counter ++;
    if (Motor.Non_RealTime_Counter > NON_REALTIME_RATE)
    {
        /* Reset counter.*/
        Motor.Non_RealTime_Counter = 0;

          /* POT ADC values 0 ~ 2^12 represent motor target speed of SPEED_LOW_LIMIT ~ SPEED_HIGH_LIMIT:*/
          Motor.Speed_by_POT_PWM = USER_VQ_REF_LOW_LIMIT + (((USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT) * ADC.ADC_POT) >> 12);
          /* Limit speed, in case ADC values not 0 ~ 2^12.*/
          Motor.Speed_by_POT_PWM = MIN_MAX_LIMIT(Motor.Speed_by_POT_PWM, USER_VQ_REF_HIGH_LIMIT, USER_VQ_REF_LOW_LIMIT);

        /* Service watchdog. Without WDT service regularly , it will reset system.*/
        XMC_WDT_Service();

        Motor.UART_Counter ++;
        if (Motor.UART_Counter > (3U))
        {
          /* Reset counter.*/
          Motor.UART_Counter = 0;
          #if(UART_ENABLE == USIC_DISABLED_ALL)
          #define POTADC_LPF    (5U)          /* ADC uses LPF. */
          uint16_t pot_adc_result;
          pot_adc_result = XMC_VADC_GROUP_GetResult(VADC_POT_GROUP,VADC_POT_RESULT_REG);
          /* POT ADC LPF. Read RES7 for ADC result (Previous ADC result). */
          ADC.ADC_POT = (ADC.ADC_POT * ((1<<POTADC_LPF)-1) + pot_adc_result) >> POTADC_LPF;

          #else
          /* Use UART to adjust POT ADC values, and hence motor speed.*/
          /* Use UART to set POT ADC, by polling.*/
          UART_Set_POT_ADC ();
          #endif
        }

    }
    //XMC_GPIO_SetOutputLow(TEST_PIN);                           //P1.2 output 1 for debug. */

}

#elif (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC)

RAM_ATTRIBUTE void VF_FOC_CCU80_0_IRQHandler(void)
{
  //XMC_GPIO_SetOutputLow(TEST_PIN);                           //P0.4 output 1 for debug. */
	switch(Motor.State)
	{
    case FOC_CLOSED_LOOP:
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        ADC34_TriggerSetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }
#endif
      //XMC_GPIO_SetOutputLow(TEST_PIN);                           //P0.4 output 1 for debug. */
      FOC_Speed_Controller ();
      //XMC_GPIO_SetOutputHigh(TEST_PIN);                           //P0.4 output 1 for debug. */
      /* Update SVM PWM */
      PWMSVM01_Update((Car2Polar.Vref32 >> CORDIC_SHIFT), (Car2Polar.Vref_AngleQ31 >> 16U));

      /* Miscellaneous works in FOC, such as ramp up, speed adjustment, stop motor, etc. */
      Misc_Works_of_FOC ();

      /* Record SVM sector information*/
      FOCOutput.New_SVM_SectorNo = SVM.CurrentSectorNo;
    break;

		case BRAKE_BOOTSTRAP:
		  /* Brake the motor before motor startup. Charge gate driver bootstrap capacitors (if any). */
			VF_FOC_Brake_Motor_Bootstrap_Charge ();
		break;

		case STOP_MOTOR:
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        ADC34_TriggerSetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }
#endif
			Stop_Motor ();
		break;

		case VFOPENLOOP_RAMP_UP:
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        ADC34_TriggerSetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }
#endif
			VF_FOC_OpenLoop_RampUp();

		break;

		case MET_FOC:
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        ADC34_TriggerSetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }
#endif
			Motor.Mode_Flag = VF_Smooth_Transition_To_FOC();
			Misc_Works_of_MET();

		break;

		default:
		 /* For trap protection if CCU8_TRAP_ENABLE (CCU8 TRAP functionality enabled) */
			Error_Handling ();
		break;

	}

  /* Miscellaneous works in CCU80_0_IRQHandler, such as tasks that don't need real-time computing. */
  Misc_Works_of_IRQ();

  //XMC_GPIO_SetOutputHigh(TEST_PIN);                           //P0.4 output 1 for debug. */
}


#elif (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_ONLY)

/*
 * Periodic CCU80 Period Match Interrupt, function called every CCU8 PWM cycle
 * It is the state machine of Sensorless FOC
 */
RAM_ATTRIBUTE void VF_ONLY_CCU80_0_IRQHandler (void)
{

  switch(Motor.State)
  {
    case BRAKE_BOOTSTRAP:
      /* Brake the motor before motor startup. Charge gate driver bootstrap capacitors (if any).*/
      VF_FOC_Brake_Motor_Bootstrap_Charge ();
    break;

    case STOP_MOTOR:
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        ADC34_TriggerSetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }
#endif
      Stop_Motor ();
    break;

    case VFOPENLOOP_RAMP_UP:
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      if(SVM.SVM_Flag == SVM_USE_PZV)
      {
        ADC34_TriggerSetting(&ADC);
        ADC.Result_Flag = RESULTS_ADCTZ12;
      }
      else
      {
        /* For next ADC interrupt, to read ADC results of standard SVM (4-segment).*/
        ADC.Result_Flag = RESULTS_STANDARD_SVM;
      }
#endif
      VF_ONLY_OpenLoop_RampUp();
    break;

    default:
      /* For trap protection if CCU8_TRAP_ENABLE (CCU8 TRAP functionality enabled).*/
      Error_Handling ();
    break;

    }
    /* Miscellaneous works in CCU80_0_IRQHandler, such as tasks that don't need real-time computing.*/
  Misc_Works_of_IRQ(); /*
                        * Miscellaneous works in CCU80_0_IRQHandler,
                        * such as tasks that don't need real-time computing.
                        */
  CCU4_Debug3Output(0, 1, 11, Current.I_U, 1, 12);
}

#endif
