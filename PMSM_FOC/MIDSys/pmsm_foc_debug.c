/**
 * @file pmsm_foc_debug.c
 * @date 2016-01-18
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
 *********************************************************************************************************************
 *
 * @file pmsm_foc_debug.c
 * @date 18 Jan, 2016
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
 * 18 Jan 2016 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include <XMC1300.h>
#include "pmsm_foc_debug.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/


/***********************************************************************************************************************
 * GLOBAL DATA
***********************************************************************************************************************/
extern ADCType ADC;
extern MotorControlType Motor;                     /* Motor control information */
int32_t Speed_in_rpm;
int32_t Motor_speed_in_rpm;
/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to use CCU4 Debug with 2 Outputs, P0.4 and P1.0 */
void CCU4_Debug3Output(int32_t In04, uint16_t In04_Flag, uint16_t In04_N, int32_t In10, uint16_t In10_Flag,
                       uint16_t In10_N)
{
  #if ((DEBUG_PWM_0_ENABLE == 1U) || (DEBUG_PWM_1_ENABLE == 1U))
  int32_t Tmp_CRS;									/* Tmp for CCU4 debug. */
  #endif

  #if	(DEBUG_PWM_0_ENABLE == 1U)
  /* Update CCU40.OUT0 (P1.0) duty-cycle for debug. */
  if (In10_Flag == 0U)
  {
    Tmp_CRS = (In10 * DEBUG_PWM_PERIOD_CNTS) >> In10_N; /* In10 is a positive integer, In10 < 2^In10_N. */
  }
  else
  {
    /* In10 is a positive or negative integer, -2^In10_N < In10 < 2^In10_N. */
    Tmp_CRS = ((In10 + (1U << In10_N)) * DEBUG_PWM_PERIOD_CNTS) >> (In10_N + 1U);
  }

  if (Tmp_CRS < 0)
  {
    Tmp_CRS = REVERSE_CRS_OR_0;
  }
  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_0_SLICE, Tmp_CRS);		/* Update CCU41 Shadow Compare Register. */
  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t)DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk);
  #endif /* (DEBUG_PWM_0_ENABLE == 1) */

  #if	(DEBUG_PWM_1_ENABLE == 1U)
  /* Update CCU40.OUT1 (P0.4) duty-cycle for debug. */
  if (In04_Flag == 0U)
  {
    Tmp_CRS = (In04 * DEBUG_PWM_PERIOD_CNTS) >> In04_N; /* In04 is a positive integer. */
  }
  else
  {
    /* In04 is a positive or negative integer. */
    Tmp_CRS = ((In04 + (1U << In04_N)) * DEBUG_PWM_PERIOD_CNTS) >> (In04_N + 1U);
  }

  if (Tmp_CRS < 0)
  {
    Tmp_CRS = REVERSE_CRS_OR_0;
  }
  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_1_SLICE, Tmp_CRS);		/* Update CCU41 Shadow Compare Register. */

  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t)DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk);
#endif /* (DEBUG_PWM_1_ENABLE == 1) */

}	/* End of CCU4_Debug3Output () */

/* Use UART to set POT ADC, and hence target motor speed
   ** Execution time: ?? executed from RAM (O3 - Optimize most).
     * ----------------------------------------------------------*/
#if(UART_ENABLE != USIC_DISABLED_ALL)
void UART_Set_POT_ADC(void)
{
  /* Init motor speed adjusted by POT ADC by default.*/
  static uint16_t UART_Received_Data = 'x';



  #if(UART_ENABLE == USIC_DISABLED_ALL)
  uint16_t pot_adc_result;
  #endif

  /* EMPTY = 0, receive buffer is not empty.*/
  if ((USIC0_CHX_TRBSR & 0x00000008) == 0)
  {
    /*Data received via UART.*/
    UART_Received_Data = (USIC0_CHX_OUTR & 0x0000FFFF);

    /* Send carriage return, and new line.*/
    UART_TX_Return_NewLine(0x02);
    /* Debug information.*/
    UART_TX_String("Speed Ref (rpm)");

    if ((UART_Received_Data == 'x') || (UART_Received_Data == 'X'))
    {
      /* Motor speed adjusted by POT ADC. */
      UART_TX_String(" by POT ADC");
    }
    else
    {
      /* Use UART to adjust POT ADC, and hence target motor speed.*/
      switch (UART_Received_Data)
      {
        /* '0', or space bar (or blank or space key).*/
        case '0':
        case ' ':
          ADC.ADC_POT = FOR_MOTOR_SPEED_0;
          /* Speed 0rpm. Stop motor. */
          UART_TX_uint16_t(MOTOR_SPEED_0);
          UART_TX_String(" ->Stop");
          break;

        case '1':
          ADC.ADC_POT = FOR_MOTOR_SPEED_1;
          /* 10% of max speed. */
          UART_TX_uint16_t(MOTOR_SPEED_1);
          UART_TX_String(" ->10%");
          break;

        case '2':
          ADC.ADC_POT = FOR_MOTOR_SPEED_2;
          /*  20% of max speed.*/
          UART_TX_uint16_t(MOTOR_SPEED_2);
          UART_TX_String(" ->20%");
          break;

        case '3':
          ADC.ADC_POT = FOR_MOTOR_SPEED_3;
          /* 30% of max speed. */
          UART_TX_uint16_t(MOTOR_SPEED_3);
          UART_TX_String(" ->30%");
          break;

        case '4':
          ADC.ADC_POT = FOR_MOTOR_SPEED_4;
          /*  40% of max speed. */
          UART_TX_uint16_t(MOTOR_SPEED_4);
          UART_TX_String(" ->40%");
          break;

        case '5':
          ADC.ADC_POT = FOR_MOTOR_SPEED_5;
          /* 50% of max speed. */
          UART_TX_uint16_t(MOTOR_SPEED_5);
          UART_TX_String(" ->50%");
          break;

        case '6':
          ADC.ADC_POT = FOR_MOTOR_SPEED_6;
          /* 60% of max speed. */
          UART_TX_uint16_t(MOTOR_SPEED_6);
          UART_TX_String(" ->60%");
          break;

        case '7':
          ADC.ADC_POT = FOR_MOTOR_SPEED_7;
          /* 70% of max speed. */
          UART_TX_uint16_t(MOTOR_SPEED_7);
          UART_TX_String(" ->70%");
          break;

        case '8':
          ADC.ADC_POT = FOR_MOTOR_SPEED_8;
          /* 80% of max speed.*/
          UART_TX_uint16_t(MOTOR_SPEED_8);
          UART_TX_String(" ->80%");
          break;

        case '9':
          ADC.ADC_POT = FOR_MOTOR_SPEED_9;
          /* 90% of max speed. */
          UART_TX_uint16_t(MOTOR_SPEED_9);
          UART_TX_String(" ->90%");
          break;

        case 'a':
          ADC.ADC_POT = FOR_MOTOR_SPEED_A;
          /* 100% of max speed. */
          UART_TX_uint16_t(MOTOR_SPEED_A);
          UART_TX_String(" ->100%");
          break;

        case '=':
        case '+':
          if (ADC.ADC_POT <= (FOR_MOTOR_SPEED_A - ADC_FOR_STEP_50_RPM ))
          {
            ADC.ADC_POT += ADC_FOR_STEP_50_RPM;
            /* Speed + 50rpm */
            UART_TX_String(" +50 rpm");
          }
          else
          {
            ADC.ADC_POT = FOR_MOTOR_SPEED_A;
            /* 100% of max speed. */
            UART_TX_uint16_t(MOTOR_SPEED_A);
          }
          break;

        case '-':                 // '-'.
          if (ADC.ADC_POT >= (FOR_MOTOR_SPEED_1 + ADC_FOR_STEP_50_RPM ))
          {
            ADC.ADC_POT -= ADC_FOR_STEP_50_RPM;
            /* Speed - 50rpm. */
            UART_TX_String(" -50 rpm");
          }
          else
          {
            ADC.ADC_POT = FOR_MOTOR_SPEED_1;
            /*  10% of max speed. */
            UART_TX_uint16_t(MOTOR_SPEED_1);
          }
          break;

        default:
          /* Process for all other cases, no update of POT ADC. */
          UART_TX_String(" no change");
          break;
      }
    }
  }
  else
  {
    /* Receive buffer is empty.*/
    if ((UART_Received_Data == 'x') || (UART_Received_Data == 'X'))
    {


    }

    Motor.UART_Debug_Counter++;
    if (Motor.UART_Debug_Counter > UART_SPEED_UPDATE_RATE)
    {
      Motor.UART_Debug_Counter = 0;

      switch (Motor.State)
      {
        /* CCU8 TRAP has occurred.*/
        case TRAP_PROTECTION:
          UART_TX_String("\r\n\n\nCCU8 Trap!, key in 0");
          break;

        case BRAKE_BOOTSTRAP:
          UART_TX_String("\r\n ADC Bias:");
          /* Bias of ADC Iu.*/
          UART_TX_uint16_t(ADC.ADC_Bias_Iu);
          UART_TX_uint16_t(ADC.ADC_Bias_Iv);
          UART_TX_uint16_t(ADC.ADC_Bias_Iw);

          break;

        default:
          /* Process for all other cases. Normal operation. */
          UART_TX_String("\r\n rpm:");


          Speed_in_rpm = (Motor.Speed * SPEED_TO_RPM ) >> SCALE_SPEED_TO_RPM;
          Motor.speed_in_rpm = Speed_in_rpm;
          /* Debug information, showing rotor speed in rpm. */
          UART_TX_uint16_t(Speed_in_rpm);

          Speed_in_rpm -= ((Motor.Ref_Speed * SPEED_TO_RPM ) >> SCALE_SPEED_TO_RPM);
          UART_TX_String("\t\t Ref");
          if (Speed_in_rpm < 0)
          {
            Speed_in_rpm = -Speed_in_rpm;
            UART_TX_String(" -");
          }
          else
          {
            UART_TX_String(" +");
          }
          /* Debug information, showing speed error in rpm. */
          UART_TX_uint16_t(Speed_in_rpm);

          UART_TX_String("\t POT");
          UART_TX_uint16_t(ADC.ADC_POT);

          break;
      }
    }
  }
}

#endif
