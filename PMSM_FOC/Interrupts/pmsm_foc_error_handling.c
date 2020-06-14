/**
 * @file pmsm_foc_error_handling.c
 * @date 2016-03-14
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
 * @file pmsm_foc_error_handling.c
 * @date 14 Mar, 2016
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
 * 14 Mar 2016 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include <XMC1300.h>                      /* SFR declarations of the selected device */
#include "pmsm_foc_error_handling.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/

extern MotorControlType Motor; /* Motor control information */
extern ADCType ADC;

#define MAX_CYCLE_TIME    7500                /* Max cycle times before system reset. */

/*
 * Error handling, e.g.: to handle CCU8 TRAP protection
 * Execution time: ??us (O3 - Optimize most).
 */
void Error_Handling(void)
{
  static uint32_t Cycle_Counter = 0; /* Static variable and init. */

  if ((Motor.State == TRAP_PROTECTION) && (SYSTEM_BE_IDLE))
  {
    /* If CCU8 TRAP has occurred, and system becomes idle (i.e.: PWM duty cycle or POT ADC too low), */
    Cycle_Counter++;

    if (Cycle_Counter > MAX_CYCLE_TIME)
    {

			  XMC_CCU8_SLICE_ClearEvent(CCU8_MODULE_PHASE_U,XMC_CCU8_SLICE_IRQ_ID_EVENT2);

        XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, ENABLE_LEVEL); /* Enable gate driver. */
        Motor.State = STOP_MOTOR;

    }
  }
  else
  {
    Cycle_Counter = 0; /* Clear counter. */
  }

} /* End of Error_Handling () */
