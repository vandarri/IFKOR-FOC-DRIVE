/**
 * @file mcuinit.c
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
 * @file mcuinit.c
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
#include <XMC1300.h>							/* SFR declarations of the selected device */
#include "mcuinit.h"
#include "..\Configuration\pmsm_foc_user_mcuhwconfig.h"
#include "..\Configuration\pmsm_foc_uCProbe_parameters.h"

#define PMSM_FOC_SETTLING_TIME    0x7FFFF

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize MCU and peripherals for motor control */
void PMSM_FOC_Init(void)
{

  volatile uint32_t delay_counter;

  /* Reset configuration, clock configuration */
  Reset_Clock_Init();

  /* Hardware Settling down Timing*/
  for(delay_counter = 0; delay_counter < PMSM_FOC_SETTLING_TIME; delay_counter++);

  /* Init CCU8 */
  CCU8_Init();

  /* Init CCU4 for debug, PWM speed adjustment, or FG / RD */
  CCU4_Init();

  #if(UART_ENABLE != USIC_DISABLED_ALL)
  /* Init UART */
  UART_Init();
  #endif
  /* Init MATH Unit (i.e.: CORDIC Coprocessor and Divider Unit DIV) */
  MATH_Init();

  /* Init GPIOs */
  GPIO_Init();

  UART_TX_String("\r\nInfineon FOC\r\n");

  /*  Init variables for motor control. Before start motor, brake the motor in case it is running */
  MotorControl_Init();

  /* Init ADC, for current sensing, ADC of DC link Vdc (and POT). Do at later stage of the init */
  ADC_Init();
  ADC_DCLink_Init();
  ADC_Pot_Init();

  /* Init WDT */
  WDT_Init();

  /* Synchronous start of CAPCOM modules, e.g.: CCU8x, and or CCU4x */
  CCUx_SynStart();

}


