/**
 * @file ccu4.c
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
 * @file ccu4.c
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
#include <XMC1300.h>        /* SFR declarations of the selected device */
#include "ccu4.h"
#include "..\Configuration\pmsm_foc_user_mcuhwconfig.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

#if (( DEBUG_PWM_0_ENABLE) | (DEBUG_PWM_1_ENABLE))
/**
 *  Data Structure initialization - CCU4 Slice Configuration.
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t DebugPwmSlice_Cfg =
{
  .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear = 0U,
  .dither_timer_period = 0U,
  .dither_duty_cycle = 0U,
  .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable = 0U,
  .prescaler_initval = 0U, /* divide by 1--> 64MHz (as fast as possible) */
  .float_limit = 0U,
  .dither_limit = 0U,
  .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
  .timer_concatenation = false
};
#endif /* ((DEBUG_PWM_0_ENABLE) | (DEBUG_PWM_1_ENABLE)) */

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize CCU4 module to outputs debug information. Outputs P1. 0, P0.4, P1.2, P1.3 */
void CCU4_Init(void)
{

  /* Init CCU40 */
  XMC_CCU4_Init(DEBUG_PWM_CCU4_MODULE, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
  XMC_CCU4_SetModuleClock(DEBUG_PWM_CCU4_MODULE, XMC_CCU4_CLOCK_SCU);

  #if	(DEBUG_PWM_0_ENABLE == 1U)
  /* Init Debug PWM Slice */
  /* Get the slice out of idle mode */
  XMC_CCU4_EnableClock(DEBUG_PWM_CCU4_MODULE, DEBUG_PWM_0_SLICE_NUM);
  /* Initialize the Slice */
  XMC_CCU4_SLICE_CompareInit(DEBUG_PWM_0_SLICE, &DebugPwmSlice_Cfg);

  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_0_SLICE, DEBUG_PWM_50_PERCENT_DC_CNTS);
  XMC_CCU4_SLICE_SetTimerPeriodMatch( DEBUG_PWM_0_SLICE, DEBUG_PWM_PERIOD_CNTS);
  XMC_CCU4_SLICE_SetTimerValue( DEBUG_PWM_0_SLICE, 0U);
  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t) DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk);

  /* Setup the I/O Pin */
  XMC_GPIO_SetMode(DEBUG_PWM_0_PORT, DEBUG_PWM_0_PIN, DEBUG_PWM_0_ALT_OUT);

  XMC_CCU4_SLICE_StartTimer(DEBUG_PWM_0_SLICE);
  #endif /* (DEBUG_PWM_0_ENABLE == 1) */

  #if	(DEBUG_PWM_1_ENABLE == 1U)
  /* Init Debug PWM Slice */
  /* Get the slice out of idle mode */
  XMC_CCU4_EnableClock(DEBUG_PWM_CCU4_MODULE, DEBUG_PWM_1_SLICE_NUM);
  /* Initialize the Slice */
  XMC_CCU4_SLICE_CompareInit(DEBUG_PWM_1_SLICE, &DebugPwmSlice_Cfg);

  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_1_SLICE, DEBUG_PWM_50_PERCENT_DC_CNTS);
  XMC_CCU4_SLICE_SetTimerPeriodMatch( DEBUG_PWM_1_SLICE, DEBUG_PWM_PERIOD_CNTS);
  XMC_CCU4_SLICE_SetTimerValue(DEBUG_PWM_1_SLICE, 0U);
  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t) DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk);

  /* Setup the I/O Pin */
  XMC_GPIO_SetMode(DEBUG_PWM_1_PORT, DEBUG_PWM_1_PIN, DEBUG_PWM_1_ALT_OUT);

  XMC_CCU4_SLICE_StartTimer(DEBUG_PWM_1_SLICE);
  #endif /* (DEBUG_PWM_1_ENABLE == 1) */

  XMC_CCU4_StartPrescaler(DEBUG_PWM_CCU4_MODULE);
}

