/**
 * @file clock.c
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
 * @file clock.c
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

#include <XMC1300.h>              /* SFR declarations of the selected device */
#include "clock.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - Clock Configuration.
 */
XMC_SCU_CLOCK_CONFIG_t clock_config =
{
	.idiv = 0x01U,
	.pclk_src = XMC_SCU_CLOCK_PCLKSRC_DOUBLE_MCLK,
	.rtc_src = XMC_SCU_CLOCK_RTCCLKSRC_DCO2
};

/***********************************************************************************************************************
 * GLOBAL DATA
***********************************************************************************************************************/
/* Global variable, MCU Reset Status Information, reason of last reset */
uint32_t g_mcu_reset_status;    // Global variable. MCU Reset Status Information, reason of last reset.

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize clock module and read reset status */
void Reset_Clock_Init(void)
{
	uint32_t Reset_Status;

	/* Reset status, get reason of last reset */
	Reset_Status = XMC_SCU_RESET_GetDeviceResetReason();

	/* Record MCU Reset Status Information by a global variable */
	g_mcu_reset_status = Reset_Status;

	/* Clear reset status, to ensure a clear indication of cause of next reset */
	XMC_SCU_RESET_ClearDeviceResetReason();

	/* Enable reset triggered by critical events: Flash ECC error, loss of clock, 16kbytes SRAM parity error */
	XMC_SCU_RESET_EnableResetRequest((uint32_t)XMC_SCU_RESET_REQUEST_FLASH_ECC_ERROR |
	                                 (uint32_t)XMC_SCU_RESET_REQUEST_CLOCK_LOSS |
	                                 (uint32_t)XMC_SCU_RESET_REQUEST_SRAM_PARITY_ERROR);

	/* 32MHz MCLK, PCLK = 2 x MCLK = 64MHz, RTC clock is standby clock, Counter Adjustment = 1024 clock cycles */
	XMC_SCU_CLOCK_Init(&clock_config);

}
