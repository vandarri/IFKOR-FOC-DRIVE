/**
 * @file ccu8.h
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
 * @file ccu8.h
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

#ifndef PMSM_FOC_MCUINIT_CCU8_H_
#define PMSM_FOC_MCUINIT_CCU8_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <xmc_ccu8.h>
#include <xmc_scu.h>
#include "..\Configuration\pmsm_foc_user_mcuhwconfig.h"

/**
 * @addtogroup
 * @{
 */

/**
 * @addtogroup
 * @{
 */
/*********************************************************************************************************************
 * MACRO
 ********************************************************************************************************************/
#define XMC_CCU8_GIDLC_CLOCK_MASK (15U)
#define XMC_CCU8_TC_TRAPSE_MASK   (15U)
/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the CCU8 module for 3 phase pwm generation to turn the motor. <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void CCU8_Init(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * CCU8 TRAP functionality enabled so PWM outputs react on state of an input pin. For over-current protection
 * Init one CCU8 output to turn on LED, indicating of CCU8 TRAP <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void Init_CCU8x_for_TRAP_LED_Indicator (void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Synchronous start of CAPCOM modules <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void CCUx_SynStart(void);

#endif /* PMSM_FOC_MCUINIT_CCU8_H_ */

/**
 * @}
 */

/**
 * @}
 */
