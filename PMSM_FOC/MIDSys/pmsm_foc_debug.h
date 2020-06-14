/**
 * @file pmsm_foc_debug.h
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
 *
 * @file pmsm_foc_debug.h
 * @date 18 Jan, 2016
 * @version 1.0.0
 *
 * @brief For SW debug <br>
 *
 * <b>Detailed description of file</b> <br>
 * CCU4 PWM can be used to output variable values.  The PWM can be filtered so the variable
 * looks like an analog signal (similar to a DAC).
 *
 * The high level enabling of PWM debug channels is done in this file.
 *
 * CCU4 Module, Slice and Pin selection is done in CCU4.h
 *
 * History
 *
 * 18 Jan 2016 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

#ifndef PMSM_FOC_MIDSYS_PMSM_FOC_DEBUG_H_
#define PMSM_FOC_MIDSYS_PMSM_FOC_DEBUG_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "..\MCUInit\ccu4.h"
#include "..\MCUInit\adc.h"
#include "..\ControlModules\pmsm_foc_functions.h"
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
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param In04      variable to be debug on P0.4     \n
 * @param In04_Flag 0: 0 < In04 < 2^In04_N           \n
 *                  1: -2^In04_N < In04 < 2^In04_N   \n
 * @param In04_N    resolution of the debug variable \n
 * @param In10      variable to be debug on P1.0     \n
 * @param In10_Flag 0: 0 < In10 < 2^In10_N           \n
 *                  1: -2^In10_N < In04 < 2^In10_N   \n
 * @param In10_N    resolution of the debug variable \n
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * To use CCU4 Debug with 2 Outputs, P0.4 and P1.0 <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void CCU4_Debug3Output(int32_t In04, uint16_t In04_Flag, uint16_t In04_N, int32_t In10, uint16_t In10_Flag,
                       uint16_t In10_N);

#endif /* PMSM_FOC_MIDSYS_PMSM_FOC_DEBUG_H_ */

/**
 * @}
 */

/**
 * @}
 */
