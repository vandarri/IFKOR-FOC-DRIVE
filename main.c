/*******************************************************************************
 Copyright (c) 2015-2016, Infineon Technologies AG                            **
 All rights reserved.                                                         **
                                                                              **
 Redistribution and use in source and binary forms, with or without           **
 modification,are permitted provided that the following conditions are met:   **
                                                                              **
 *Redistributions of source code must retain the above copyright notice,      **
 this list of conditions and the following disclaimer.                        **
 *Redistributions in binary form must reproduce the above copyright notice,   **
 this list of conditions and the following disclaimer in the documentation    **
 and/or other materials provided with the distribution.                       **
 *Neither the name of the copyright holders nor the names of its contributors **
 may be used to endorse or promote products derived from this software without**
 specific prior written permission.                                           **
                                                                              **
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  **
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE    **
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE   **
 ARE  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE   **
 LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         **
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF         **
 SUBSTITUTE GOODS OR  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    **
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN      **
 CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)       **
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE   **
 POSSIBILITY OF SUCH DAMAGE.                                                  **
                                                                              **
 To improve the quality of the software, users are encouraged to share        **
 modifications, enhancements or bug fixes with Infineon Technologies AG       **
 dave@infineon.com).                                                          **
                                                                              **
********************************************************************************
**                                                                            **
**                                                                            **
** PLATFORM : Infineon XMC1300 Series                                         **
**                                                                            **
** AUTHOR : Motor Control  R&D Team                                           **
**                                                                            **
** version 1.0.0  Initial version                                             **
**                                                                            **
** MODIFICATION DATE : Sep 30, 2016                                           **
**                                                                            **
*******************************************************************************/

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/

/* SFR declarations of the selected device */
#include <XMC1300.h>
#include "xmc_gpio.h"

#include "pmsm_foc_user_mcuhwconfig.h"
#include "pmsm_foc_features_config.h"
#include "..\ControlModules\pmsm_foc_interface.h"
#include "..\MCUInit\uart.h"
#include "..\ControlModules\pmsm_foc_pi.h"
#include "xmc1_flash.h"
/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

#if(uCPROBE_GUI == ENABLED)
/* uC Probe Variables */
uint32_t User_Para[40];
uint32_t Update_KpKi;
#endif

extern void PMSM_FOC_Init (void);


int main(void)
{

#if(uCPROBE_GUI == ENABLED)
  uint32_t * MotorCONF_Addr = MotorConfig_Addr;

  if ((*MotorCONF_Addr)!= PARAM_HEADER)
  {
    uint32_t *MotorCONF_Address = MotorConfig_Addr;
    uint32_t *UserConfig_Address ;

    User_Para[0] = PARAM_HEADER;
    User_Para[1] = USER_PI_SPEED_KP;
    User_Para[2] = USER_PI_SPEED_KI;
    User_Para[3] = USER_PI_SPEED_SCALE_KPKI;
    User_Para[4] = USER_PI_TORQUE_KP;
    User_Para[5] = USER_PI_TORQUE_KI;
    User_Para[6] = USER_PI_TORQUE_SCALE_KPKI;
    User_Para[7] = USER_PI_FLUX_KP;
    User_Para[8] = USER_PI_FLUX_KI;
    User_Para[9] = USER_PI_FLUX_SCALE_KPKI;
    User_Para[10] = USER_PI_PLL_KP;
    User_Para[11] = USER_PI_PLL_KI;
    User_Para[12] = USER_PI_PLL_SCALE_KPKI;
    User_Para[13] = USER_RES_INC;

    UserConfig_Address = &User_Para[0];
    XMC_FLASH_ProgramVerifyPage(MotorCONF_Address,UserConfig_Address);
    NVIC_SystemReset();
  }
  else
  {
    User_Para[0] = *MotorCONF_Addr;
    User_Para[1] = *(MotorCONF_Addr+1);
    User_Para[2] = *(MotorCONF_Addr+2);
    User_Para[3] = *(MotorCONF_Addr+3);
    User_Para[4] = *(MotorCONF_Addr+4);
    User_Para[5] = *(MotorCONF_Addr+5);
    User_Para[6] = *(MotorCONF_Addr+6);
    User_Para[7] = *(MotorCONF_Addr+7);
    User_Para[8] = *(MotorCONF_Addr+8);
    User_Para[9] = *(MotorCONF_Addr+9);
    User_Para[10] = *(MotorCONF_Addr+10);
    User_Para[11] = *(MotorCONF_Addr+11);
    User_Para[12] = *(MotorCONF_Addr+12);
    User_Para[12] = *(MotorCONF_Addr+12);
    User_Para[13] = *(MotorCONF_Addr+13);
  }
#endif


	PMSM_FOC_Init ();


	while (1)
	 /* MCU main loop. Actually only require the processor to run when an interrupt occurs. */
	{



	}

	 return 0;
}
/* End of main () */
