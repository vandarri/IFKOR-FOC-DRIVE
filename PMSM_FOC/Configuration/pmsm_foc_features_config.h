/**
 * @file pmsm_foc_features_config.h
 * @date 2016-06-01
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
 * @file pmsm_foc_features_config.h
 * @date 01 Jun, 2016
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
 * 01 Jun 2016 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_FEATURES_CONFIG_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_FEATURES_CONFIG_H_

/**
 * @addtogroup
 * @{
 */

/**
 * @addtogroup
 * @{
 */

#include "pmsm_foc_user_parameter.h"

/*******************************************************************************
 * MACROS
 *******************************************************************************/

/* Motor application kit */
#define   KIT_XMC1X_AK_MOTOR_001                    (1U)      /*!< Low voltage 15W kit KIT_XMC1X_AK_MOTOR_001 */
#define   KIT_XMC750WATT_MC_AK_V1                   (2U)      /*!< High voltage 750W kit KIT_XMC750WATT_MC_AK_V1 */
#define   IFX_XMC_LVPB_R2                           (3U)      /*!< High voltage 300W kit IFX_XMC_LVPB_R2 */
#define   IFI_EVAL_24V_250W                         (4U)      /*!< Low Italy voltage 300W kit IFI_EVAL_24V_250W */
#define   IFX_XMC_LVPB_R3                           (5U)
#define   IFX_XMC_PINUS_V2                          (6U)      /* IFX PINUS HW Board V2.0 FOC*/
#define   DEMO_OptiMOS_3PH_INV                      (7U)
#define   DEMO_DISCRETE_CFD7_INV                    (8U)
#define   CUSTOM_BOARD                              (9U)

#define  MCI_DRONE_MOTOR                            (1U)
#define  DJI_DRONE_MOTOR                            (2U)
#define  VORNADO_FAN_MOTOR                          (3U)
#define  NANOTEC_MOTOR                              (4U)
#define  MAXON_MOTOR                                (5U)
#define  BEKO_WM_MOTOR                              (6U)
#define  EBM_PAPST_VENTI_FAN_MOTOR                  (7U)
#define  LG_CordZero                                (8U)
#define  HV_Drive                                   (9U)

/* Current Sensing Feedback Scheme*/
#define USER_THREE_SHUNT_ASSYNC_CONV                (1U)
#define USER_THREE_SHUNT_SYNC_CONV                  (2U)
#define USER_SINGLE_SHUNT_CONV                      (3U)
/*      --------------------------------------------------- FOC Control and Startup Scheme  ---------------------------------------- */
#define  CONSTANT_SPEED_VF_ONLY                     (1U)
#define  CONSTANT_SPEED_VF_MET_FOC                  (2U)
#define  CONSTANT_SPEED_DIRECT_FOC                  (3U)
#define  CONSTANT_TORQUE_DIRECT_FOC                 (4U)
#define  CONSTANT_VQ_DIRECT_FOC                     (5U)
/*      --------------------------------------------------- Space Vector PWM Switching Scheme (Only Select 1 Scheme at one time) ---------------------------------------- */
#define  STANDARD_SVM_5_SEGMENT                     (1U)
#define  STANDARD_SVM_7_SEGMENT                     (2U)
/*      -----------------------------Over current / Over Voltage Protection Scheme  -------------------------------- */
#define ENABLED                                      (1U)
#define DISABLED                                     (0U)
/*      -----------------------------UART USIC Channel Configuration  -------------------------------- */
#define USIC_DISABLED_ALL       (0U)
#define USIC0_CH0_P1_4_P1_5     (1U)
#define USIC0_CH1_P1_2_P1_3     (2U)

/*      --------------------------------------------------- SVM with Pseudo Zero Vectors ---------------------------------------- */
#define USER_INVERSE_SVM_LAMDA                      (float)(20.0)
/*      --------------------------------------------------- MCU Parameters ---------------------------------------- */
#define USER_MCLK_FREQ_MHz                          (32U)       /* CPU Clock in Mhz*/
#define USER_PCLK_FREQ_MHz                          (64U)       /* Peripheral CLK frequency = double of CPU Clock */
#define USER_CCU8_PRESCALER                         (1U)
#define USER_CORDIC_MPS                             (2.0f)      /* CORDIC module MPS Setting value */
#define CORDIC_K                                    (1.646760258f)                  /* CORDIC SCALE (Inherent Gain Factor) */
#define CORDIC_SHIFT                                (14U)             /* 8 ~ 16. Shift for CORDIC input / output registers, whose [7:0] are 0x00. Normally no need change.*/
/*      --------------------------------------------------- Parameters for Startup Lock / Stall Detection ---------------------------------------- */
#define USER_MAX_RETRY_MOTORSTARTUP_STALL           (3U)           /* Max retry of motor startup if stall  */
/*      --------------------------------------------------- Increase Angle (and Speed Resolution)  ---------------------------------------- */
#define USER_RES_INC                                (3U)
/*      --------------------------------------------------- MISC Constant ---------------------------------------- */
#define USER_SQRT_3_CONSTANT                        (1.7320508075f)
#define USER_CCU4_DEBUG_KHZ                         (160U)
#define USER_PI                                     (3.1415926536f)
#define SPEED_TO_RPM_SCALE                          (11U)




#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_FEATURES_CONFIG_H_ */


/**
 * @}
 */

/**
 * @}
 */
