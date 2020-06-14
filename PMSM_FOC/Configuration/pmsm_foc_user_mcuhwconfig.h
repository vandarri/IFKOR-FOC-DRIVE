/**
 * @file pmsm_foc_user_mcuhwconfig.h
 * @date 2016-03-29
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
 * @file pmsm_foc_user_mcuhwconfig.h
 * @date 29 Mar, 2016
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
 * 29 Mar 2016 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_MCUHWCONFIG_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_MCUHWCONFIG_H_

/**********************************************************************************************************************
 * HEADER FILES
 *********************************************************************************************************************/
#include "pmsm_foc_uCProbe_parameters.h"
#include "pmsm_foc_user_parameter.h"
#include "xmc_gpio.h"
#include "xmc_ccu8.h"
#include "pmsm_foc_features_config.h"
#include  <xmc1_flash.h>


/**
 * @addtogroup
 * @{
 */

/**
 * @addtogroup
 * @{
 */

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/


#if (PMSM_FOC_HARDWARE_BOARD == IFX_XMC_PINUS_V2)
/*********************************************************************************************************************
 * IFX_XMC_PINUS_V2
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_2
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_V_LS_PIN         P0_3
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P1_2

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

//#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (0U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

/* TRAP LED blinking period. */
#define LED_BLINK_PRS   (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)


/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_G0_CHANNEL    (2U)       /* P2.9, VADC group0 channel 2 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_RESULT_REG    (4U)

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G0
#define VADC_VDC_GROUP_NO     (0U)
#define VADC_VDC_CHANNEL      (1U)      /* P2.8 VADC group1 channel 6 */
#define VADC_VDC_RESULT_REG   (1U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (2U)       /* No use */
#define VADC_IDC_RESULT_REG   (2U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (5U)      /* P2.3 VADC POT G1CH5 */
#define VADC_POT_RESULT_REG   (5U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_VDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

#define UART_ENABLE             USIC_DISABLED_ALL             /* 1. USIC_DISABLED_ALL
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   */

#elif (PMSM_FOC_HARDWARE_BOARD == IFX_XMC_LVPB_R3)
/*********************************************************************************************************************
 * IFX_XMC_LVPB_R3
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_2
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_V_LS_PIN         P0_3
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P1_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
//#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (0U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

/* TRAP LED blinking period. */
#define LED_BLINK_PRS   (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)


/*ADC Asynchrononus Conversion */
#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (4U)       /* P2.3, VADC group1 channel 5 */
#define VADC_IW_RESULT_REG    (4U)



/*ADC_Synchrononus Conversion */
/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_G0_CHANNEL    (2U)       /* No group 0 for this pin  */

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (4U)       /* P2.9 VADC group1 channel 4 */
#define VADC_ISS_RESULT_REG   (15U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (6U)      /* P2.4 VADC group1 channel 6 */
#define VADC_VDC_RESULT_REG   (6U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (1U)       /* P2.9 VADC group0 channel 2 */
#define VADC_IDC_RESULT_REG   (1U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

#define UART_ENABLE             USIC0_CH1_P1_2_P1_3             /* 1. USIC_DISABLED_ALL
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */

#elif (PMSM_FOC_HARDWARE_BOARD == IFX_XMC_LVPB_R2)
/*********************************************************************************************************************
 * IFX_XMC_LVPB_R2
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_2
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_V_LS_PIN         P0_3
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P1_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
//#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (1U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

/* TRAP LED blinking period. */
#define LED_BLINK_PRS   (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)


/*ADC Asynchrononus Conversion */
#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (5U)       /* P2.3, VADC group1 channel 5 */
#define VADC_IW_RESULT_REG    (5U)



/*ADC_Synchrononus Conversion */
/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (5U)       /* P2.3, VADC group1 channel 5 */
#define VADC_IW_G0_CHANNEL    (0xFU)       /* No group 0 for this pin  */

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (4U)       /* P2.9 VADC group1 channel 4 */
#define VADC_ISS_RESULT_REG   (15U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (6U)      /* P2.4 VADC group1 channel 6 */
#define VADC_VDC_RESULT_REG   (6U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (2U)       /* P2.9 VADC group0 channel 2 */
#define VADC_IDC_RESULT_REG   (2U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3             /* 1. USIC_DISABLED_ALL
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */

#elif(PMSM_FOC_HARDWARE_BOARD == KIT_XMC1X_AK_MOTOR_001)                    /* LV15W Motor Kit */
/*********************************************************************************************************************
 * KIT_XMC1X_AK_MOTOR_001
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_7
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_LS_PIN         P0_6
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P0_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

//#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (1U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif
/* TRAP LED blinking period. */
#define LED_BLINK_PRS     (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_G0_CHANNEL    (2U)       /* P2.9, VADC group0 channel 2 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_RESULT_REG    (4U)

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (1U)       /* P2.7 VADC group1 channel 1 */
#define VADC_ISS_RESULT_REG   (15U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (5U)      /* P2.3 VADC group1 channel 5 */
#define VADC_VDC_RESULT_REG   (5U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G1
#define VADC_IDC_GROUP_NO     (1U)
#define VADC_IDC_CHANNEL      (6U)       /* P2.4 VADC group1 channel 6 */
#define VADC_IDC_RESULT_REG   (6U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL


#define UART_ENABLE             USIC0_CH1_P1_2_P1_3             /* 1. USIC_DISABLED_ALL   -- which enable POT ADC speed adjustment
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 -- which disabled POT ADC speed adjustment */


#elif(PMSM_FOC_HARDWARE_BOARD == IFI_EVAL_24V_250W)
/*********************************************************************************************************************
 * IFI_EVAL_24V_250W
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_7
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_LS_PIN         P0_6
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P1_4
#define TEST_PIN1          P1_5

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

//#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (0U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

/* TRAP LED blinking period. */
#define LED_BLINK_PRS     (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_G0_CHANNEL    (2U)        /* P2.9, VADC group0 channel 2 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_RESULT_REG    (4U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_G0_CHANNEL    (4U)       /* P2.11, VADC group0 channel 4 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_RESULT_REG    (3U)

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (1U)       /* P2.7 VADC group1 channel 1 */
#define VADC_ISS_RESULT_REG   (15U)

#endif

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (6U)       /* P2.1 VADC group0 channel 6 */
#define VADC_IDC_RESULT_REG   (1U)       /* alias channel to grp 0 channel 1*/

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (5U)      /* P2.3 VADC group1 channel 5 */
#define VADC_VDC_RESULT_REG   (5U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G0_CHANNEL
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3             /* 1. USIC_DISABLED_ALL
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */

#elif(PMSM_FOC_HARDWARE_BOARD == KIT_XMC750WATT_MC_AK_V1)              /* 750W Motor Control Application Kit */
/*********************************************************************************************************************
 * KIT_XMC750WATT_MC_AK_V1
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_7
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_LS_PIN         P0_6
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P0_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
//#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (0U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
  #define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
  #define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

/* TRAP LED blinking period. */
#define LED_BLINK_PRS     (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_G0_CHANNEL    (2U)        /* P2.9, VADC group0 channel 2 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_RESULT_REG    (4U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_G0_CHANNEL    (4U)       /* P2.11, VADC group0 channel 4 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_RESULT_REG    (3U)

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (1U)       /* P2.7 VADC group1 channel 1 */
#define VADC_ISS_RESULT_REG   (15U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (5U)      /* P2.3 VADC group1 channel 5 */
#define VADC_VDC_RESULT_REG   (5U)

/* DC link average current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (6U)       /* P2.1 VADC group0 channel 6 */
#define VADC_IDC_RESULT_REG   (6U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

#define UART_ENABLE             USIC0_CH1_P1_2_P1_3              /* 1. USIC_DISABLED_ALL
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */


#elif (PMSM_FOC_HARDWARE_BOARD == DEMO_OptiMOS_3PH_INV)
/*********************************************************************************************************************
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12 //Not need to change
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_2
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_V_LS_PIN         P0_3
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P1_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

//#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (1U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

/* TRAP LED blinking period. */
#define LED_BLINK_PRS   (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)


/*ADC Asynchrononus Conversion */
#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_RESULT_REG    (4U)



/*ADC_Synchrononus Conversion */
/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (4U)       /* P2.3, VADC group1 channel 5 */
#define VADC_IW_G0_CHANNEL    (2U)       /* No group 0 for this pin  */

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (0U)       /* P2.8 VADC group1 channel 0 */
#define VADC_ISS_RESULT_REG   (0U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (6U)      /* P2.4 VADC group1 channel 6 */
#define VADC_VDC_RESULT_REG   (6U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (1U)       /* P2.8 VADC group0 channel 1 */
#define VADC_IDC_RESULT_REG   (1U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3//USIC_DISABLED_ALL
                                                                /* 1. USIC_DISABLED_ALL
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */

#elif (PMSM_FOC_HARDWARE_BOARD == DEMO_DISCRETE_CFD7_INV)
/*********************************************************************************************************************
 * IFKOR_PMM_TEST
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12 //No need to change
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_2
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_V_LS_PIN         P0_3
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P1_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

//#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (1U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

/* TRAP LED blinking period. */
#define LED_BLINK_PRS   (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)


/*ADC Asynchrononus Conversion */
#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (4U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (4U)

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (3U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_RESULT_REG    (3U)



/*ADC_Synchrononus Conversion */
/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (4U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (2U)        /* P2.11, VADC group0 channel 4 */

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (3U)       /* P2.3, VADC group1 channel 5 */
#define VADC_IW_G0_CHANNEL    (4U)       /* No group 0 for this pin  */

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (0U)       /* P2.8 VADC group1 channel 0 */
#define VADC_ISS_RESULT_REG   (0U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (6U)      /* P2.4 VADC group1 channel 6 */
#define VADC_VDC_RESULT_REG   (6U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (1U)       /* P2.8 VADC group0 channel 1 */
#define VADC_IDC_RESULT_REG   (1U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3   //USIC_DISABLED_ALL
                                                                /* 1. USIC_DISABLED_ALL
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */


#elif (PMSM_FOC_HARDWARE_BOARD == CUSTOM_BOARD)
/*********************************************************************************************************************
 * IFKOR_PMM_TEST
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#define TRAP_PIN               P0_12 //Not need to change
#define INVERTER_EN_PIN        P0_11

#define PHASE_U_HS_PIN         P0_0
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P0_1
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P0_2
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_V_LS_PIN         P0_3
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7

#define PHASE_W_HS_PIN         P0_8
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P0_9
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN           P1_4

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE          CCU80
#define CCU8_MODULE_PHASE_U  CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU80_CC82
#define CCU8_MODULE_ADC_TR   CCU80_CC83

//#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH
#define CCU8_PASSIVE_LEVEL    XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW

#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//#define CCU8_INPUT_TRAP_LEVEL  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

#define INVERTER_ENABLE_PIN    (1U)             /* 1 = Active High, 0 Active Low*/

#if(INVERTER_ENABLE_PIN == 0U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#elif(INVERTER_ENABLE_PIN == 1U)
#define ENABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_HIGH
#define DISABLE_LEVEL XMC_GPIO_OUTPUT_LEVEL_LOW
#endif

/* TRAP LED blinking period. */
#define LED_BLINK_PRS   (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)


/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/

#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)


/*ADC Asynchrononus Conversion */
#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_RESULT_REG    (4U)



/*ADC_Synchrononus Conversion */
/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (4U)       /* P2.3, VADC group1 channel 5 */
#define VADC_IW_G0_CHANNEL    (2U)       /* No group 0 for this pin  */

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (0U)       /* P2.8 VADC group1 channel 0 */
#define VADC_ISS_RESULT_REG   (0U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (6U)      /* P2.4 VADC group1 channel 6 */
#define VADC_VDC_RESULT_REG   (6U)

/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (1U)       /* P2.8 VADC group0 channel 1 */
#define VADC_IDC_RESULT_REG   (1U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3   //USIC_DISABLED_ALL
                                                                /* 1. USIC_DISABLED_ALL
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */

#endif


/* NVIC Interrupt Resources Configuration */
/* ********************************************************************************************************************/
#define Trap_Protection_INT CCU80_1_IRQHandler
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  #if (VADC_ISS_GROUP_NO == 0)
#define VADC_Source_IRQHandler VADC0_G0_1_IRQHandler
  #else
#define VADC_Source_IRQHandler VADC0_G1_1_IRQHandler
  #endif
#endif
/*********************************************************************************************************************
 * CCU4 DAC Debug
 ********************************************************************************************************************/
  #define DEBUG_PWM_CCU4_MODULE   CCU40

  #define DEBUG_PWM_0_ENABLE      (0U)        /* 1 = Enable Debug PWM P1.0, 0 = Disable Debug PWM */
  #define DEBUG_PWM_1_ENABLE      (0U)        /* 1 = Enable Debug PWM P0.4, 0 = Disable Debug PWM */


  /* Debug Period Value controls the resolution of the PWM.
   * This is the value that goes into the PWM period register.
   */
  #define DEBUG_PWM_PERIOD_CNTS (400U)

  /* Initial Duty Cycle of Debug PWM Channels */
  #define DEBUG_PWM_50_PERCENT_DC_CNTS  ((uint16_t)(DEBUG_PWM_PERIOD_CNTS >> 1))

#if (DEBUG_PWM_0_ENABLE == 1U)
  #define DEBUG_PWM_0_SLICE                         CCU40_CC40
  #define DEBUG_PWM_0_SLICE_NUM                     (0U)
  #define DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_0
  #define DEBUG_PWM_0_PORT                          XMC_GPIO_PORT1
  #define DEBUG_PWM_0_PIN                           (0U)
  #define DEBUG_PWM_0_ALT_OUT                       XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2
#endif  /*DEBUG_PWM_0_ENABLE == 1*/

#if (DEBUG_PWM_1_ENABLE == 1U)
  #define DEBUG_PWM_1_SLICE                         CCU40_CC41
  #define DEBUG_PWM_1_SLICE_NUM                     (1U)
  #define DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_1
  #define DEBUG_PWM_1_PORT                          XMC_GPIO_PORT0
  #define DEBUG_PWM_1_PIN                           (4U)
  #define DEBUG_PWM_1_ALT_OUT                       XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT4
#endif  /*DEBUG_PWM_1_ENABLE == 1 */

  /* Tmp_CRS = 0 or (- Tmp_CRS) if Tmp_CRS < 0. */
  #define REVERSE_CRS_OR_0  (- Tmp_CRS)

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_MCUHWCONFIG_H_ */

/**
 * @}
 */

/**
 * @}
 */
