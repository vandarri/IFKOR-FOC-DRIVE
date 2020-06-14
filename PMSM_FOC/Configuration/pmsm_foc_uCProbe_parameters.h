/**
 * @file pmsm_foc_uCProbe_parameters.h
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
 *********************************************************************************************************************
 *
 * @file pmsm_foc_uCProbe_parameters.h
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

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_UCPROBE_PARAMETERS_H_    /* Copy and paste all to file "Parameters.h" */
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_UCPROBE_PARAMETERS_H_

/**********************************************************************************************************************
 * HEADER FILES
 *********************************************************************************************************************/

#include "pmsm_foc_user_parameter.h"

/**
 * @addtogroup
 * @{
 */

/**
 * @addtogroup
 * @{
 */

void MotorControl_Init (void);
/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
/* Timing parameters */
#define PERIOD_REG                    CCU8_PERIOD_REG
#define TZ_PZV                        SVM_TZ_PZV
#define BRAKE_TIME                    BOOTSTRAP_BRAKE_TIME
#define ALIGNMENT_TIME                PRE_ALIGNMENT_TIME

/* Scale of SVM sine Look-Up Table (LUT) */
#define SVM_LUT_SCALE                 SVM_LUTTABLE_SCALE

/* Motor parameters for ωL|I|, Vref/L in MET and PLL Observer */
#define L_OMEGALI                     DEFAULT_L_SCALEDUP
#define SCALE_L                       DEFAULT_SCALE_OF_L

/* V/f parameter */

#define VQ_VF_OFFSET                  STARTUP_VF_OFFSET
#define VQ_VF_SLEW                    STARTUP_VF_SLEWRATE
#define DEFAULT_SPEED_STARTUP         STARTUP_SPEED
#define VF_TRANSITION_SPEED           STARTUP_SPEED_THRESHOLD
#define DEFAULT_SPEED_REFERENCE       REFERENCE_SPEED_USER

#define RAMPUP_RATE                   RAMP_UP_SPEED
#define RAMPDOWN_RATE                 RAMP_DOWN_SPEED

/* Motor speed limit */
#define SPEED_LOW_LIMIT               SPEED_LOW_LIMIT_RPM
#define SPEED_HIGH_LIMIT              SPEED_HIGH_LIMIT_RPM
#define TZ_PZVX2                      (SVM_TZ_PZV<<1)
#define HALF_TZ_PZV                   (SVM_TZ_PZV>>1)

/* For SW debug */
#define CCU4_PWM_PERIOD               CCU4_PERIOD_REG

/* CCU8 dead time */
#define DEAD_TIME                     CCU8_DEAD_TIME

#define SPEEDRAMPSTEP                 SPEED_RAMP_UPDOWN_STEP

/* For MET Fine-Tuning */
#define THRESHOLD_HIGH                USER_MET_THRESHOLD_HIGH
#define THRESHOLD_LOW                 USER_MET_THRESHOLD_LOW
#define SHIFT_MET_PLL                 USER_MET_LPF

/* ADC trigger point of Pseudo Zero Vectors */
#define TRIGGER_POINT                 ADC_TRIGGER_POINT

/* SVM voltage compensation */
#define ADC_DCLINK_IDEAL              VADC_DCLINK
/* Angle/speed resolution increase, especially for low-speed motor drive. */
/* Parameters for Startup Lock / Stall Detection */
#define MAX_RETRY_START_STALL         USER_MAX_RETRY_MOTORSTARTUP_STALL

/* Increase Angle (and Speed) Resolution */
#define RES_INC                       USER_RES_INC

/* Integer speed in SW to rpm speed of real world (for debug) */
#define SPEED_TO_RPM                  CONVERT_SPEED_TO_RPM
#define SCALE_SPEED_TO_RPM            SPEED_TO_RPM_SCALE

#define RAM_ATTRIBUTE  __attribute__((section(".ram_code")))
/* Use UART to set POT ADC, and hence target motor speed. */
#define ADC_FOR_STEP_50_RPM         ADC_STEP_INC_EACHRPM



/*      --------------------------------------------------- UCProbe Update Button  ---------------------------------------- */
#define UCPROBE_BUTTON_PRESSED                      1234
#define UCPROBE_BUTTON_CLEAR                        0


#endif  /* PMSM_FOC_CONFIGURATION_PMSM_FOC_UCPROBE_PARAMETERS_H_ */

/**
 * @}
 */

/**
 * @}
 */
