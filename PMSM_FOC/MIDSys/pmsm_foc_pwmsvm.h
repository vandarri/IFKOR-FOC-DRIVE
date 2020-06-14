/**
 * @file pmsm_foc_pwmsvm.h
 * @date 2015-12-15
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
 * @file pmsm_foc_pwmsvm.h
 * @date 15 Dec, 2015
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
 * 15 Dec 2015 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

#ifndef PMSM_FOC_MIDSYS_PMSM_FOC_PWMSVM_H_
#define PMSM_FOC_MIDSYS_PMSM_FOC_PWMSVM_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../MCUInit/ccu8.h"

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
#define SVM_USE_PZV       (0)        /* To indicate using SVM with Pseudo Zero Vectors (PZV) */
#define SVM_USE_STANDARD  (0xAD)     /* To indicate using standard SVM (3 or 4-segment) */

#define ANGLETEMP_SHIFT   (8U)       /* For calculation of Angle_Temp */
#define SECTOR_ANGLE_AND  (0x00FFU)  /* For calculation of Sector_Angle */
#define SECTOR_NO_SHIFT   (8U)       /* For calculation of sector number */
#define MAX_LUT_INDEX     (255U)     /* Maximum angle index in LUT */

/*
 * T0 threshold time, to tell if PWM T0 has enough time for three valid ADC samplings, for 3-shunt phase current
 * sensing
 */
#define T0_THRESHOLD      (USER_PCLK_FREQ_MHz * 14) //((uint32_t)TZ_PZV * 6U)

/* To use all (e.g.: three) ADC samplings for current reconstruction, for 3-shunt phase current sensing */
#define USE_ALL_ADC       (0U)
#define USE_2_ADC         (0xBBU)     /* To use two of three ADC samplings for current reconstruction */

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef struct SVMType
{
  uint16_t CurrentSectorNo; /*
                             * Current new sector number: 0 ~ 5 (represent Sector A ~ F) in SVM space vector
                             * hexagon
                             */
  uint16_t PreviousSectorNo; /* To record the sector number of last PWM cycle, for 3-phase current reconstruction */

  uint16_t Flag_3or2_ADC; /*
                           * Flag to indicate using three or two ADC samplings for current reconstruction,
                           * for 3-shunt phase current sensing
                           */

  uint16_t SVM_Flag; /* Flag to indicate using SVM with Pseudo Zero Vectors (PZV), or standard SVM */
} SVMType;



/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/

/**
 * @param Amplitude   \n
 * @param Angle Rotor Angle \n
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * To update SVPWM CCU8 duty cycles, use standard SVM (7-segment symmetric PWM) <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PWMSVM01_Update(uint16_t Amplitude, uint16_t Angle);

#endif /* PMSM_FOC_MIDSYS_PMSM_FOC_PWMSVM_H_ */

/**
 * @}
 */

/**
 * @}
 */
