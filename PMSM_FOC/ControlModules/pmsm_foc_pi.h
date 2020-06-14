/**
 * @file pmsm_foc_pi.h
 * @date 2015-12-23
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
 ***********************************************************************************************************************

 * @file pmsm_foc_pi.h
 * @date 23 Dec, 2015
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
 * 23 Dec 2015 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

#ifndef PMSM_FOC_CONTROLMODULES_PMSM_FOC_PI_H_
#define PMSM_FOC_CONTROLMODULES_PMSM_FOC_PI_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <XMC1300.h>          /* SFR declarations of the selected device */
#include "../Configuration/pmsm_foc_uCProbe_parameters.h"
#include "../Configuration/pmsm_foc_user_parameter.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*%%%%%%%% Update / Modify PI Parameters Here %%%%%%%%*/

              /* For DJI Drone (black). */
/*################### For Speed PI controller ######################*/
#if(MOTOR_TYPE == DJI_DRONE_MOTOR)
        #define USER_PI_SPEED_KP            ((uint16_t)1U<<15U)          /* (1<<15). Proportional gain Kp, uint16_t. */
        #define USER_PI_SPEED_KI            ((uint16_t)2)                /* (1<<3). Integral gain Ki, uint16_t. */
        #define USER_PI_SPEED_SCALE_KPKI    (10 + RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/

        /* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
        #define PI_SPEED_IK_LIMIT_MIN  (-(((1<<14) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
        #define PI_SPEED_IK_LIMIT_MAX  (((1<<14) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */

        #define PI_SPEED_UK_LIMIT_MIN  (16)            /* (-32767), 16. U[k] output limit LOW. */
        #define PI_SPEED_UK_LIMIT_MAX  (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

        /*################### For Torque / Iq PI controller ######################*/
        /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
        #define USER_PI_TORQUE_KP        (USER_DEFAULT_IQID_KP)              /* (1<<13). Proportional gain Kp, uint16_t. */
        #define USER_PI_TORQUE_KI        (USER_DEFAULT_IQID_KI >> 0)       /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
        #define USER_PI_TORQUE_SCALE_KPKI    (SCALING_CURRENT_KPKI + 0)

        #define PI_TORQUE_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
        #define PI_TORQUE_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

        #define PI_TORQUE_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
        #define PI_TORQUE_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

        /*################### For Flux / Id PI controller ######################*/
        /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
        #define USER_PI_FLUX_KP          (USER_DEFAULT_IQID_KP)              /* (1<<13). Proportional gain Kp, uint16_t. */
        #define USER_PI_FLUX_KI          (USER_DEFAULT_IQID_KI >> 0)       /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
        #define USER_PI_FLUX_SCALE_KPKI  (SCALING_CURRENT_KPKI + 0)

        #define PI_FLUX_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
        #define PI_FLUX_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

        #define PI_FLUX_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
        #define PI_FLUX_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

        /*################### For PLL rotor speed PI controller ######################*/
        #define USER_PI_PLL_KP         ((uint16_t)1<<5)              /* Proportional gain Kp, uint16_t. */
        #define USER_PI_PLL_KI         ((uint16_t)1<<4)              /* (1<<4). Integral gain Ki, uint16_t. */
        #define USER_PI_PLL_SCALE_KPKI     (18 - RES_INC)

        /* I[k] output limit LOW. */
        #define PI_PLL_IK_LIMIT_MIN     (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
        #define PI_PLL_IK_LIMIT_MAX     ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

        #define PI_PLL_UK_LIMIT_MIN     ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
        #define PI_PLL_UK_LIMIT_MAX     (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/


#elif(MOTOR_TYPE == MCI_DRONE_MOTOR)
/*      --------------------------------------------------- User Fine Tune Motor Speed PI and Estimator PLL PI ---------------------------------------- */

        #define USER_PI_SPEED_KP            ((uint16_t)1U<<1U)          /* (1<<15). Proportional gain Kp, uint16_t. */
        #define USER_PI_SPEED_KI            ((uint16_t)1U<<1U)                /* (1<<3). Integral gain Ki, uint16_t. */
        #define USER_PI_SPEED_SCALE_KPKI    (16 + RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/

        /* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
        #define PI_SPEED_IK_LIMIT_MIN  (-(((1<<14) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
        #define PI_SPEED_IK_LIMIT_MAX  (((1<<14) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */

        #define PI_SPEED_UK_LIMIT_MIN  (16>>1)            /* (-32767), 16. U[k] output limit LOW. */
        #define PI_SPEED_UK_LIMIT_MAX  (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

        /*################### For Torque / Iq PI controller ######################*/
        /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
        #define USER_PI_TORQUE_KP        (USER_DEFAULT_IQID_KP >> 0)             /* (1<<13). Proportional gain Kp, uint16_t. */
        #define USER_PI_TORQUE_KI        (USER_DEFAULT_IQID_KI >> 4)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
        #define USER_PI_TORQUE_SCALE_KPKI    (SCALING_CURRENT_KPKI + 2)

        #define PI_TORQUE_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
        #define PI_TORQUE_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

        #define PI_TORQUE_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
        #define PI_TORQUE_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

        /*################### For Flux / Id PI controller ######################*/
        /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
        #define USER_PI_FLUX_KP          (USER_DEFAULT_IQID_KP >> 0)             /* (1<<13). Proportional gain Kp, uint16_t. */
        #define USER_PI_FLUX_KI          (USER_DEFAULT_IQID_KI >> 4)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
        #define USER_PI_FLUX_SCALE_KPKI  (SCALING_CURRENT_KPKI + 2)

        #define PI_FLUX_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
        #define PI_FLUX_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

        #define PI_FLUX_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
        #define PI_FLUX_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

        /*################### For PLL rotor speed PI controller ######################*/
        #define USER_PI_PLL_KP             ((uint16_t)1 << 5)              /* Proportional gain Kp, uint16_t. */
        #define USER_PI_PLL_KI             ((uint16_t)1 << 4)              /* (1<<4). Integral gain Ki, uint16_t. */
        #define USER_PI_PLL_SCALE_KPKI     (18 - RES_INC)

        /* I[k] output limit LOW. */
        #define PI_PLL_IK_LIMIT_MIN     (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
        #define PI_PLL_IK_LIMIT_MAX     ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

#if(MY_FOC_CONTROL_SCHEME == CONSTANT_TORQUE_DIRECT_FOC)

        #define PI_PLL_UK_LIMIT_MIN     ((uint32_t)200 )                /* U[k] output limit LOW. */
        #define PI_PLL_UK_LIMIT_MAX     (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT )            /* U[k] output limit HIGH.*/
#elif((MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC) || (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_ONLY))
        #define PI_PLL_UK_LIMIT_MIN     ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
        #define PI_PLL_UK_LIMIT_MAX     (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/
#elif(MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC)
        #define PI_PLL_UK_LIMIT_MIN     ((uint32_t)200)                /* U[k] output limit LOW. */
        #define PI_PLL_UK_LIMIT_MAX     (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT )            /* U[k] output limit HIGH.*/
#endif


#elif (MOTOR_TYPE == VORNADO_FAN_MOTOR)
        #define USER_PI_SPEED_KP            ((uint16_t)1U << 15U)          /* (1<<15). Proportional gain Kp, uint16_t. */
        #define USER_PI_SPEED_KI            ((uint16_t)3U)                /* (1<<3). Integral gain Ki, uint16_t. */
        #define USER_PI_SPEED_SCALE_KPKI    (10 + RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/

        /* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
        //        #define PI_SPEED_IK_LIMIT_MIN  (-(((1<<14) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
        //        #define PI_SPEED_IK_LIMIT_MAX  (((1<<14) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */
        #define PI_SPEED_IK_LIMIT_MIN  (-32768)      /* (-(1<<15)). I[k] output limit LOW. */
        #define PI_SPEED_IK_LIMIT_MAX  (32768)         /* (1<<15). I[k] output limit HIGH. */

        #define PI_SPEED_UK_LIMIT_MIN  (16)            /* (-32767), 16. U[k] output limit LOW. */
        #define PI_SPEED_UK_LIMIT_MAX  (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

        /*################### For Torque / Iq PI controller ######################*/
        /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
        #define USER_PI_TORQUE_KP        (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
        #define USER_PI_TORQUE_KI        (USER_DEFAULT_IQID_KI >> 1)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
        #define USER_PI_TORQUE_SCALE_KPKI    (SCALING_CURRENT_KPKI + 0)

        #define PI_TORQUE_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
        #define PI_TORQUE_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

        #define PI_TORQUE_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
        #define PI_TORQUE_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

        /*################### For Flux / Id PI controller ######################*/
        /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
        #define USER_PI_FLUX_KP          (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
        #define USER_PI_FLUX_KI          (USER_DEFAULT_IQID_KI >> 1)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
        #define USER_PI_FLUX_SCALE_KPKI  (SCALING_CURRENT_KPKI + 0)

        #define PI_FLUX_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
        #define PI_FLUX_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

        #define PI_FLUX_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
        #define PI_FLUX_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

        /*################### For PLL rotor speed PI controller ######################*/
        #define USER_PI_PLL_KP             ((uint16_t)1<<8)              /* Proportional gain Kp, uint16_t. */
        #define USER_PI_PLL_KI             ((uint16_t)1<<6)              /* (1<<4). Integral gain Ki, uint16_t. */
        #define USER_PI_PLL_SCALE_KPKI     (20 - RES_INC)

        /* I[k] output limit LOW. */
        #define PI_PLL_IK_LIMIT_MIN     (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
        #define PI_PLL_IK_LIMIT_MAX     ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

        #define PI_PLL_UK_LIMIT_MIN     ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
        #define PI_PLL_UK_LIMIT_MAX     ((SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT) << 1)            /* U[k] output limit HIGH.*/


#elif (MOTOR_TYPE == NANOTEC_MOTOR)
        /* For Low Voltage 15W Board with Nanotec Motor */
        /*################### For Speed PI controller ######################*/
  #define USER_PI_SPEED_KP            ((uint16_t)1U<<15U)          /* (1<<15). Proportional gain Kp, uint16_t. */
  #define USER_PI_SPEED_KI            ((uint16_t)2)                /* (1<<3). Integral gain Ki, uint16_t. */
  #define USER_PI_SPEED_SCALE_KPKI    (10 + RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/

  /* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
  #define PI_SPEED_IK_LIMIT_MIN  (-(((1<<11) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
  #define PI_SPEED_IK_LIMIT_MAX  (((1<<11) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */

  #define PI_SPEED_UK_LIMIT_MIN  (16)            /* (-32767), 16. U[k] output limit LOW. */
  #define PI_SPEED_UK_LIMIT_MAX  (4000)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

  /*################### For Torque / Iq PI controller ######################*/
  /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
  #define USER_PI_TORQUE_KP        (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
  #define USER_PI_TORQUE_KI        (USER_DEFAULT_IQID_KI >> 1)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
  #define USER_PI_TORQUE_SCALE_KPKI    (SCALING_CURRENT_KPKI + 0)

  #define PI_TORQUE_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
  #define PI_TORQUE_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

  #define PI_TORQUE_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
  #define PI_TORQUE_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

  /*################### For Flux / Id PI controller ######################*/
  /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
  #define USER_PI_FLUX_KP          (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
  #define USER_PI_FLUX_KI          (USER_DEFAULT_IQID_KI >> 1)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
  #define USER_PI_FLUX_SCALE_KPKI  (SCALING_CURRENT_KPKI + 0)

  #define PI_FLUX_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
  #define PI_FLUX_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

  #define PI_FLUX_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
  #define PI_FLUX_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

  /*################### For PLL rotor speed PI controller ######################*/
  #define USER_PI_PLL_KP         ((uint16_t)(1<<6))              /* Proportional gain Kp, uint16_t. */
  #define USER_PI_PLL_KI         ((uint16_t)(1<<4))              /* (1<<4). Integral gain Ki, uint16_t. */
  #define USER_PI_PLL_SCALE_KPKI     (17 - RES_INC)

  /* I[k] output limit LOW. */
  #define PI_PLL_IK_LIMIT_MIN     (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
  #define PI_PLL_IK_LIMIT_MAX     ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

#define PI_PLL_UK_LIMIT_MIN     ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX     (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/


#elif (MOTOR_TYPE == MAXON_MOTOR)
              /* For Low Voltage 15W Board with MAXON Motor */
/*################### For Speed PI controller ######################*/
#define USER_PI_SPEED_KP            ((uint16_t)1U<<15U)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI            ((uint16_t)3)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI    (10 + RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/

/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN  (-(((1<<15) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX  (((1<<15) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */

#define PI_SPEED_UK_LIMIT_MIN  (16)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX  (32767)          /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

/*################### For Torque / Iq PI controller ######################*/
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define USER_PI_TORQUE_KP        (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_TORQUE_KI        (USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_TORQUE_SCALE_KPKI    (SCALING_CURRENT_KPKI + 0)

#define PI_TORQUE_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_TORQUE_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

/*################### For Flux / Id PI controller ######################*/
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define USER_PI_FLUX_KP          (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_FLUX_KI          (USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_FLUX_SCALE_KPKI  (SCALING_CURRENT_KPKI + 0)

#define PI_FLUX_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_FLUX_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

/*################### For PLL rotor speed PI controller ######################*/
#define USER_PI_PLL_KP         ((uint16_t)(1<<8))              /* Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI         ((uint16_t)(1<<6))              /* (1<<4). Integral gain Ki, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI     (19 - RES_INC)

/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN     (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX     ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

#define PI_PLL_UK_LIMIT_MIN     ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX     (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/


#elif (MOTOR_TYPE == BEKO_WM_MOTOR)
/*################### For Speed PI controller ######################*/
#define USER_PI_SPEED_KP            ((uint16_t)1U<<16U - 1)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI            ((uint16_t)1U << 8U)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI    (8 + RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/

/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN  (-(1<<13))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX  (1<<13)         /* (1<<15). I[k] output limit HIGH. */

#define PI_SPEED_UK_LIMIT_MIN  (4)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX  (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

/*################### For Torque / Iq PI controller ######################*/
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define USER_PI_TORQUE_KP        (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_TORQUE_KI        (USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_TORQUE_SCALE_KPKI    (SCALING_CURRENT_KPKI + 0)

#define PI_TORQUE_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_TORQUE_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

/*################### For Flux / Id PI controller ######################*/
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define USER_PI_FLUX_KP          (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_FLUX_KI          (USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_FLUX_SCALE_KPKI  (SCALING_CURRENT_KPKI + 0)

#define PI_FLUX_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_FLUX_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

/*################### For PLL rotor speed PI controller ######################*/
#define USER_PI_PLL_KP         ((uint16_t)(2000))              /* Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI         ((uint16_t)(1<<5))              /* (1<<4). Integral gain Ki, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI     (17 - RES_INC)

/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN     (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX     ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

#define PI_PLL_UK_LIMIT_MIN     ((uint32_t)SPEED_LOW_LIMIT >> 0)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX     (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/


#elif (MOTOR_TYPE == EBM_PAPST_VENTI_FAN_MOTOR)
/*################### For Speed PI controller ######################*/
#define USER_PI_SPEED_KP            ((uint16_t)(1U<<15U) - 1)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI            ((uint16_t)1U<<3)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI    (12 + RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/

/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN  (-(1<<14))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX  (1<<14)         /* (1<<15). I[k] output limit HIGH. */

#define PI_SPEED_UK_LIMIT_MIN  (4)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX  (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

/*################### For Torque / Iq PI controller ######################*/
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define USER_PI_TORQUE_KP        (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_TORQUE_KI        (USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_TORQUE_SCALE_KPKI    (SCALING_CURRENT_KPKI + 0)

#define PI_TORQUE_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_TORQUE_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

/*################### For Flux / Id PI controller ######################*/
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define USER_PI_FLUX_KP          (USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_FLUX_KI          (USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_FLUX_SCALE_KPKI  (SCALING_CURRENT_KPKI + 0)

#define PI_FLUX_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_FLUX_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

/*################### For PLL rotor speed PI controller ######################*/
#define USER_PI_PLL_KP         ((uint16_t)(1<<8))              /* Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI         ((uint16_t)(1<<4))              /* (1<<4). Integral gain Ki, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI     (20 - RES_INC)

/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN     (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX     ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

#define PI_PLL_UK_LIMIT_MIN     ((uint32_t)SPEED_LOW_LIMIT >> 1)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX     (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/


#elif (MOTOR_TYPE == LG_CordZero)
    #define USER_PI_SPEED_KP            ((uint16_t)1U << 15U)          /* (1<<15). Proportional gain Kp, uint16_t. */
    #define USER_PI_SPEED_KI            ((uint16_t)3U)                /* (1<<3). Integral gain Ki, uint16_t. */
    #define USER_PI_SPEED_SCALE_KPKI    (10U + RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/

        /* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
//  #define PI_SPEED_IK_LIMIT_MIN  (-(((1<<14) * 3) >> 2))      /* (-(1<<15)). I[k] output limit LOW. */
//  #define PI_SPEED_IK_LIMIT_MAX  (((1<<14) * 3) >> 2)         /* (1<<15). I[k] output limit HIGH. */
    #define PI_SPEED_IK_LIMIT_MIN  (-32768)      /* (-(1<<15)). I[k] output limit LOW. */
    #define PI_SPEED_IK_LIMIT_MAX  (32768)         /* (1<<15). I[k] output limit HIGH. */


    #define PI_SPEED_UK_LIMIT_MIN  (16)            /* (-32767), 16. U[k] output limit LOW. */
    #define PI_SPEED_UK_LIMIT_MAX  (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

    /*################### For Torque / Iq PI controller ######################*/
    /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */

    #define USER_PI_TORQUE_KP        (62397U)//(USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
    #define USER_PI_TORQUE_KI        (1078U)//(USER_DEFAULT_IQID_KI >> 1)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
    #define USER_PI_TORQUE_SCALE_KPKI    (15U)//(SCALING_CURRENT_KPKI + 2)

    #define PI_TORQUE_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
    #define PI_TORQUE_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

    #define PI_TORQUE_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
    #define PI_TORQUE_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

    /*################### For Flux / Id PI controller ######################*/
    /* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */

    #define USER_PI_FLUX_KP          (62397U)//(USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
    #define USER_PI_FLUX_KI          (1078U)//(USER_DEFAULT_IQID_KI >> 1)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
    #define USER_PI_FLUX_SCALE_KPKI  (15U)//(SCALING_CURRENT_KPKI + 2)

    #define PI_FLUX_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
    #define PI_FLUX_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

    #define PI_FLUX_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
    #define PI_FLUX_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

    /*################### For PLL rotor speed PI controller ######################*/

    #define USER_PI_PLL_KP             (1260U)//((uint16_t)1<<10)              /* Proportional gain Kp, uint16_t. */
    #define USER_PI_PLL_KI             (25U)//((uint16_t)1<<5)              /* (1<<4). Integral gain Ki, uint16_t. */
    #define USER_PI_PLL_SCALE_KPKI     (15U)//(20 - (RES_INC+3))

    /* I[k] output limit LOW. */
    #define PI_PLL_IK_LIMIT_MIN     (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
    #define PI_PLL_IK_LIMIT_MAX     ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

    #define PI_PLL_UK_LIMIT_MIN     ((uint32_t)SPEED_LOW_LIMIT >> 4)                /* U[k] output limit LOW. */
    #define PI_PLL_UK_LIMIT_MAX     ((SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT) << 1)            /* U[k] output limit HIGH.*/


#elif (MOTOR_TYPE == HV_Drive)
/*################### For Speed PI controller ######################*/
#define USER_PI_SPEED_KP            ((uint16_t)(1U<<15U) - 1)          /* (1<<15). Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KI            ((uint16_t)1U<<3)                /* (1<<3). Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKI    (12 + RES_INC)               /* RES_INC: Angle/speed resolution increase from 16 bit.*/

/* Note: (IK_LIMIT_MIN << SCALE_KPKI) and (IK_LIMIT_MAX << SCALE_KPKI) are maximum int32_t. Same as below. */
#define PI_SPEED_IK_LIMIT_MIN  (-(1<<14))      /* (-(1<<15)). I[k] output limit LOW. */
#define PI_SPEED_IK_LIMIT_MAX  (1<<14)         /* (1<<15). I[k] output limit HIGH. */

#define PI_SPEED_UK_LIMIT_MIN  (4)            /* (-32767), 16. U[k] output limit LOW. */
#define PI_SPEED_UK_LIMIT_MAX  (32767)         /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */

/*################### For Torque / Iq PI controller ######################*/
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define USER_PI_TORQUE_KP        (32113U)//(USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_TORQUE_KI        (23929U)//(USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_TORQUE_SCALE_KPKI    (17U)//(SCALING_CURRENT_KPKI + 0)

#define PI_TORQUE_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_TORQUE_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_TORQUE_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

/*################### For Flux / Id PI controller ######################*/
/* Kp and Ki (from excel file) calculated from motor parameter L and R. Normally no need change. */
#define USER_PI_FLUX_KP          (32113U)//(USER_DEFAULT_IQID_KP)             /* (1<<13). Proportional gain Kp, uint16_t. */
#define USER_PI_FLUX_KI          (23929U)//(USER_DEFAULT_IQID_KI >> 0)        /* (1<<6). Integral gain Ki, Ki/Kp = RxTs/L. uint16_t. */
#define USER_PI_FLUX_SCALE_KPKI  (17U)//(SCALING_CURRENT_KPKI + 0)

#define PI_FLUX_IK_LIMIT_MIN    (-32768)      /* (-(1<<15)). I[k] output limit LOW. Normally no need change. */
#define PI_FLUX_IK_LIMIT_MAX    (32767)       /* (1<<15). I[k] output limit HIGH. Normally no need change. */

#define PI_FLUX_UK_LIMIT_MIN    (-32768)      /* U[k] output limit LOW. Normally no need change. */
#define PI_FLUX_UK_LIMIT_MAX    32767         /* U[k] output limit HIGH. Normally no need change. */

/*################### For PLL rotor speed PI controller ######################*/
#define USER_PI_PLL_KP         (456U)//((uint16_t)(1<<8))              /* Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI         (26U)//((uint16_t)(1<<4))              /* (1<<4). Integral gain Ki, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI     (16U)//(20 - RES_INC)

/* I[k] output limit LOW. */
#define PI_PLL_IK_LIMIT_MIN     (-(int32_t)((uint32_t)1 << (uint32_t)(30U-(uint32_t)PI_PLL_SCALE_KPKI)))
#define PI_PLL_IK_LIMIT_MAX     ((uint32_t)1 << (30U-(uint32_t)PI_PLL_SCALE_KPKI))     /* I[k] output limit HIGH. */

#define PI_PLL_UK_LIMIT_MIN     ((uint32_t)SPEED_LOW_LIMIT >> 1)                /* U[k] output limit LOW. */
#define PI_PLL_UK_LIMIT_MAX     (SPEED_HIGH_LIMIT + SPEED_LOW_LIMIT)            /* U[k] output limit HIGH.*/


#endif

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef struct PI_Coefs_Type
{
  int32_t error;										/* PI error signal (reference value � feedback value), error[k] */

  int32_t Uk;											  /* PI output U[k] */
  int32_t Ik;											  /* Integral result I[k] */

  uint16_t Kp;										  /* Proportional gain Kp */
  uint16_t Ki;										  /* Integral gain Ki */
  int16_t Scale_KpKi;								/* Scale-up Kp and Ki by 2^Scale_KpKi */

  int32_t Ik_limit_min;
  int32_t Ik_limit_max;

  int32_t Uk_limit_min;
  int32_t Uk_limit_max;
  uint8_t Uk_limit_status;
} PI_Coefs_Type;



/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @brief Fixed point implementation for filter saturation logic.
 * @param input_val    Value that need to be limited
 * @param higher_limit Maximum value for <i>input_val</i>
 * @param lower_limit  Minimum value for <i>input_val</i>
 * @return int32_t
 * <i>input_val</i>, if <i>lower_limit</i> < <i>input_val</i> < <i>higher_limit</i>
 * <i>higher_limit</i>, if <i>input_val</i> > <i>higher_limit</i>
 * <i>lower_limit</i>, if <i>input_val</i> < <i>lower_limit</i>
 *
 * \par<b>Description: </b><br>
 * This function is used by PI Controller to limit the <i>input_val</i> within its minimum and maximum range.
 *
 * @endcode
 */
__STATIC_INLINE RAM_ATTRIBUTE int32_t MIN_MAX_LIMIT(int32_t input_val,int32_t higher_limit,int32_t lower_limit);

/**
 * @brief PI controller
 *      U(t)=Kp x e(t) + (Ki/Ts) x ∫e(t)dt, where Ts is sampling period, e.g.: Ts = 50us.
 *      I[k] = I[k-1] + Ki * error[k]
 *      U[k] = Kp * error[k] + I[k]
 *
 * @param *PI_data
 *      error
 *
 *@retval *PI_data
 */
__STATIC_INLINE void PI_controller(int32_t reference, int32_t feedback, PI_Coefs_Type *PI_data);

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
__STATIC_INLINE RAM_ATTRIBUTE int32_t MIN_MAX_LIMIT(int32_t input_val,int32_t higher_limit,int32_t lower_limit)
{
  int32_t return_val;
  if ( input_val > higher_limit )
  {
    return_val = higher_limit;
  }
  else if ( input_val < lower_limit )
  {
    return_val = lower_limit;
  }
  else
  {
    return_val = input_val;
  }
  return return_val;
}

__STATIC_INLINE RAM_ATTRIBUTE void PI_controller(int32_t reference, int32_t feedback, PI_Coefs_Type *PI_data)
{
  int32_t Tmp_Ik_Uk;

  PI_data->error = reference - feedback;

  /* Integral output I[k] = I[k-1] + Ki * error[k] */
  Tmp_Ik_Uk = ((int32_t)PI_data->Ki * PI_data->error) + PI_data->Ik;

  /* Check I[k] limit */
  PI_data->Ik = MIN_MAX_LIMIT(Tmp_Ik_Uk, PI_data->Ik_limit_max, PI_data->Ik_limit_min);

  /* PI output U[k] = Kp * error[k] + I[k] */
  Tmp_Ik_Uk = ((int32_t)PI_data->Kp * PI_data->error) + PI_data->Ik;

  /* Check U[k] output limit */
  PI_data->Uk = MIN_MAX_LIMIT((Tmp_Ik_Uk >> PI_data->Scale_KpKi), PI_data->Uk_limit_max, PI_data->Uk_limit_min);

}

void PI_controller_Init(void);
__STATIC_INLINE RAM_ATTRIBUTE void PI_controller_anti_windup(int32_t reference, int32_t feedback,
                                                             PI_Coefs_Type *PI_data);

__STATIC_INLINE RAM_ATTRIBUTE void PI_controller_anti_windup(int32_t reference, int32_t feedback,
                                                             PI_Coefs_Type *PI_data)
{
  static int32_t Tmp_Ik_Uk;

  PI_data->error = reference - feedback;

  if(PI_data->Uk_limit_status == 0)
  {
    /* Integral output I[k] = I[k-1] + Ki * error[k] */
    Tmp_Ik_Uk = ((int32_t)PI_data->Ki * PI_data->error) + PI_data->Ik;
    PI_data->Ik = MIN_MAX_LIMIT(Tmp_Ik_Uk, PI_data->Ik_limit_max, PI_data->Ik_limit_min);
  }

  /* PI output U[k] = Kp * error[k] + I[k] */
  Tmp_Ik_Uk = ((int32_t)PI_data->Kp * PI_data->error) + PI_data->Ik;
  Tmp_Ik_Uk = Tmp_Ik_Uk >> PI_data->Scale_KpKi;
  /* Check U[k] output limit */
  PI_data->Uk = MIN_MAX_LIMIT(Tmp_Ik_Uk, PI_data->Uk_limit_max, PI_data->Uk_limit_min);
  if(PI_data->Uk != Tmp_Ik_Uk)
  {
    PI_data->Uk_limit_status = 1;
  }
  else
  {
    PI_data->Uk_limit_status = 0;
  }

}

#endif /* PMSM_FOC_CONTROLMODULES_PMSM_FOC_PI_H_ */

/**
 * @}
 */

/**
 * @}
 */
