/**
 * @file pmsm_foc_user_parameter.h
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
 * @file pmsm_foc_user_parameter.h
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

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_PARAMETER_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_PARAMETER_H_

/**********************************************************************************************************************
 * HEADER FILES
 *********************************************************************************************************************/
#include<system_XMC1300.h>
#include "pmsm_foc_features_config.h"
#include "math.h"

extern uint32_t User_Para[40];

/**
 * @addtogroup
 * @{
 */

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define  PMSM_FOC_HARDWARE_BOARD                    DEMO_DISCRETE_CFD7_INV                 /*1. KIT_XMC1X_AK_MOTOR_001
                                                                                             2. KIT_XMC750WATT_MC_AK_V1
                                                                                             3. IFX_XMC_LVPB_R2
                                                                                             4. IFI_EVAL_24V_250W
                                                                                             5. IFX_XMC_LVPB_R3
                                                                                             6. IFX_XMC_PINUS_V2
                                                                                             7. DEMO_OptiMOS_3PH_INV
                                                                                             8. DEMO_DISCRETE_CFD7_INV
                                                                                             9. CUSTOM_BOARD*/

/*      --------------------------------------------------- Motor Type Selection ---------------------------------------- */
#define  MOTOR_TYPE                                 HV_Drive                               /*1. MCI_DRONE_MOTOR
                                                                                             2. DJI_DRONE_MOTOR
                                                                                             3. VORNADO_FAN_MOTOR
                                                                                             4. NANOTEC_MOTOR
                                                                                             5. MAXON_MOTOR
                                                                                             6. BEKO_WM_MOTOR
                                                                                             7. EBM_PAPST_VENTI_FAN_MOTOR
                                                                                             8. LG_CordZero
                                                                                             9. HV_Drive*/
/*      --------------------------------------------------- Current feedback Sensing Mechanism ---------------------------------------- */
#define  CURRENT_SENSING                            USER_THREE_SHUNT_SYNC_CONV                  /*1. USER_SINGLE_SHUNT_CONV
                                                                                              2. USER_THREE_SHUNT_ASSYNC_CONV
                                                                                              3. USER_THREE_SHUNT_SYNC_CONV*/
/*      --------------------------------------------------- FOC Control and Startup Scheme (Only Select 1 Scheme at one time) ---------------------------------------- */
#define MY_FOC_CONTROL_SCHEME                       CONSTANT_TORQUE_DIRECT_FOC            /* 1. CONSTANT_SPEED_VF_ONLY,
                                                                                             2. CONSTANT_SPEED_VF_MET_FOC
                                                                                             3. CONSTANT_SPEED_DIRECT_FOC
                                                                                             4. CONSTANT_TORQUE_DIRECT_FOC
                                                                                             5. CONSTANT_VQ_DIRECT_FOC */
/*      --------------------------------------------------- Space Vector PWM Switching Scheme (Only Select 1 Scheme at one time) ---------------------------------------- */
#define SVM_SWITCHING_SCHEME                        STANDARD_SVM_5_SEGMENT                /* 1. STANDARD_SVM_5_SEGMENT
                                                                                             2. STANDARD_SVM_7_SEGMENT */
/*      --------------------------------------------------- uCPROBE_GUI ---------------------------------------- */
#define uCPROBE_GUI                                 ENABLED                               /*1. ENABLED       2. DISABLED*/
/*      --------------------------------------------------- FOC Control Safety Protection ---------------------------------------- */
#define VDC_UNDER_OVERVOLTAGE_PROTECTION            ENABLED                              /*1. ENABLED       2. DISABLED*/
#define OVERCURRENT_PROTECTION                      ENABLED                               /*1. ENABLED       2. DISABLED*/

#if(MOTOR_TYPE == MCI_DRONE_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (0.1f)          /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (60.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (6.0f)          /* Motor Pole Pairs (change to integer) */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (16000U)          /* Max Speed of User Motor*/
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (3000U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (3000U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop
                                                                       * threshold Speed to transit from Open loop to
                                                                       * closed loop
                                                                       */
#define USER_STARTUP_VF_OFFSET_V                    (0.3f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.05f)           /* V/F start up slew rate in V/Hz */

#elif(MOTOR_TYPE == DJI_DRONE_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (0.1f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (20.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (7.0f)          /* Motor Pole Pairs (change to integer) */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (10000U)          /* Max Speed of User Motor*/
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (1000U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (1000U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (800U)            /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (0.3f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.05f)           /* V/F start up slew rate in V/Hz */
#elif(MOTOR_TYPE == VORNADO_FAN_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (0.55f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (1440.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (2200U)      /* Max Speed of User Motor*/
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (500U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (500U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (0.4f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.07f)           /* V/F start up slew rate in V/Hz */
#elif(MOTOR_TYPE == NANOTEC_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (1.0f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (1050.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (4000U)
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (1000U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (1000U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (0.3f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.05f)           /* V/F start up slew rate in V/Hz */

#elif (MOTOR_TYPE == MAXON_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (6.8f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (3865.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (4530.0f)
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (500U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (500U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (1.0f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.1f)           /* V/F start up slew rate in V/Hz */

#elif (MOTOR_TYPE == BEKO_WM_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (2.5f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (14000.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (4800.0f)
#define USER_SPEED_LOW_LIMIT_RPM                    (50.0f)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (500U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (400U)

/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (100U)            /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (5.0f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.75f)           /* V/F start up slew rate in V/Hz */

#elif (MOTOR_TYPE == EBM_PAPST_VENTI_FAN_MOTOR)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (9.8f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (96000.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (3.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (1200U)      /* Max Speed of User Motor*/
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 20U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (1000U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (1000U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (6.0f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (3.0f)           /* V/F start up slew rate in V/Hz */

#elif(MOTOR_TYPE == LG_CordZero)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (1.24f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (37.22f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (1.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
//#define  CONSTANT_VQ_START                           ENABLED           /* ENABLED: Enable constant Vq control startup for washing machine, DISABLED: disable*/
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (100000U)
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (5000U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (5000U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (0.7f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.05f)           /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- Flux Weakening (FW) Parameters ---------------------------------------- */
//#define USER_FW_ENABLE                              ENABLED          /* ENABLED or DISABLED. To enable or disable FW */

#elif(MOTOR_TYPE == HV_Drive)
/*      --------------------------------------------------- Motor Parameters ---------------------------------------- */
#define  USER_MOTOR_R_PER_PHASE_OHM                 (225.0f)        /* Motor Resistance per phase in Ohm*/
#define  USER_MOTOR_L_PER_PHASE_uH                  (1765.0f)         /* Motor Inductance per phase in uH */
#define  USER_MOTOR_POLE_PAIR                       (6.0f)          /* Motor Pole Pairs */
/*      --------------------------------------------------- Constant Speed Control Mode (Used when Constant Speed Control is enabled) ---------------------------------------- */
//#define  CONSTANT_VQ_START                           ENABLED           /* ENABLED: Enable constant Vq control startup for washing machine, DISABLED: disable*/
/*      --------------------------------------------------- POT ADC, or PWM to Adjust Speed ---------------------------------------- */
#define USER_SPEED_HIGH_LIMIT_RPM                   (5000U)
#define USER_SPEED_LOW_LIMIT_RPM                    (uint32_t) (USER_SPEED_HIGH_LIMIT_RPM / 30U)
#define USER_SPEED_RAMPUP_RPM_PER_S                 (500U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S               (500U)
/*      --------------------------------------------------- V/F Start Up Parameters ---------------------------------------- */
#define USER_STARTUP_SPEED_RPM                      (0U)
#define USER_STARTUP_SPEED_THRESHOLD_RPM            (200U)            /* threshold Speed to transit from Open loop to closed loop */
#define USER_STARTUP_VF_OFFSET_V                    (0.7f)            /* V/F startup offset in V */
#define USER_STARTUP_VF_SLEWRATE_V_PER_HZ           (0.05f)           /* V/F start up slew rate in V/Hz */

/*      --------------------------------------------------- Flux Weakening (FW) Parameters ---------------------------------------- */
//#define USER_FW_ENABLE                              ENABLED          /* ENABLED or DISABLED. To enable or disable FW */

#endif

/*      --------------------------------------------------- Hardware Inverter Parameters ---------------------------------------- */
#if(PMSM_FOC_HARDWARE_BOARD == IFX_XMC_PINUS_V2)
#define INTERNAL_OP_GAIN                            ENABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (12.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.75f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (25000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (20U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(0.33f/(0.33f+2.32f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(0.33f/(0.33f+2.32f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (1.2f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.05f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.030f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (5.1f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (39.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (10.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (75.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (6U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif


#elif(PMSM_FOC_HARDWARE_BOARD == IFX_XMC_LVPB_R3)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (25.5f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.75f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (30000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (50U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(3.0f/(3.0f+33.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(4.7f/(3.0f+33.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (1.2f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.025f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.030f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (5.1f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (39.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (10.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (75.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif

/*      --------------------------------------------------- Hardware Inverter Parameters ---------------------------------------- */
#elif(PMSM_FOC_HARDWARE_BOARD == IFX_XMC_LVPB_R2)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (13.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.75f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (25000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (20U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(3.0f/(3.0f+33.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(3.0f/(3.0f+33.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (1.2f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.0125f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.030f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (5.1f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (39.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (10.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (75.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif

#elif(PMSM_FOC_HARDWARE_BOARD == KIT_XMC1X_AK_MOTOR_001)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (24.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.75f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (20000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (20U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(5.1f/(5.1f+47.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(5.2f/(5.2f+47.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (3.0f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.05f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.05f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (1.0f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (16.4f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (10.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (75.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif
/*      --------------------------------------------------- Hardware Inverter Parameters ---------------------------------------- */
#elif(PMSM_FOC_HARDWARE_BOARD == IFI_EVAL_24V_250W)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (24.0f)                 /* Hardware Inverter VDC link voltage in V  */
//#define USER_VDC_LINK_V                             (15.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.5f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (20000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (100U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(5.6f/(5.6f+56.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(5.6f/(5.6f+56.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (1.2f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.01f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.010f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (1.0f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (12.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (1.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (12.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif

#elif (PMSM_FOC_HARDWARE_BOARD == KIT_XMC750WATT_MC_AK_V1)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (313.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.75f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (15000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (80U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(10.0f/(10.0f+990.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(12.0f/(12.0f+990.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (9.0f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.05f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.05f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (1.0f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (12.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (10.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (75.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define USER_IQ_CURRENT_ALLOWED_A                   (5.2f)
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */
#define USER_IDC_MAXCURRENT_A                       (4.3f)

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif

#elif(PMSM_FOC_HARDWARE_BOARD == DEMO_OptiMOS_3PH_INV)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (25.5f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.8f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (15000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (150U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(12.0f/(12.0f+68.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(4.7f/(4.7f+33.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
//#define USER_CURRENT_TRIP_THRESHOLD_A               (14.5f)               /* threshold current for trip detection in Ampere*/
//#define USER_TRIP_THRESHOLD_TIME_MS                 (10U)               /* threshold time for trip detection in ms */
//#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.005f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.005f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (3.0f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (43.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (3.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (43.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define USER_IQ_CURRENT_ALLOWED_A                   (5.5f)                    /* 0 < USER_IQ_CURRENT_ALLOWED_A < I_MAX_A*/
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)//25
#define IQ_MAX_LIMIT                                (int32_t)((USER_IQ_CURRENT_ALLOWED_A * (1<<15)/ I_MAX_A))
#define USER_IDC_MAXCURRENT_A                       (19.3f)

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif

#elif(PMSM_FOC_HARDWARE_BOARD == DEMO_DISCRETE_CFD7_INV)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (320.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.85f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (15000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (20U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(12.0f/(12.0f+1080.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
//#define USER_VBEMF_RATIO                            (float)(12.0f/(12.0f+1080.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
//#define USER_CURRENT_TRIP_THRESHOLD_A               (3.0f)               /* threshold current for trip detection in Ampere*/
//#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
//#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.01f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.01f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (16.0f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (820.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (16.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (820.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define USER_IQ_CURRENT_ALLOWED_A                   (7.0f)                    /* 0 < USER_IQ_CURRENT_ALLOWED_A < I_MAX_A*/
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */
#define USER_IDC_MAXCURRENT_A                       (3.0f)

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif

#elif (PMSM_FOC_HARDWARE_BOARD == CUSTOM_BOARD)
#define INTERNAL_OP_GAIN                            DISABLED               /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#define USER_VDC_LINK_V                             (320.0f)                 /* Hardware Inverter VDC link voltage in V  */
#define USER_DEAD_TIME_US                           (0.75f)                 /* deadtime, rise(left) and fall values in us  */
#define USER_CCU8_PWM_FREQ_HZ                       (15000U)              /* CCU8 PWM Switching Frequency in Hz*/
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (80U)                 /* Initial Bootstrap precharging time in ms */
#define USER_DC_LINK_DIVIDER_RATIO                  (float)(10.0f/(10.0f+990.0f))           /* R1/(R2+R1) ratio for DC link MCU ADC */
#define USER_VBEMF_RATIO                            (float)(12.0f/(12.0f+990.0f))           /* R1/(R2+R1) ratio for BEMF Voltage sensing circuit ratio */
#define USER_CURRENT_TRIP_THRESHOLD_A               (9.0f)               /* threshold current for trip detection in Ampere*/
#define USER_TRIP_THRESHOLD_TIME_MS                 (100U)               /* threshold time for trip detection in ms */
#define USER_MAX_RETRY_MOTORSTARTUP_TRIP            (3U)                /* Max retry of motor startup if trip  */
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
#define USER_R_SHUNT_OHM                            (0.05f)               /* Phase shunt resistor in ohm */
#define USER_DC_SHUNT_OHM                           (0.05f)               /* DC link shunt current resistor in ohm */
#define USER_RIN_PHASECURRENT_KOHM                  (1.0f)                 /* R_IN (of equivalent amplifier) kohm */
#define USER_R_PHASECURRENT_FEEDBACK_KOHM           (12.0f)                  /* R_FEEDBACK (of equivalent amplifier) kohm */
#define USER_RIN_DCCURRENT_KOHM                     (10.0f)                  /* Rf for dc current sensing */
#define USER_R_DCCURRENT_FEEDBACK_KOHM              (75.0f)                   /* Rin for dc current sensing */
#define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
#define USER_IQ_CURRENT_ALLOWED_A                   (8.0f)
#define G_OPAMP_PER_PHASECURRENT                    (USER_R_PHASECURRENT_FEEDBACK_KOHM / USER_RIN_PHASECURRENT_KOHM)
#define I_MAX_A                                     ((VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2U)               /* For IFX_XMC_LVPB_R3, I_MAX_A = 13.16A */
#define USER_IDC_MAXCURRENT_A                       (4.3f)

#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR                             (3U)                       /* Different HW Board has different OP Gain factor, XMC13 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR                              G_OPAMP_PER_PHASECURRENT
#endif
#endif
/*      --------------------------------------------------- Constant Torque Control Mode (Used when Constant Torque Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- IQ_REF-limit low < IQ_REF < IQ_REF-limit high  ---------------------------------------- */
#if((MY_FOC_CONTROL_SCHEME == CONSTANT_TORQUE_DIRECT_FOC) || (MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC))
//#define USER_IQ_CURRENT_ALLOWED_A                   (4.0f)                                                                          /* 0 < USER_IQ_CURRENT_ALLOWED_A < I_MAX_A*/
#define USER_IQ_REF_LOW_LIMIT                       (0U)
#define USER_IQ_REF_HIGH_LIMIT                      (uint32_t) (32768 * USER_IQ_CURRENT_ALLOWED_A /I_MAX_A)                          /*  I_MAX_A = (VAREF_V/(USER_R_SHUNT_OHM * OP_GAIN_FACTOR)) / 2.0f), IFX_XMC_LVPB_R3 - 13.6A */
#define USER_IQ_RAMPUP                              (1U)
#define USER_IQ_RAMPDOWN                            (1U)
#define USER_IQ_RAMP_SLEWRATE                       (1U)                                                                           /* USER_IQ_RAMP_SLEWRATE x PWM period, every cycle increase USER_IQ_RAMPUP or USER_IQ_RAMPDOWN */
#endif
/*      --------------------------------------------------- Constant VQ Control Mode (Used when Constant VQ Control is enabled) ---------------------------------------- */
/*      --------------------------------------------------- VQ_REF-limit low < VQ_REF < VQ_REF-limit high  ---------------------------------------- */
#if(MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC)
#define USER_VQ_VOLTAGE_ALLOWED_V                   (8U)                                                                            /* 0 < USER_VQ_VOLTAGE_ALLOWED_V < VREF_MAX_V          VREF_MAX_V =  (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT)*/
#define USER_VQ_REF_LOW_LIMIT                       (0U)
#define USER_VQ_REF_HIGH_LIMIT                      (32768)  //(uint32_t)(32768U * USER_VQ_VOLTAGE_ALLOWED_V/VREF_MAX_V)
#define USER_VQ_RAMPUP                              (1U)
#define USER_VQ_RAMPDOWN                            (1U)
#define USER_VQ_RAMP_SLEWRATE                       (1U)         /* USER_VQ_RAMP_SLEWRATE x PWM period, every cycle increase USER_VQ_RAMPUP or USER_VQ_RAMPDOWN */
#endif







#if(uCPROBE_GUI == DISABLED)
#define PI_SPEED_KP                                 USER_PI_SPEED_KP
#define PI_SPEED_KI                                 USER_PI_SPEED_KI                              /* (1<<3). Integral gain Ki, uint16_t. */
#define PI_SPEED_SCALE_KPKI                         USER_PI_SPEED_SCALE_KPKI                      /* RES_INC: Angle/speed resolution increase from 16 bit.*/
#define PI_TORQUE_KP                                USER_PI_TORQUE_KP
#define PI_TORQUE_KI                                USER_PI_TORQUE_KI
#define PI_TORQUE_SCALE_KPKI                        USER_PI_TORQUE_SCALE_KPKI
#define PI_FLUX_KP                                  USER_PI_FLUX_KP
#define PI_FLUX_KI                                  USER_PI_FLUX_KI
#define PI_FLUX_SCALE_KPKI                          USER_PI_FLUX_SCALE_KPKI
#define PI_PLL_KP                                   USER_PI_PLL_KP
#define PI_PLL_KI                                   USER_PI_PLL_KI
#define PI_PLL_SCALE_KPKI                           USER_PI_PLL_SCALE_KPKI
#else
#define PARAM_HEADER                                (0x1010 + CURRENT_SENSING)                    /* header for the Flash array */
#define MotorConfig_Addr                            (uint32_t *)0x10006800   /* Flash location for Configuration data storage*/
#define PI_SPEED_KP                                 User_Para[1]
#define PI_SPEED_KI                                 User_Para[2]
#define PI_SPEED_SCALE_KPKI                         User_Para[3]
#define PI_TORQUE_KP                                User_Para[4]
#define PI_TORQUE_KI                                User_Para[5]
#define PI_TORQUE_SCALE_KPKI                        User_Para[6]
#define PI_FLUX_KP                                  User_Para[7]
#define PI_FLUX_KI                                  User_Para[8]
#define PI_FLUX_SCALE_KPKI                          User_Para[9]
#define PI_PLL_KP                                   User_Para[10]
#define PI_PLL_KI                                   User_Para[11]
#define PI_PLL_SCALE_KPKI                           User_Para[12]
#endif




#define USER_ROTOR_PREPOSITION_TIME_MS              (100U)            /* Rotor startup pre alignment time in miliseconds */
#define USER_REFERENCE_SPEED_RPM                    (500U)
#define USER_SPEED_THRESHOLD_FW_RPM_PER_S           (100000U)        /* Threshold speed to use Flux Weakening */
/*      --------------------------------------------------- MET Fine-tuning ---------------------------------------- */
#define USER_MET_THRESHOLD_HIGH                     (64U)
#define USER_MET_THRESHOLD_LOW                      (16U)
#define USER_MET_LPF                                (2U)
/*      --------------------------------------------------- pmsm_foc_parameter.h ---------------------------------------- */
/* ********************************************* Scaling SVM Modulator ***********************************************************/
#define PWM_PERIOD_TS_US                            (1.0f/(USER_CCU8_PWM_FREQ_HZ)*1000000)
#define SVM_LAMDA                                   (1.0f/USER_INVERSE_SVM_LAMDA)
#define SVM_TZ_PZV                                  (uint32_t)((CCU8_PERIOD_REG * SVM_LAMDA) + 0.5f)
#define uTZ_LAMDA_TS_US                             (SVM_LAMDA * PWM_PERIOD_TS_US)
/* ********************************************* Scaling SVM Modulator ***********************************************************/
#define KS_SCALE_SVM                                (CCU8_PERIOD_REG / MAX_VREF_AMPLITUDE)
/* ********************************************* ADC Range  ***********************************************************/
#define VAREF_V                                     (5.0f)
#define VADC_DCLINK                                 (uint32_t)(((USER_VDC_LINK_V * USER_DC_LINK_DIVIDER_RATIO)/VAREF_V) * (1<<12))
#define VDC_MAX_LIMIT                               ((uint16_t)((VADC_DCLINK * 19U)>>4U))                                         /* Vdc_ideal + 18.7%, DC link voltage Vdc maximum limit, only for braking usage, voltage clamping */

#if(OVERCURRENT_PROTECTION == ENABLED)
//#define USER_IDC_MAXCURRENT_A                       (4.3f)
#define G_OPAMP_DC_CURRENT                          (USER_R_DCCURRENT_FEEDBACK_KOHM/ USER_RIN_DCCURRENT_KOHM)
#define IDC_MAX_LIMIT                               (uint32_t)((USER_IDC_MAXCURRENT_A * (1<<12)) / ((VAREF_V/(USER_DC_SHUNT_OHM*G_OPAMP_DC_CURRENT))/2.0f))          /* IDC Max limit to USER defined IDC Max Current */
#endif

#if(VDC_UNDER_OVERVOLTAGE_PROTECTION == ENABLED)
#define VDC_OVER_LIMIT                              ((uint16_t)(VADC_DCLINK * 120 / 100))                      /* VADC_DCLINK + 20%, DC link voltage Vdc maximum limit */
#define VDC_MIN_LIMIT                               ((uint16_t)(VADC_DCLINK * 80 / 100))                     /* VADC_DCLINK - 20%, DC link voltage Vdc min limit */
#endif
/* ********************************************* Normalization (u,v,w) represented by +2^15 ***********************************************************/
#define N_I_UVW_A                                   I_MAX_A
#define N_VREF_SVM_V                                VREF_MAX_V
/* ********************************************* Normalization (alpha,beta) represented by +2^15 ***********************************************************/
#define N_I_ALPHABETA_A                            I_MAX_A
#define N_VREF_ALPHABETA_V                         VREF_MAX_V
/* ********************************************* Normalization (d,q) represented by +2^15 ***********************************************************/
#define N_I_DQ_A                                    I_MAX_A
#define N_V_DQ_V                                    VREF_MAX_V
/* ********************************************* Motor Control Timing   ***********************************************************/
#define BOOTSTRAP_BRAKE_TIME                       ((USER_BOOTSTRAP_PRECHARGE_TIME_MS * 1000) / PWM_PERIOD_TS_US)
#define PRE_ALIGNMENT_TIME                          (uint32_t)(((USER_ROTOR_PREPOSITION_TIME_MS * 1000) / PWM_PERIOD_TS_US))
/* ********************************************* Amplitude Limit ***********************************************************/
#define MAX_VREF_AMPLITUDE                          (32768 - 0.5f)
#define VREF_MAX_V                                  (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT)
/* ********************************************* V/F Startup Parameter  ***********************************************************/
#define STARTUP_CURRENT_A                          (uint32_t)((USER_STARTUP_VF_OFFSET_V / USER_MOTOR_R_PER_PHASE_OHM))
#define STARTUP_SPEED                              (uint32_t)(((USER_STARTUP_SPEED_RPM * USER_MOTOR_POLE_PAIR)/(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define STARTUP_SPEED_THRESHOLD                    (uint32_t)(((USER_STARTUP_SPEED_THRESHOLD_RPM * USER_MOTOR_POLE_PAIR)/(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define STARTUP_VF_OFFSET                          (uint32_t)((USER_STARTUP_VF_OFFSET_V * 32768) / (N_VREF_SVM_V))
#define STARTUP_VF_SLEWRATE                        (uint32_t)(((USER_STARTUP_VF_SLEWRATE_V_PER_HZ * 32768) / (N_VREF_SVM_V) * (USER_MOTOR_POLE_PAIR)) / \
                                                     (USER_CCU8_PWM_FREQ_HZ*60)*65536)
/* ********************************************* POT ADC, or PWM to Adjust Speed  ***********************************************************/
#define SPEED_LOW_LIMIT_RPM                        (uint32_t)(((USER_SPEED_LOW_LIMIT_RPM * USER_MOTOR_POLE_PAIR) /(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define REFERENCE_SPEED_USER                       (uint32_t)(((USER_REFERENCE_SPEED_RPM * USER_MOTOR_POLE_PAIR) /(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define SPEED_HIGH_LIMIT_RPM                       (uint32_t)(((USER_SPEED_HIGH_LIMIT_RPM * USER_MOTOR_POLE_PAIR) /(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC))
#define RAMP_UP_SPEED                              (uint32_t)(USER_CCU8_PWM_FREQ_HZ / (((USER_SPEED_RAMPUP_RPM_PER_S * USER_MOTOR_POLE_PAIR)/(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC)))
#define RAMP_DOWN_SPEED                            (uint32_t)(USER_CCU8_PWM_FREQ_HZ / \
                                                      (((float)(USER_SPEED_RAMPDOWN_RPM_PER_S * USER_MOTOR_POLE_PAIR)/(USER_CCU8_PWM_FREQ_HZ*60))*65536 * (1<< USER_RES_INC)))
#define SPEED_RAMP_UPDOWN_STEP                      (1U)
#define ELECTRICAL_SPEED_FREQ_HZ                    ((float)USER_SPEED_HIGH_LIMIT_RPM/(60/USER_MOTOR_POLE_PAIR) )
/* ********************************************* PI Controller Parameters (pmsm_foc_pi.h, if used) ***********************************************************/
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
#define USER_CURRENTCTRL_CUTOFF_FREQ_HZ             (ELECTRICAL_SPEED_FREQ_HZ * 1.0f)
#else
#define USER_CURRENTCTRL_CUTOFF_FREQ_HZ             (ELECTRICAL_SPEED_FREQ_HZ * 3)
#endif
#define SCALING_CURRENT_KPKI                        (uint32_t)(log2((1<<15)/((USER_CURRENTCTRL_CUTOFF_FREQ_HZ/ELECTRICAL_SPEED_FREQ_HZ)*(SPEED_HIGH_LIMIT_RPM/(1<<USER_RES_INC))*DEFAULT_L_SCALEDUP /(1<<(uint32_t)DEFAULT_SCALE_OF_L))))
#define USER_DEFAULT_IQID_KP                        (uint32_t)((USER_CURRENTCTRL_CUTOFF_FREQ_HZ/ELECTRICAL_SPEED_FREQ_HZ)*(SPEED_HIGH_LIMIT_RPM/(1<<USER_RES_INC))*DEFAULT_L_SCALEDUP /(1<<(uint32_t)(DEFAULT_SCALE_OF_L- SCALING_CURRENT_KPKI)))
#define USER_DEFAULT_IQID_KI                        (uint32_t)(USER_DEFAULT_IQID_KP * USER_MOTOR_R_PER_PHASE_OHM * \
                                                       PWM_PERIOD_TS_US / USER_MOTOR_L_PER_PHASE_uH)
#define SCALEUP_MPS_K                               (uint16_t)(1<<8)
#define CORDIC_MPS_PER_K                            (USER_CORDIC_MPS / CORDIC_K) * (1 << SCALEUP_MPS_K)
/* ********************************************* Motor Parameter Scaling Conversion ***********************************************************/
#define DEFAULT_L_NO_SCALEUP                       (float)((3.0f/2.0f)*(2.0f*USER_PI*USER_CCU8_PWM_FREQ_HZ) * \
                                                     (N_I_ALPHABETA_A/N_VREF_ALPHABETA_V) *(USER_MOTOR_L_PER_PHASE_uH/1000000)/(1<<16))
#define DEFAULT_L_SCALEDUP                         (uint32_t)((DEFAULT_L_NO_SCALEUP) * (1<<(uint32_t)DEFAULT_SCALE_OF_L))
#define DEFAULT_SCALE_OF_L                         (uint32_t)(log2((((1<<16) - 1)/(SPEED_HIGH_LIMIT_RPM/(1<< USER_RES_INC)))/(DEFAULT_L_NO_SCALEUP)))
/* ********************************************* MCU Parameters  ***********************************************************/
#define CCU8_PERIOD_REG                             ((uint32_t)(USER_PCLK_FREQ_MHz*1000000)/USER_CCU8_PWM_FREQ_HZ)
#define CCU4_PERIOD_REG                             ((uint32_t)(USER_PCLK_FREQ_MHz*1000)/USER_CCU4_DEBUG_KHZ)
#define CCU8_DEADTIME_RISE                          (uint32_t)((USER_DEAD_TIME_US*USER_PCLK_FREQ_MHz) - 0.5)
#define CCU8_DEADTIME_FALL                          (uint32_t)((USER_DEAD_TIME_US*USER_PCLK_FREQ_MHz) - 0.5)
#define CCU8_DEAD_TIME                              (uint32_t)(CCU8_DEADTIME_RISE + 256 * CCU8_DEADTIME_FALL)
#define ADC_TRIGGER_POINT                           (uint32_t)(SVM_TZ_PZV * 0.85f)
#define SVM_LUTTABLE_SCALE                          (uint32_t)((CCU8_PERIOD_REG / MAX_VREF_AMPLITUDE) * 32768)
/* ********************************************* MISC Integer Speed in SW to RPM speed of real world  ***********************************************************/
#define CONVERT_SPEED_TO_RPM                        (uint32_t)((USER_SPEED_HIGH_LIMIT_RPM * (1<<SPEED_TO_RPM_SCALE)) / \
                                                       SPEED_HIGH_LIMIT_RPM)
#if((MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_ONLY)||(MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC) || (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC))
#define MOTOR_SPEED_0                               (0U)
#define MOTOR_SPEED_1                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.1f)
#define MOTOR_SPEED_2                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.2f)
#define MOTOR_SPEED_3                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.3f)
#define MOTOR_SPEED_4                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.4f)
#define MOTOR_SPEED_5                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.5f)
#define MOTOR_SPEED_6                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.6f)
#define MOTOR_SPEED_7                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.7f)
#define MOTOR_SPEED_8                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.8f)
#define MOTOR_SPEED_9                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 0.9f)
#define MOTOR_SPEED_A                               (uint32_t)(USER_SPEED_HIGH_LIMIT_RPM * 1.0f)

#define FOR_MOTOR_SPEED_0                           (0U)
#define FOR_MOTOR_SPEED_1                           (uint32_t)(((MOTOR_SPEED_1 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_2                           (uint32_t)(((MOTOR_SPEED_2 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_3                           (uint32_t)(((MOTOR_SPEED_3 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_4                           (uint32_t)(((MOTOR_SPEED_4 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_5                           (uint32_t)(((MOTOR_SPEED_5 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_6                           (uint32_t)(((MOTOR_SPEED_6 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_7                           (uint32_t)(((MOTOR_SPEED_7 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_8                           (uint32_t)(((MOTOR_SPEED_8 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_9                           (uint32_t)(((MOTOR_SPEED_9 - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))
#define FOR_MOTOR_SPEED_A                           (uint32_t)(((MOTOR_SPEED_A - USER_SPEED_LOW_LIMIT_RPM) * (1<<12)) / \
                                                      (USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM))

#define ADC_STEP_INC_EACHRPM                        (uint32_t)(50/(USER_SPEED_HIGH_LIMIT_RPM - USER_SPEED_LOW_LIMIT_RPM) * (1<<12))

#elif(MY_FOC_CONTROL_SCHEME == CONSTANT_TORQUE_DIRECT_FOC)
#define MOTOR_SPEED_0                               (0U)
#define MOTOR_SPEED_1                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.1f)
#define MOTOR_SPEED_2                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.2f)
#define MOTOR_SPEED_3                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.3f)
#define MOTOR_SPEED_4                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.4f)
#define MOTOR_SPEED_5                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.5f)
#define MOTOR_SPEED_6                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.6f)
#define MOTOR_SPEED_7                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.7f)
#define MOTOR_SPEED_8                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.8f)
#define MOTOR_SPEED_9                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 0.9f)
#define MOTOR_SPEED_A                               (uint32_t)(USER_IQ_REF_HIGH_LIMIT * 1.0f)
#define FOR_MOTOR_SPEED_0                           (0U)
#define FOR_MOTOR_SPEED_1                           (uint32_t)(((MOTOR_SPEED_1 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_2                           (uint32_t)(((MOTOR_SPEED_2 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_3                           (uint32_t)(((MOTOR_SPEED_3 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_4                           (uint32_t)(((MOTOR_SPEED_4 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_5                           (uint32_t)(((MOTOR_SPEED_5 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_6                           (uint32_t)(((MOTOR_SPEED_6 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_7                           (uint32_t)(((MOTOR_SPEED_7 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_8                           (uint32_t)(((MOTOR_SPEED_8 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_9                           (uint32_t)(((MOTOR_SPEED_9 - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_A                           (uint32_t)(((MOTOR_SPEED_A - USER_IQ_REF_LOW_LIMIT) * (1<<12))/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT))

#define ADC_STEP_INC_EACHRPM                        (uint32_t)(50/(USER_IQ_REF_HIGH_LIMIT - USER_IQ_REF_LOW_LIMIT) * (1<<12))
#elif(MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC)
#define MOTOR_SPEED_0                               (0U)
#define MOTOR_SPEED_1                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.1f)
#define MOTOR_SPEED_2                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.2f)
#define MOTOR_SPEED_3                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.3f)
#define MOTOR_SPEED_4                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.4f)
#define MOTOR_SPEED_5                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.5f)
#define MOTOR_SPEED_6                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.6f)
#define MOTOR_SPEED_7                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.7f)
#define MOTOR_SPEED_8                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.8f)
#define MOTOR_SPEED_9                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 0.9f)
#define MOTOR_SPEED_A                               (uint32_t)(USER_VQ_REF_HIGH_LIMIT * 1.0f)
#define FOR_MOTOR_SPEED_0                           (0U)
#define FOR_MOTOR_SPEED_1                           (uint32_t)(((MOTOR_SPEED_1 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_2                           (uint32_t)(((MOTOR_SPEED_2 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_3                           (uint32_t)(((MOTOR_SPEED_3 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_4                           (uint32_t)(((MOTOR_SPEED_4 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_5                           (uint32_t)(((MOTOR_SPEED_5 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_6                           (uint32_t)(((MOTOR_SPEED_6 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_7                           (uint32_t)(((MOTOR_SPEED_7 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_8                           (uint32_t)(((MOTOR_SPEED_8 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_9                           (uint32_t)(((MOTOR_SPEED_9 - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define FOR_MOTOR_SPEED_A                           (uint32_t)(((MOTOR_SPEED_A - USER_VQ_REF_LOW_LIMIT) * (1<<12))/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT))
#define ADC_STEP_INC_EACHRPM                        (uint32_t)(50/(USER_VQ_REF_HIGH_LIMIT - USER_VQ_REF_LOW_LIMIT) * (1<<12))
#endif




#if (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_ONLY)
  #define VF_ONLY_CCU80_0_IRQHandler           CCU80_0_IRQHandler
#elif (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_VF_MET_FOC)
  #define VF_FOC_CCU80_0_IRQHandler            CCU80_0_IRQHandler
#elif (MY_FOC_CONTROL_SCHEME == CONSTANT_SPEED_DIRECT_FOC)
  #define DirectFOCStartUp_CCU80_0_IRQHandler   CCU80_0_IRQHandler
#elif  (MY_FOC_CONTROL_SCHEME == CONSTANT_TORQUE_DIRECT_FOC)
 #define DirectFOCIQStartUp_CCU80_0_IRQHandler   CCU80_0_IRQHandler
#elif  (MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC)
 #define DirectFOCVQStartUp_CCU80_0_IRQHandler   CCU80_0_IRQHandler
#endif

#endif /* PMSM_FOC_CONFIGURATION_PMSM_FOC_USER_PARAMETER_H_ */
/**
 * @}
 */

/**
 * @}
 */
