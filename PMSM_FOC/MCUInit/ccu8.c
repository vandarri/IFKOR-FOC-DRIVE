/**
 * @file ccu8.c
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
 * @file ccu8.c
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
#include <XMC1300.h>							/* SFR declarations of the selected device */
#include "CCU8.h"
#include "../Configuration/pmsm_foc_uCProbe_parameters.h"
#include "../Configuration/pmsm_foc_user_mcuhwconfig.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - CC8 Slices configuration for all three pwm phase.
 */
XMC_CCU8_SLICE_COMPARE_CONFIG_t CCU8_PhaseInit =
{
  .timer_mode           = (uint8_t)XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot             = 0U,
  .shadow_xfer_clear    = 1U,     /* CLST = 1b, Enable a shadow transfer whenever a clearing action by SW or external event */
  .dither_timer_period  = 0U,
  .dither_duty_cycle    = 0U,
  .prescaler_mode       = (uint8_t)XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_ch1_enable       = 0U,
  .mcm_ch2_enable       = 0U,
  .slice_status         = (uint8_t)XMC_CCU8_SLICE_STATUS_CHANNEL_1,
  .passive_level_out0   = CCU8_PASSIVE_LEVEL,
  .passive_level_out1   = CCU8_PASSIVE_LEVEL,
  .asymmetric_pwm       = (uint8_t)true,
  .invert_out0          = 1U,
  .invert_out1          = 0U,
  .prescaler_initval    = (uint8_t)0,
  .float_limit          = 0U,
  .dither_limit         = 0U,
  .timer_concatenation  = 0U

};

/**
 *  Data Structure initialization - CC8 Slice configuration to trigger ADC conversion start.
 */
XMC_CCU8_SLICE_COMPARE_CONFIG_t CCU8_ADC_TriggerInit =
{
  .timer_mode           = (uint8_t)XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot             = 0U,
  .shadow_xfer_clear    = 1U,
  .dither_timer_period  = 0U,
  .dither_duty_cycle    = 0U,
  .prescaler_mode       = (uint8_t)XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_ch1_enable       = 0U,
  .mcm_ch2_enable       = 0U,
  .slice_status         = (uint8_t)XMC_CCU8_SLICE_STATUS_CHANNEL_1,
  .passive_level_out0   = 0,
  .passive_level_out1   = 0,
  .asymmetric_pwm       = (uint8_t)true,
  .invert_out0          = 0U,
  .invert_out1          = 1U,
  .prescaler_initval    = (uint8_t)0,
  .float_limit          = 0U,
  .dither_limit         = 0U,
  .timer_concatenation  = 0U

};

/**
 *  Data Structure initialization - CC8 Slice dead time configuration.
 */

XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t CCU8_PWM_Deadtime_config =
{
  .dtc = 0x3FU,
};

/**
 *  Data Structure initialization - CC8 Slice event 0 set to sync start.
 */
XMC_CCU8_SLICE_EVENT_CONFIG_t CCU8_Input_event0_config =
{
  .mapped_input = XMC_CCU8_SLICE_INPUT_H,
  .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE
};

/**
 *  Data Structure initialization - CC8 Slice trap event configuration.
 */
XMC_CCU8_SLICE_EVENT_CONFIG_t CCU8_Input_trap_config =
{
  .mapped_input = XMC_CCU8_SLICE_INPUT_A,
  .level = CCU8_INPUT_TRAP_LEVEL,
  .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
  .duration = XMC_CCU8_SLICE_EVENT_FILTER_7_CYCLES
};


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize CCU8 module for 3 phase pwm generation. Trap functionality enable */
void CCU8_Init(void)
{

	XMC_CCU8_Init(CCU8_MODULE, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
	/* Set Prescaler run bit */
	XMC_CCU8_StartPrescaler(CCU8_MODULE);

	XMC_CCU8_SLICE_CompareInit(CCU8_MODULE_PHASE_U, &CCU8_PhaseInit);
	XMC_CCU8_SLICE_CompareInit(CCU8_MODULE_PHASE_V, &CCU8_PhaseInit);
	XMC_CCU8_SLICE_CompareInit(CCU8_MODULE_PHASE_W, &CCU8_PhaseInit);
	XMC_CCU8_SLICE_CompareInit(CCU8_MODULE_ADC_TR, &CCU8_ADC_TriggerInit);


	/* Timer Period Shadow Value */
	XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_MODULE_PHASE_U, PERIOD_REG);
	XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_MODULE_PHASE_V, PERIOD_REG);
	XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_MODULE_PHASE_W, PERIOD_REG);


  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_PHASE_U, 0);
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_PHASE_U, PERIOD_REG+1);

  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_PHASE_V, 0);
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_PHASE_V, PERIOD_REG+1);

  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_PHASE_W, 0);
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_PHASE_W, PERIOD_REG+1);

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  /* First CCU83 period is a constant, e.g.: 2Tz. */
  XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_MODULE_ADC_TR, PERIOD_REG);
  /* For ADCTz1 trigger. */
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_ADC_TR, TRIGGER_POINT);
  /* For ADCTz2 trigger. */
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_ADC_TR, (TZ_PZV + TRIGGER_POINT));

#else

  XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_MODULE_ADC_TR, PERIOD_REG);
  /* For ADC trigger after CR1 */
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_MODULE_ADC_TR, (PERIOD_REG /2));
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_MODULE_ADC_TR, ((PERIOD_REG) /2) + 1);
#endif

	/* Dead time enabled, No prescaler for the dead time counters */
	/* Dead time could cause motor phase current distortion, especially at low motor speed */
	CCU8_PWM_Deadtime_config.dc1r = DEAD_TIME;
	CCU8_PWM_Deadtime_config.dc2r = DEAD_TIME;

	XMC_CCU8_SLICE_DeadTimeInit(CCU8_MODULE_PHASE_U, &CCU8_PWM_Deadtime_config);
	XMC_CCU8_SLICE_DeadTimeInit(CCU8_MODULE_PHASE_V, &CCU8_PWM_Deadtime_config);
	XMC_CCU8_SLICE_DeadTimeInit(CCU8_MODULE_PHASE_W, &CCU8_PWM_Deadtime_config);

	/* Enable shadow transfer for slice 0,1,2,3 for CCU80 Kernel */
	XMC_CCU8_EnableShadowTransfer(CCU8_MODULE, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_3));

	/*  Enable interrupt for CCU80 Period Match Enable */
	XMC_CCU8_SLICE_EnableEvent(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
	XMC_CCU8_SLICE_SetInterruptNode(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU8_SLICE_SR_ID_0);

	/* CCU80_0_IRQn = 25. CCU80 SR0 Interrupt has lower priority than ADC IRQ. CCU80_0_IRQHandler */
	NVIC_SetPriority (CCU80_0_IRQn, 2U);
	NVIC_EnableIRQ(CCU80_0_IRQn);

	/* Configuring CCU80 CC8xINS - Input Selector Configuration to SCU.GSC80 */
	/* Event0 -> INyH (SCU.GSC80, Global Start Control CCU80) for EV0IS */
	/* INyH for EV0IS */
	XMC_CCU8_SLICE_ConfigureEvent(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_event0_config);
	XMC_CCU8_SLICE_ConfigureEvent(CCU8_MODULE_PHASE_V, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_event0_config);
	XMC_CCU8_SLICE_ConfigureEvent(CCU8_MODULE_PHASE_W, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_event0_config);
	XMC_CCU8_SLICE_ConfigureEvent(CCU8_MODULE_ADC_TR, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_event0_config);

	/* Disable Global Start Control CCU80 */
	XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);

	/* Clear idle bit slice 0,1,2,3 for CCU80 Kernel */
	XMC_CCU8_EnableMultipleClocks(CCU8_MODULE, XMC_CCU8_GIDLC_CLOCK_MASK);

	/* Trap functionality enable */
	/* LPF2M = 11b, Event2 LPF -> 7 clock cycles of fCCU8 */
	/* EV2LM = 1b, Event2 (active LOW) -> TRAP -> CCU8x.INyA -> P0.12. Note trap function is level active */
	XMC_CCU8_SLICE_ConfigureEvent(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_EVENT_2, &CCU8_Input_trap_config);
	XMC_CCU8_SLICE_ConfigureEvent(CCU8_MODULE_PHASE_V, XMC_CCU8_SLICE_EVENT_2, &CCU8_Input_trap_config);
	XMC_CCU8_SLICE_ConfigureEvent(CCU8_MODULE_PHASE_W, XMC_CCU8_SLICE_EVENT_2, &CCU8_Input_trap_config);

		/* CCU8 TRAP functionality enabled so PWM outputs react on state of an input pin. For over-current protection */
	/* TS = 1, TRAP function enabled and connected to CCU80-CC8x Event2 */
	/* TRPSW = TRPSE = 1b, TRAP state only exited by SW, synch with PWM */
	/* TRAPE3/2/1/0 = 1b, TRAP affects CCU8x.OUTy0/1/2/3 */
	XMC_CCU8_SLICE_TrapConfig(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
	XMC_CCU8_SLICE_EnableTrap(CCU8_MODULE_PHASE_U, XMC_CCU8_TC_TRAPSE_MASK);

	XMC_CCU8_SLICE_TrapConfig(CCU8_MODULE_PHASE_V, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
	XMC_CCU8_SLICE_EnableTrap(CCU8_MODULE_PHASE_V, XMC_CCU8_TC_TRAPSE_MASK);

	XMC_CCU8_SLICE_TrapConfig(CCU8_MODULE_PHASE_W, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
	XMC_CCU8_SLICE_EnableTrap(CCU8_MODULE_PHASE_W, XMC_CCU8_TC_TRAPSE_MASK);

	/* Enable interrupt for CCU80-CC80 Event2 */
	XMC_CCU8_SLICE_EnableEvent(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2);

	/* Event2 interrupt forward to CC8ySR1 */
	XMC_CCU8_SLICE_SetInterruptNode(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2, XMC_CCU8_SLICE_SR_ID_1);

	/* CCU80_1_IRQn = 26. Trap protection interrupt. CCU80_1_IRQHandler Trap protection interrupt has highest priority */
	NVIC_SetPriority(CCU80_1_IRQn, 0U);
	NVIC_EnableIRQ(CCU80_1_IRQn);

	/* Interrupt Status Clear, for interrupts of Period Match, Trap Flag, and Event2 */
	CCU8_MODULE_PHASE_U->SWR |= 0x00000C01U;

}

/* API to enable synchronous start of CAPCOM modules. */
void CCUx_SynStart(void)
{
	/* Setup Event0 for external start trigger */
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_V, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_W, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_ADC_TR, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);

	/* Enable Global Start Control CCU80 */
	XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);

	/* Disable external start trigger */
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_U, XMC_CCU8_SLICE_EVENT_NONE, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_V, XMC_CCU8_SLICE_EVENT_NONE, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_PHASE_W, XMC_CCU8_SLICE_EVENT_NONE, XMC_CCU8_SLICE_START_MODE_TIMER_START);
	XMC_CCU8_SLICE_StartConfig(CCU8_MODULE_ADC_TR, XMC_CCU8_SLICE_EVENT_NONE, XMC_CCU8_SLICE_START_MODE_TIMER_START);

	/* Disable Global Start Control CCU80 */
	XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
}

/* API to initialize CC81 and CC82 slices to drive LEDs to indicate Trap event occurs */
void Init_CCU8x_for_TRAP_LED_Indicator (void)
{
	/* CCU81PSIV prescaler of clock fccu8/32768 = 1.953 kHz */
	XMC_CCU8_SLICE_SetPrescaler(CCU80_CC81, 0x0FU);

	/* CCU82PSIV prescaler of clock fccu8/32768 = 1.953 kHz */
	XMC_CCU8_SLICE_SetPrescaler(CCU80_CC82, 0x0FU);

	/* Timer Period Shadow Value */
	XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU80_CC81, LED_BLINK_PRS);
	XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU80_CC82, LED_BLINK_PRS);

	/* Timer Compare Shadow Value. LEDs blinking */
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU80_CC81, LED_BLINK_CRS1);
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU80_CC81, LED_BLINK_CRS2);
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU80_CC82, LED_BLINK_CRS1);
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU80_CC82, LED_BLINK_CRS2);

	/* Enable shadow transfer for slice 1,2 for CCU80 Kernel */
	XMC_CCU8_EnableShadowTransfer(CCU8_MODULE, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 | (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2));
}

