/**
 * @file adc.c
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
 * @file adc.c
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
#include <XMC1300.h>    /* SFR declarations of the selected device */
#include "adc.h"
#include "..\Configuration\pmsm_foc_user_mcuhwconfig.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - VADC global Configuration.
 */
XMC_VADC_GLOBAL_CONFIG_t VADC_global_config =
{
  .module_disable = 0U,
  .disable_sleep_mode_control = 1U,
  .clock_config =
  {
    .analog_clock_divider = 0U,       /*Divider Factor for the Analog Internal Clock*/
    .arbiter_clock_divider = 0U,
    .msb_conversion_clock = 0U,
  },
  .data_reduction_control         = 0U, /* Data Reduction disabled*/
  .wait_for_read_mode             = 0U, /* GLOBRES Register will not be overwriten untill the previous value is read*/
  .event_gen_enable               = 0U, /* Result Event from GLOBRES is disabled*/

  .boundary0    = 0U, /* Lower boundary value for Normal comparison mode*/
  .boundary1    = 0U/* Upper boundary value for Normal comparison mode*/
};

/**
 *  Data Structure initialization - VADC group 0 Configuration.
 */
XMC_VADC_GROUP_CONFIG_t VADC_grp0_init =
{
  .emux_config =
  {
    .stce_usage                = 0U, 					 /*Use STCE when the setting changes*/
    .emux_mode                 = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
    .emux_coding               = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
    .starting_external_channel = (uint32_t) 0U,                   /* Channel starts at 0 for EMUX*/
    .connected_channel         = (uint32_t) 0U                    /* Channel connected to EMUX*/
  },
  .class0 =
  {
    .sample_time_std_conv            = 1U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
    .sampling_phase_emux_channel     = (uint32_t) 1U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
  },  /* !<ICLASS-0 */
  .class1 =
  {
    .sample_time_std_conv = 0U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
    .sampling_phase_emux_channel     = (uint32_t) 0U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
  }, /* !< ICLASS-1 */
  .boundary0  = 0U,  /* Lower boundary value for Normal comparison mode*/
  .boundary1	= 0U,  /* Upper boundary value for Normal comparison mode*/
  .arbitration_round_length = (uint32_t) 0U,  /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
  .arbiter_mode             = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS,	/*Determines when the arbiter should run.*/
};

/**
 *  Data Structure initialization - VADC group 1 Configuration.
 */
XMC_VADC_GROUP_CONFIG_t VADC_grp1_init =
{
  .emux_config =
  {
    .stce_usage                = 0U, 					 /*Use STCE when the setting changes*/
    .emux_mode                 = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
    .emux_coding               = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
    .starting_external_channel = (uint32_t) 0U,                   /* Channel starts at 0 for EMUX*/
    .connected_channel         = (uint32_t) 0U                    /* Channel connected to EMUX*/
  },
  .class0 =
  {
    .sample_time_std_conv            = 1U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
    .sampling_phase_emux_channel     = (uint32_t) 1U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
  },  /* !<ICLASS-0 */
  .class1 =
  {
    .sample_time_std_conv = 0U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
    .sampling_phase_emux_channel     = (uint32_t) 0U,                /*The Sample time is (2*tadci)*/
    .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
  }, /* !< ICLASS-1 */
  .boundary0  = 0U,  /* Lower boundary value for Normal comparison mode*/
  .boundary1	= 0U,  /* Upper boundary value for Normal comparison mode*/
  .arbitration_round_length = (uint32_t) 0U,  /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
  .arbiter_mode             = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS,	/*Determines when the arbiter should run.*/
};

/**
 *  Data Structure initialization - VADC group queue request source.
 */
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/**
 *  Data Structure initialization - VADC group queue request source.
 */
XMC_VADC_QUEUE_CONFIG_t VADC_grp_queue_single_shunt_config =
{
  .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_CNR,     /* Conversion start mode WFS/CIR/CNR*/
  .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_3, /*The queue request source priority */
  .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P,      /*If trigger needed the signal input*/
  .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_ANY,   /*Trigger edge needed if trigger enabled*/
  .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_E,            /*If gating needed the signal input*/
  .timer_mode       = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger = (uint32_t) 1            /*External trigger Enabled/Disabled*/
};


XMC_VADC_BACKGROUND_CONFIG_t VADC_grp_scan_config =
{
  .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_WFS,
  .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_0,
  .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P,      /*If trigger needed the signal input*/
  .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_RISING,   /*Trigger edge needed if trigger enabled*/
  .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_E,            /*If gating needed the signal input*/
  .timer_mode       = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger = (uint32_t) 1,
  .load_mode       = (uint32_t) XMC_VADC_SCAN_LOAD_COMBINE,   /*Response from SCAN when a Load event occours.*/
};

/**
 *  Data Structure initialization - VADC group queue entries.
 */
XMC_VADC_QUEUE_ENTRY_t VADC_queue_entry_ss_Idc =
{
  .channel_num = VADC_ISS_CHANNEL,
  .external_trigger = true,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_queue_entry_ss_last_Idc =
{
  .channel_num = VADC_ISS_CHANNEL,
  .external_trigger = true,
  .generate_interrupt = true,
  .refill_needed = true
};

/**
 *  Data Structure initialization - VADC group channels.
 */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_ss_idc_init =
{
  .alias_channel = XMC_VADC_CHANNEL_ALIAS_DISABLED,
  .result_reg_number = VADC_IDC_SS_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = true,
  .sync_conversion = false
};

/* DC current ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_idc_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IDC_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};
/* DC voltage ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_vdc_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_VDC_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false,
#ifdef USER_VOLTAGE_PROTECTION
  .lower_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .upper_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1,
  .event_gen_criteria = XMC_VADC_CHANNEL_EVGEN_OUTBOUND,
#endif
};

/* Potiemeter ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_pot_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_POT_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

/* Idc Channel Result configuration structure*/
static const XMC_VADC_RESULT_CONFIG_t VADC_channel_ss_idc_1st_res_config =
{
  .data_reduction_control  = (uint32_t) 0,
  .post_processing_mode    = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
  .wait_for_read_mode      = (uint32_t) false,
  .part_of_fifo            = (uint32_t) false , /* Head result register not in FIFO*/
  .event_gen_enable        = (uint32_t) false , /* No result event generation */
};

/*Idc channel 2nd Result configuration structure*/
static const XMC_VADC_RESULT_CONFIG_t VADC_channel_ss_idc_2nd_res_config =
{
  .data_reduction_control  = (uint32_t) 0,  /* No Accumulation */
  .post_processing_mode    = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
  .wait_for_read_mode      = (uint32_t) false,  /* Disabled */
  .part_of_fifo            = (uint32_t) true,  /* Tail result register is a part of FIFO*/
  .event_gen_enable        = (uint32_t) false,   /* Result event Enabled*/
};

#elif(CURRENT_SENSING == USER_THREE_SHUNT_SYNC_CONV)
/**
 *  Data Structure initialization - VADC group queue request source.
 */
XMC_VADC_QUEUE_CONFIG_t VADC_grp_queue_config =
{
  .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_CIR,     /* Conversion start mode WFS/CIR/CNR*/
  .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_3, /*The queue request source priority */
  .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P,      /*If trigger needed the signal input*/
  .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_RISING,   /*Trigger edge needed if trigger enabled*/
  .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_E,            /*If gating needed the signal input*/
  .timer_mode       = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger = (uint32_t) 1            /*External trigger Enabled/Disabled*/
};

XMC_VADC_BACKGROUND_CONFIG_t VADC_grp_scan_config =
{
  .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_CIR,
  .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_0,
  .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P,      /*If trigger needed the signal input*/
  .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_NONE,   /*Trigger edge needed if trigger enabled*/
  .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_E,            /*If gating needed the signal input*/
  .timer_mode       = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger = (uint32_t) 0,
  .load_mode       = (uint32_t) XMC_VADC_SCAN_LOAD_COMBINE,   /*Response from SCAN when a Load event occours.*/
};

/**
 *  Data Structure initialization - VADC group queue entries.
 */
XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_alias_ch0 =
{
  .channel_num = VADC_I1_CHANNEL,
  .external_trigger = true,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_alias_ch1 =
{
  .channel_num = VADC_I3_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};
XMC_VADC_QUEUE_ENTRY_t VADC_queue_entry_Idc =
{
  .channel_num = VADC_IDC_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};
XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_volt_DC =
{
  .channel_num = VADC_VDC_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_pot =
{
  .channel_num = VADC_POT_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

/**
 *  Data Structure initialization - VADC group channels.
 */
XMC_VADC_CHANNEL_CONFIG_t VADC_grp1_ch0_init =
{
  .alias_channel = (int8_t)VADC_G1_CHANNEL_ALIAS0,
  .result_reg_number = 0,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = true
};

XMC_VADC_CHANNEL_CONFIG_t VADC_grp1_ch1_init =
{
  .alias_channel = (int8_t)VADC_G1_CHANNEL_ALIAS1,
  .result_reg_number = 1,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = true
};

XMC_VADC_CHANNEL_CONFIG_t VADC_grp0_ch0_init =
{
  .alias_channel = (int8_t)VADC_G0_CHANNEL_ALIAS0,
  .result_reg_number = 0,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

XMC_VADC_CHANNEL_CONFIG_t VADC_grp0_ch1_init =
{
  .alias_channel = (int8_t)VADC_G0_CHANNEL_ALIAS1,
  .result_reg_number = 1,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};
/* DC current ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_idc_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IDC_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

/* DC voltage ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_vdc_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_VDC_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false,
#ifdef USER_VOLTAGE_PROTECTION
  .lower_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .upper_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1,
  .event_gen_criteria = XMC_VADC_CHANNEL_EVGEN_OUTBOUND,
#endif
};

/* Potiemeter ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_pot_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_POT_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

#elif(CURRENT_SENSING == USER_THREE_SHUNT_ASSYNC_CONV)
/**
 *  Data Structure initialization - VADC group queue request source.
 */
XMC_VADC_QUEUE_CONFIG_t VADC_grp_queue_config =
{
  .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_CIR,     /* Conversion start mode WFS/CIR/CNR*/
  .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_3, /*The queue request source priority */
  .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P,      /*If trigger needed the signal input*/
  .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_RISING,   /*Trigger edge needed if trigger enabled*/
  .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_E,            /*If gating needed the signal input*/
  .timer_mode       = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger = (uint32_t) 1            /*External trigger Enabled/Disabled*/
};

XMC_VADC_QUEUE_ENTRY_t VADC_queue_entry_Idc =
{
  .channel_num = VADC_IDC_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};
XMC_VADC_QUEUE_ENTRY_t VADC_queue_entry_Iu =
{
  .channel_num = VADC_IU_CHANNEL,
  .external_trigger = true,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_queue_entry_Iv =
{
  .channel_num = VADC_IV_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_queue_entry_Iw =
{
  .channel_num = VADC_IW_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_volt_DC =
{
  .channel_num = VADC_VDC_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_grp1_queue_entry_pot =
{
  .channel_num = VADC_POT_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

/**
 *  Data Structure initialization - Motor Phases VADC channels.
 */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_Iu_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IU_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

XMC_VADC_CHANNEL_CONFIG_t VADC_channel_Iv_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IV_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

XMC_VADC_CHANNEL_CONFIG_t VADC_channel_Iw_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IW_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

/* DC current ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_idc_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IDC_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

/* DC voltage ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_vdc_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_VDC_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false,
#ifdef USER_VOLTAGE_PROTECTION
  .lower_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .upper_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1,
  .event_gen_criteria = XMC_VADC_CHANNEL_EVGEN_OUTBOUND,
#endif
};

/* Potiemeter ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_pot_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_POT_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

#endif

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
#if (CURRENT_SENSING ==  USER_THREE_SHUNT_SYNC_CONV)
/* API to initialize VADC module for 3-shunt phase current sensing */
void ADC_Init(void)
{
  #if(INTERNAL_OP_GAIN == ENABLED)
  uint32_t gain_factor;
  #endif

	XMC_VADC_GLOBAL_Init(VADC, &VADC_global_config);
	XMC_VADC_GROUP_Init(VADC_G0, &VADC_grp0_init);
	XMC_VADC_GROUP_Init(VADC_G1, &VADC_grp1_init);

	/* Configuration of VADC_G1/0,  Turn on ADC modules */
	XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
	XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

	/* Trigger Start-up Calibration */
	XMC_VADC_GLOBAL_StartupCalibration(VADC);

	XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 0U);
	XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 1U);

	/* Configuration of VADC_G1 - Q source */
	/* External trigger	1,  refill 1 */
	XMC_VADC_GROUP_QueueInit(VADC_G1, &VADC_grp_queue_config);

	 /* Configure the gating mode for queue*/
	XMC_VADC_GROUP_QueueSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);

	/* Request the LLD to insert the channel */
	XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_grp1_queue_entry_alias_ch0);

	XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_grp1_queue_entry_alias_ch1);

	if(VADC_IDC_GROUP_NO == 1)
  {
    XMC_VADC_GROUP_QueueInsertChannel(VADC_IDC_GROUP, VADC_queue_entry_Idc);
  }

  if((VADC_VDC_GROUP_NO == 0) | (VADC_POT_GROUP_NO == 0))
  { /* Configure scan request source*/
    XMC_VADC_GROUP_ScanInit(VADC_G0, &VADC_grp_scan_config);
    /* Configure the gating mode for scan*/
    XMC_VADC_GROUP_ScanSetGatingMode(VADC_G0,XMC_VADC_GATEMODE_IGNORE);
  }
  if((VADC_IDC_GROUP_NO == 1) && (VADC_IDC_CHANNEL > 1))
  { /* Configure scan request source*/
    XMC_VADC_GROUP_ScanInit(VADC_G1, &VADC_grp_scan_config);
    /* Configure the gating mode for scan*/
    XMC_VADC_GROUP_ScanSetGatingMode(VADC_G1,XMC_VADC_GATEMODE_IGNORE);
    /* Initializes the DC Link current channel for conversion */
    XMC_VADC_GROUP_ChannelInit(VADC_IDC_GROUP, VADC_IDC_CHANNEL, &VADC_channel_idc_init);

  }
	/* Master group - G1 for Synchronous ADC */
	/* I1, Result Register RES0 */
	XMC_VADC_GROUP_ChannelInit(VADC_I1_GROUP, VADC_I1_CHANNEL, &VADC_grp1_ch0_init);

	/* I3, Result Register RES1 */
	XMC_VADC_GROUP_ChannelInit(VADC_I3_GROUP, VADC_I3_CHANNEL, &VADC_grp1_ch1_init);

	/* I2, Result Register RES0 */
	XMC_VADC_GROUP_ChannelInit(VADC_I2_GROUP, VADC_I2_CHANNEL, &VADC_grp0_ch0_init);

	/* Idc, Result Register RES1 */
	XMC_VADC_GROUP_ChannelInit(VADC_I4_GROUP, VADC_I4_CHANNEL, &VADC_grp0_ch1_init);
	//XMC_VADC_GROUP_ChannelInit(VADC_IDC_GROUP, VADC_IDC_CHANNEL, &VADC_channel_idc_init);

  #if(INTERNAL_OP_GAIN == ENABLED)
  /* Channel Gain factor setting for Iu, Iv, Iw and DC link current */
  #if(OP_GAIN_FACTOR == 1U)
    gain_factor = SHS_GAIN_FACTOR_1;
  #elif(OP_GAIN_FACTOR == 3U)
    gain_factor = SHS_GAIN_FACTOR_3;
  #elif(OP_GAIN_FACTOR == 6U)
    gain_factor = SHS_GAIN_FACTOR_6;
  #elif(OP_GAIN_FACTOR == 12U)
    gain_factor = SHS_GAIN_FACTOR_12;
  #endif

  if(VADC_IU_G0_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_0, VADC_IU_G0_CHANNEL);
  }
  if(VADC_IU_G1_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_1, VADC_IU_G1_CHANNEL);
  }
  if(VADC_IV_G0_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_0, VADC_IV_G0_CHANNEL);
  }
  if(VADC_IV_G1_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_1, VADC_IV_G1_CHANNEL);
  }
  if(VADC_IW_G0_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_0, VADC_IW_G0_CHANNEL);
  }
  if(VADC_IW_G1_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_1, VADC_IW_G1_CHANNEL);
  }
  #endif

	XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_OFF);
	XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_OFF);

	/* G0: synchronization slave */
	XMC_VADC_GROUP_SetSyncSlave(VADC_G0, 1U, 0U);

	/* Ready input R1 is considered */
	XMC_VADC_GROUP_CheckSlaveReadiness(VADC_G0, 0U);

	/* G1: synchronization master */
	XMC_VADC_GROUP_SetSyncMaster(VADC_G1);

	/* ANONS = 11B: Normal Operation */
	XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

  XMC_VADC_GLOBAL_BackgroundEnableContinuousMode(VADC);
  XMC_VADC_GLOBAL_BackgroundTriggerConversion(VADC);
  XMC_VADC_GLOBAL_BackgroundSetGatingMode(VADC,XMC_VADC_GATEMODE_IGNORE);


  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_IDC_GROUP_NO, VADC_IDC_CHANNEL);

  if(VADC_IDC_GROUP_NO == 0)
  {
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_IDC_GROUP,VADC_IDC_CHANNEL);
  }
}
#elif (CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
void ADC_Init(void)
{
  XMC_VADC_GLOBAL_Init(VADC, &VADC_global_config);
  XMC_VADC_GROUP_Init(VADC_G0, &VADC_grp0_init);
  XMC_VADC_GROUP_Init(VADC_G1, &VADC_grp1_init);

  /* Configuration of VADC_G1/0,  Turn on ADC modules */
  XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
  XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

  /* Trigger Start-up Calibration */
  XMC_VADC_GLOBAL_StartupCalibration(VADC);
  XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 0U);
  XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 1U);

  /* Configuration of VADC_IDC group - Q source */
  /* External trigger 1,  refill 1 */
  XMC_VADC_GROUP_QueueInit(VADC_ISS_GROUP, &VADC_grp_queue_single_shunt_config);

  /* Configure scan request source*/
  XMC_VADC_GROUP_ScanInit(VADC_G0, &VADC_grp_scan_config);
  XMC_VADC_GROUP_ScanInit(VADC_G1, &VADC_grp_scan_config);

  /* Configure the gating mode for queue*/
  XMC_VADC_GROUP_QueueSetGatingMode(VADC_ISS_GROUP, XMC_VADC_GATEMODE_IGNORE);
  XMC_VADC_GROUP_ScanSetGatingMode(VADC_G0,XMC_VADC_GATEMODE_IGNORE);
  XMC_VADC_GROUP_ScanSetGatingMode(VADC_G1,XMC_VADC_GATEMODE_IGNORE);
  /* Request the LLD to insert the channel */
  XMC_VADC_GROUP_QueueInsertChannel(VADC_ISS_GROUP, VADC_queue_entry_ss_Idc);
  XMC_VADC_GROUP_QueueInsertChannel(VADC_ISS_GROUP, VADC_queue_entry_ss_last_Idc);

  /* Channel initialisation - Result Register / Gain factor setting  */
  /* Idc, Result Register RES1 */
  XMC_VADC_GROUP_ChannelInit(VADC_ISS_GROUP, VADC_ISS_CHANNEL, &VADC_channel_ss_idc_init);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_ISS_GROUP_NO, VADC_ISS_CHANNEL);

  XMC_VADC_GROUP_ResultInit(VADC_ISS_GROUP, VADC_IDC_SS_RESULT_REG, &VADC_channel_ss_idc_1st_res_config);
  XMC_VADC_GROUP_ResultInit(VADC_ISS_GROUP, VADC_IDC_SS_RESULT_REG_FIFO1, &VADC_channel_ss_idc_2nd_res_config);

  XMC_VADC_GLOBAL_BackgroundEnableContinuousMode(VADC);
  XMC_VADC_GLOBAL_BackgroundTriggerConversion(VADC);
  XMC_VADC_GLOBAL_BackgroundSetGatingMode(VADC,XMC_VADC_GATEMODE_IGNORE);

  /* Initializes the DC Link current channel for conversion */
  XMC_VADC_GROUP_ChannelInit(VADC_IDC_GROUP, VADC_IDC_CHANNEL, &VADC_channel_idc_init);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_IDC_GROUP_NO, VADC_IDC_CHANNEL);

  XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_IDC_GROUP,VADC_IDC_CHANNEL);


  /*Interrupt Configuration*/
  XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(VADC_ISS_GROUP, XMC_VADC_SR_GROUP_SR1);

#if(VADC_ISS_GROUP_NO == 0U)
  NVIC_SetPriority((IRQn_Type)VADC0_G0_1_IRQn, 0U);
  NVIC_EnableIRQ(VADC0_G0_1_IRQn);
#else
  NVIC_SetPriority((IRQn_Type)VADC0_G1_1_IRQn, 0U);
  NVIC_EnableIRQ(VADC0_G1_1_IRQn);
#endif
  XMC_VADC_GROUP_QueueClearReqSrcEvent(VADC_ISS_GROUP);
  /* Connect RS Events to NVIC nodes */

}
#else
void ADC_Init(void)
{
  XMC_VADC_GLOBAL_Init(VADC, &VADC_global_config);
  XMC_VADC_GROUP_Init(VADC_G0, &VADC_grp0_init);
  XMC_VADC_GROUP_Init(VADC_G1, &VADC_grp1_init);

  /* Configuration of VADC_G1/0,  Turn on ADC modules */
  XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
  XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

  /* Trigger Start-up Calibration */
  XMC_VADC_GLOBAL_StartupCalibration(VADC);
  XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 0U);
  XMC_VADC_GLOBAL_DisablePostCalibration(VADC, 1U);

  /* Configuration of VADC_G1 - Q source */
  /* External trigger 1,  refill 1 */
  XMC_VADC_GROUP_QueueInit(VADC_G0, &VADC_grp_queue_config);
  XMC_VADC_GROUP_QueueInit(VADC_G1, &VADC_grp_queue_config);

   /* Configure the gating mode for queue*/
  XMC_VADC_GROUP_QueueSetGatingMode(VADC_G0, XMC_VADC_GATEMODE_IGNORE);
  XMC_VADC_GROUP_QueueSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);

  /* Request the LLD to insert the channel */
  XMC_VADC_GROUP_QueueInsertChannel(VADC_IU_GROUP, VADC_queue_entry_Iu);
  XMC_VADC_GROUP_QueueInsertChannel(VADC_IV_GROUP, VADC_queue_entry_Iv);
  XMC_VADC_GROUP_QueueInsertChannel(VADC_IW_GROUP, VADC_queue_entry_Iw);
  XMC_VADC_GROUP_QueueInsertChannel(VADC_IDC_GROUP, VADC_queue_entry_Idc);

  /* Channel initialisation - Result Register / Gain factor setting  */
  XMC_VADC_GROUP_ChannelInit(VADC_IU_GROUP, VADC_IU_CHANNEL, &VADC_channel_Iu_init);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_IU_GROUP_NO, VADC_IU_CHANNEL);

  /* Channel IV */
  XMC_VADC_GROUP_ChannelInit(VADC_IV_GROUP, VADC_IV_CHANNEL, &VADC_channel_Iv_init);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_IV_GROUP_NO, VADC_IV_CHANNEL);
  /* Channel IW */
  XMC_VADC_GROUP_ChannelInit(VADC_IW_GROUP, VADC_IW_CHANNEL, &VADC_channel_Iw_init);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_IW_GROUP_NO, VADC_IW_CHANNEL);

  /* Idc, Result Register RES1 */
  XMC_VADC_GROUP_ChannelInit(VADC_IDC_GROUP, VADC_IDC_CHANNEL, &VADC_channel_idc_init);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_IDC_GROUP_NO, VADC_IDC_CHANNEL);
}
#endif

/* API to initialize VADC channel for potentiameter voltage sensing that is used to set user speed */
void ADC_Pot_Init(void)
{
  /* Initializes the POT VADC channel for conversion */
  XMC_VADC_GROUP_ChannelInit(VADC_POT_GROUP, VADC_POT_CHANNEL, &VADC_channel_pot_init);

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_POT_GROUP,VADC_POT_CHANNEL);
#else
  if(VADC_POT_GROUP_NO == 0)
  {
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_POT_GROUP,VADC_POT_CHANNEL);
  }
  else
  {
    /* Request to insert the channel to the queue source */
    XMC_VADC_GROUP_QueueInsertChannel(VADC_POT_GROUP, VADC_grp1_queue_entry_pot);
  }
#endif
}

/* API to initialize VADC channel for DC link voltage sensing */
void ADC_DCLink_Init(void)
{
  /* Initializes the DC Link VADC channel for conversion */
  XMC_VADC_GROUP_ChannelInit(VADC_VDC_GROUP, VADC_VDC_CHANNEL, &VADC_channel_vdc_init);

#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
  XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_VDC_GROUP,VADC_VDC_CHANNEL);
#else
  if(VADC_VDC_GROUP_NO == 0)
  {
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_VDC_GROUP,VADC_VDC_CHANNEL);
  }
  else
  {
    /* Request to insert the channel to the queue source */
    XMC_VADC_GROUP_QueueInsertChannel(VADC_VDC_GROUP, VADC_grp1_queue_entry_volt_DC);
  }
#endif
  #ifdef USER_VOLTAGE_PROTECTION
  XMC_VADC_GROUP_SetIndividualBoundary(VADC_VDC_GROUP,XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,VDC_MIN_LIMIT);
  XMC_VADC_GROUP_SetIndividualBoundary(VADC_VDC_GROUP,XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1,VDC_OVER_LIMIT);
  #endif
}

