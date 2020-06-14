/**
 * @file xmc_can_map.h
 * @date 2015-10-09
 *
 * @cond
*********************************************************************************************************************
 * XMClib v2.1.0 - XMC Peripheral Driver Library 
 *
 * Copyright (c) 2015, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes with
 * Infineon Technologies AG dave@infineon.com).
 *********************************************************************************************************************
 *
 * Change History
 * --------------
 *
 * 2015-09-15:
 *     - Initial version
 *
 * @endcond
 *
 */

#ifndef XMC_CAN_MAP_H
#define XMC_CAN_MAP_H

/*******************************************************************************
 * MACROS
 *******************************************************************************/

#if (UC_DEVICE == XMC1403) && (UC_PACKAGE == VQFN48)
#define CAN_NODE0_RXD_P0_4   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P0_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P0_14   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE0_RXD_P0_15   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE0_RXD_P2_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE0_RXD_P2_1   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE0_RXD_P1_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE0_RXD_P1_1      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#define CAN_NODE1_RXD_P0_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P0_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_10   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE1_RXD_P2_11   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE1_RXD_P1_2   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE1_RXD_P1_3      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#endif

#if (UC_DEVICE == XMC1403) && (UC_PACKAGE == VQFN64)
#define CAN_NODE0_RXD_P0_4   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P0_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P0_14   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE0_RXD_P0_15   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE0_RXD_P2_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE0_RXD_P2_1   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE0_RXD_P1_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE0_RXD_P1_1      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#define CAN_NODE1_RXD_P0_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P0_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P4_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P4_9   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_P2_10   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE1_RXD_P2_11   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE1_RXD_P1_2   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE1_RXD_P1_3      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#endif

#if (UC_DEVICE == XMC1404) && (UC_PACKAGE == VQFN48)
#define CAN_NODE0_RXD_P0_4   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P0_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P0_14   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE0_RXD_P0_15   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE0_RXD_P2_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE0_RXD_P2_1   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE0_RXD_P1_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE0_RXD_P1_1      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#define CAN_NODE1_RXD_P0_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P0_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_10   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE1_RXD_P2_11   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE1_RXD_P1_2   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE1_RXD_P1_3      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#endif

#if (UC_DEVICE == XMC1404) && (UC_PACKAGE == VQFN64)
#define CAN_NODE0_RXD_P0_4   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P0_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P0_14   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE0_RXD_P0_15   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE0_RXD_P2_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE0_RXD_P2_1   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE0_RXD_P1_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE0_RXD_P1_1      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#define CAN_NODE1_RXD_P0_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P0_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P4_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P4_9   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_P2_10   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE1_RXD_P2_11   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE1_RXD_P1_2   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE1_RXD_P1_3      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#endif

#if (UC_DEVICE == XMC1404) && (UC_PACKAGE == LQFP64)
#define CAN_NODE0_RXD_P0_4   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P0_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P0_14   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE0_RXD_P0_15   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE0_RXD_P2_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE0_RXD_P2_1   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE0_RXD_P1_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE0_RXD_P1_1      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#define CAN_NODE1_RXD_P0_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P0_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P4_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P4_9   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_P2_10   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCE
#define CAN_NODE1_RXD_P2_11   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE1_RXD_P1_2   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCG
#define CAN_NODE1_RXD_P1_3      XMC_CAN_NODE_RECEIVE_INPUT_RXDCH
#endif

#if (UC_DEVICE == XMC4100) && (UC_PACKAGE == LQFP64)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif

#if (UC_DEVICE == XMC4100) && (UC_PACKAGE == LQFP64)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif

#if (UC_DEVICE == XMC4100) && (UC_PACKAGE == VQFN48)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif

#if (UC_DEVICE == XMC4108) && (UC_PACKAGE == LQFP64)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#endif


#if (UC_DEVICE == XMC4108) && (UC_PACKAGE == VQFN48)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#endif


#if (UC_DEVICE == XMC4200) && (UC_PACKAGE == LQFP64)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif


#if (UC_DEVICE == XMC4200) && (UC_PACKAGE == VQFN48)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif


#if (UC_DEVICE == XMC4400) && (UC_PACKAGE == LQFP100)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif


#if (UC_DEVICE == XMC4400) && (UC_PACKAGE == LQFP64)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif


#if (UC_DEVICE == XMC4402) && (UC_PACKAGE == LQFP100)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif


#if (UC_DEVICE == XMC4402) && (UC_PACKAGE == LQFP64)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif


#if (UC_DEVICE == XMC4500) && (UC_PACKAGE == BGA144)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P3_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P3_11   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P1_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE2_RXD_P1_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE2_RXD_P3_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE2_RXD_P4_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE2_RXD_CAN1INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif


#if (UC_DEVICE == XMC4500) && (UC_PACKAGE == LQFP100)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE2_RXD_P1_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE2_RXD_CAN1INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif


#if (UC_DEVICE == XMC4500) && (UC_PACKAGE == LQFP144)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P3_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P3_11   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P1_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE2_RXD_P1_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE2_RXD_P3_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE2_RXD_P4_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE2_RXD_CAN1INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif


#if (UC_DEVICE == XMC4502) && (UC_PACKAGE == LQFP100)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE2_RXD_P1_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE2_RXD_CAN1INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#endif

#if (UC_DEVICE == XMC4800) && (UC_PACKAGE == LQFP144)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P3_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P3_11   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P1_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE2_RXD_P1_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE2_RXD_P3_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE2_RXD_P4_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE2_RXD_CAN1INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE3_RXD_P0_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE3_RXD_P6_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE4_RXD_P2_15   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE4_RXD_P14_4   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE5_RXD_P5_10   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE5_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#endif

#if (UC_DEVICE == XMC4800) && (UC_PACKAGE == LQFP100)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P3_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P1_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE2_RXD_P1_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE2_RXD_CAN1INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE3_RXD_P0_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE3_RXD_P6_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE4_RXD_P2_15   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE4_RXD_P14_4   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE5_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#endif

#if (UC_DEVICE == XMC4800) && (UC_PACKAGE == LFBGA196)
#define CAN_NODE0_RXD_P1_5   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE0_RXD_P14_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE0_RXD_P3_12   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE1_RXD_P3_11   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE1_RXD_P1_13   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE1_RXD_P1_4      XMC_CAN_NODE_RECEIVE_INPUT_RXDCD
#define CAN_NODE1_RXD_CAN0INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE2_RXD_P1_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE2_RXD_P3_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE2_RXD_P4_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE2_RXD_CAN1INS   XMC_CAN_NODE_RECEIVE_INPUT_RXDCF
#define CAN_NODE3_RXD_P0_8   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE3_RXD_P6_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE3_RXD_P7_1   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE4_RXD_P2_15   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE4_RXD_P14_4   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE4_RXD_P7_3   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#define CAN_NODE5_RXD_P5_10   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCA
#define CAN_NODE5_RXD_P2_6   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCB
#define CAN_NODE5_RXD_P8_0   	XMC_CAN_NODE_RECEIVE_INPUT_RXDCC
#endif

#if (UC_SERIES == XMC47) || (UC_SERIES == XMC48) || (UC_SERIES == XMC14)
#define CAN_MO0 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[0]))
#define CAN_MO1 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[1]))
#define CAN_MO2 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[2]))
#define CAN_MO3 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[3]))
#define CAN_MO4 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[4]))
#define CAN_MO5 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[5]))
#define CAN_MO6 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[6]))
#define CAN_MO7 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[7]))
#define CAN_MO8 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[8]))
#define CAN_MO9 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[9]))
#define CAN_MO10 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[10]))
#define CAN_MO11 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[11]))
#define CAN_MO12 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[12]))
#define CAN_MO13 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[13]))
#define CAN_MO14 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[14]))
#define CAN_MO15 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[15]))
#define CAN_MO16 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[16]))
#define CAN_MO17 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[17]))
#define CAN_MO18 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[18]))
#define CAN_MO19 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[19]))
#define CAN_MO20 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[20]))
#define CAN_MO21 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[21]))
#define CAN_MO22 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[22]))
#define CAN_MO23 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[23]))
#define CAN_MO24 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[24]))
#define CAN_MO25 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[25]))
#define CAN_MO26 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[26]))
#define CAN_MO27 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[27]))
#define CAN_MO28 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[28]))
#define CAN_MO29 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[29]))
#define CAN_MO30 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[30]))
#define CAN_MO31 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[31]))
#endif


#if (UC_SERIES == XMC47) || (UC_SERIES == XMC48)
#define CAN_MO32 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[32]))
#define CAN_MO33 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[33]))
#define CAN_MO34 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[34]))
#define CAN_MO35 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[35]))
#define CAN_MO36 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[36]))
#define CAN_MO37 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[37]))
#define CAN_MO38 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[38]))
#define CAN_MO39 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[39]))
#define CAN_MO40 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[40]))
#define CAN_MO41 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[41]))
#define CAN_MO42 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[42]))
#define CAN_MO43 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[43]))
#define CAN_MO44 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[44]))
#define CAN_MO45 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[45]))
#define CAN_MO46 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[46]))
#define CAN_MO47 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[47]))
#define CAN_MO48 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[48]))
#define CAN_MO49 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[49]))
#define CAN_MO50 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[50]))
#define CAN_MO51 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[51]))
#define CAN_MO52 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[52]))
#define CAN_MO53 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[53]))
#define CAN_MO54 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[54]))
#define CAN_MO55 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[55]))
#define CAN_MO56 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[56]))
#define CAN_MO57 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[57]))
#define CAN_MO58 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[58]))
#define CAN_MO59 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[59]))
#define CAN_MO60 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[60]))
#define CAN_MO61 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[61]))
#define CAN_MO62 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[62]))
#define CAN_MO63 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[63]))
#define CAN_MO64 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[64]))
#define CAN_MO65 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[65]))
#define CAN_MO66 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[66]))
#define CAN_MO67 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[67]))
#define CAN_MO68 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[68]))
#define CAN_MO69 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[69]))
#define CAN_MO70 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[70]))
#define CAN_MO71 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[71]))
#define CAN_MO72 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[72]))
#define CAN_MO73 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[73]))
#define CAN_MO74 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[74]))
#define CAN_MO75 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[75]))
#define CAN_MO76 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[76]))
#define CAN_MO77 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[77]))
#define CAN_MO78 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[78]))
#define CAN_MO79 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[79]))
#define CAN_MO80 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[80]))
#define CAN_MO81 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[81]))
#define CAN_MO82 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[82]))
#define CAN_MO83 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[83]))
#define CAN_MO84 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[84]))
#define CAN_MO85 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[85]))
#define CAN_MO86 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[86]))
#define CAN_MO87 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[87]))
#define CAN_MO88 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[88]))
#define CAN_MO89 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[89]))
#define CAN_MO90 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[90]))
#define CAN_MO91 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[91]))
#define CAN_MO92 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[92]))
#define CAN_MO93 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[93]))
#define CAN_MO94 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[94]))
#define CAN_MO95 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[95]))
#define CAN_MO96 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[96]))
#define CAN_MO97 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[97]))
#define CAN_MO98 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[98]))
#define CAN_MO99 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[99]))
#define CAN_MO100 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[100]))
#define CAN_MO101 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[101]))
#define CAN_MO102 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[102]))
#define CAN_MO103 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[103]))
#define CAN_MO104 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[104]))
#define CAN_MO105 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[105]))
#define CAN_MO106 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[106]))
#define CAN_MO107 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[107]))
#define CAN_MO108 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[108]))
#define CAN_MO109 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[109]))
#define CAN_MO110 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[110]))
#define CAN_MO111 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[111]))
#define CAN_MO112 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[112]))
#define CAN_MO113 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[113]))
#define CAN_MO114 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[114]))
#define CAN_MO115 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[115]))
#define CAN_MO116 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[116]))
#define CAN_MO117 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[117]))
#define CAN_MO118 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[118]))
#define CAN_MO119 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[119]))
#define CAN_MO120 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[120]))
#define CAN_MO121 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[121]))
#define CAN_MO122 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[122]))
#define CAN_MO123 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[123]))
#define CAN_MO124 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[124]))
#define CAN_MO125 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[125]))
#define CAN_MO126 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[126]))
#define CAN_MO127 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[127]))
#define CAN_MO128 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[128]))
#define CAN_MO129 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[129]))
#define CAN_MO130 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[130]))
#define CAN_MO131 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[131]))
#define CAN_MO132 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[132]))
#define CAN_MO133 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[133]))
#define CAN_MO134 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[134]))
#define CAN_MO135 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[135]))
#define CAN_MO136 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[136]))
#define CAN_MO137 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[137]))
#define CAN_MO138 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[138]))
#define CAN_MO139 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[139]))
#define CAN_MO140 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[140]))
#define CAN_MO141 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[141]))
#define CAN_MO142 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[142]))
#define CAN_MO143 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[143]))
#define CAN_MO144 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[144]))
#define CAN_MO145 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[145]))
#define CAN_MO146 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[146]))
#define CAN_MO147 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[147]))
#define CAN_MO148 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[148]))
#define CAN_MO149 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[149]))
#define CAN_MO150 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[150]))
#define CAN_MO151 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[151]))
#define CAN_MO152 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[152]))
#define CAN_MO153 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[153]))
#define CAN_MO154 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[154]))
#define CAN_MO155 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[155]))
#define CAN_MO156 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[156]))
#define CAN_MO157 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[157]))
#define CAN_MO158 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[158]))
#define CAN_MO159 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[159]))
#define CAN_MO160 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[160]))
#define CAN_MO161 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[161]))
#define CAN_MO162 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[162]))
#define CAN_MO163 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[163]))
#define CAN_MO164 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[164]))
#define CAN_MO165 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[165]))
#define CAN_MO166 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[166]))
#define CAN_MO167 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[167]))
#define CAN_MO168 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[168]))
#define CAN_MO169 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[169]))
#define CAN_MO170 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[170]))
#define CAN_MO171 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[171]))
#define CAN_MO172 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[172]))
#define CAN_MO173 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[173]))
#define CAN_MO174 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[174]))
#define CAN_MO175 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[175]))
#define CAN_MO176 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[176]))
#define CAN_MO177 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[177]))
#define CAN_MO178 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[178]))
#define CAN_MO179 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[179]))
#define CAN_MO180 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[180]))
#define CAN_MO181 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[181]))
#define CAN_MO182 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[182]))
#define CAN_MO183 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[183]))
#define CAN_MO184 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[184]))
#define CAN_MO185 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[185]))
#define CAN_MO186 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[186]))
#define CAN_MO187 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[187]))
#define CAN_MO188 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[188]))
#define CAN_MO189 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[189]))
#define CAN_MO190 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[190]))
#define CAN_MO191 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[191]))
#define CAN_MO192 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[192]))
#define CAN_MO193 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[193]))
#define CAN_MO194 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[194]))
#define CAN_MO195 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[195]))
#define CAN_MO196 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[196]))
#define CAN_MO197 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[197]))
#define CAN_MO198 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[198]))
#define CAN_MO199 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[199]))
#define CAN_MO200 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[200]))
#define CAN_MO201 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[201]))
#define CAN_MO202 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[202]))
#define CAN_MO203 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[203]))
#define CAN_MO204 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[204]))
#define CAN_MO205 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[205]))
#define CAN_MO206 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[206]))
#define CAN_MO207 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[207]))
#define CAN_MO208 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[208]))
#define CAN_MO209 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[209]))
#define CAN_MO210 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[210]))
#define CAN_MO211 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[211]))
#define CAN_MO212 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[212]))
#define CAN_MO213 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[213]))
#define CAN_MO214 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[214]))
#define CAN_MO215 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[215]))
#define CAN_MO216 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[216]))
#define CAN_MO217 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[217]))
#define CAN_MO218 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[218]))
#define CAN_MO219 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[219]))
#define CAN_MO220 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[220]))
#define CAN_MO221 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[221]))
#define CAN_MO222 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[222]))
#define CAN_MO223 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[223]))
#define CAN_MO224 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[224]))
#define CAN_MO225 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[225]))
#define CAN_MO226 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[226]))
#define CAN_MO227 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[227]))
#define CAN_MO228 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[228]))
#define CAN_MO229 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[229]))
#define CAN_MO230 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[230]))
#define CAN_MO231 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[231]))
#define CAN_MO232 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[232]))
#define CAN_MO233 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[233]))
#define CAN_MO234 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[234]))
#define CAN_MO235 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[235]))
#define CAN_MO236 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[236]))
#define CAN_MO237 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[237]))
#define CAN_MO238 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[238]))
#define CAN_MO239 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[239]))
#define CAN_MO240 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[240]))
#define CAN_MO241 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[241]))
#define CAN_MO242 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[242]))
#define CAN_MO243 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[243]))
#define CAN_MO244 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[244]))
#define CAN_MO245 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[245]))
#define CAN_MO246 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[246]))
#define CAN_MO247 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[247]))
#define CAN_MO248 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[248]))
#define CAN_MO249 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[249]))
#define CAN_MO250 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[250]))
#define CAN_MO251 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[251]))
#define CAN_MO252 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[252]))
#define CAN_MO253 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[253]))
#define CAN_MO254 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[254]))
#define CAN_MO255 ((CAN_MO_TypeDef *const)&(CAN_MO->MO[255]))
#endif

#endif /* XMC_CAN_MAP_H*/