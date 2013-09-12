/************************************************************************************
 * arch/arm/src/sama5/chip/sam_emac.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_EMAC_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_EMAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* EMAC Register Offsets ************************************************************/

#define SAM_EMAC_NCR_OFFSET       0x0000 /* Network Control Register  */
#define SAM_EMAC_NCFGR_OFFSET     0x0004 /* Network Configuration Register  */
#define SAM_EMAC_NSR_OFFSET       0x0008 /* Network Status Register  */
                                         /* 0x000c-0x0010 Reserved */
#define SAM_EMAC_TSR_OFFSET       0x0014 /* Transmit Status Register  */
#define SAM_EMAC_RBQP_OFFSET      0x0018 /* Receive Buffer Queue Pointer Register  */
#define SAM_EMAC_TBQP_OFFSET      0x001c /* Transmit Buffer Queue Pointer Register  */
#define SAM_EMAC_RSR_OFFSET       0x0020 /* Receive Status Register  */
#define SAM_EMAC_ISR_OFFSET       0x0024 /* Interrupt Status Register  */
#define SAM_EMAC_IER_OFFSET       0x0028 /* Interrupt Enable Register  */
#define SAM_EMAC_IDR_OFFSET       0x002c /* Interrupt Disable Register  */
#define SAM_EMAC_IMR_OFFSET       0x0030 /* Interrupt Mask Register  */
#define SAM_EMAC_MAN_OFFSET       0x0034 /* Phy Maintenance Register  */
#define SAM_EMAC_PTR_OFFSET       0x0038 /* Pause Time Register  */
#define SAM_EMAC_PFR_OFFSET       0x003c /* Pause Frames Received Register  */
#define SAM_EMAC_FTO_OFFSET       0x0040 /* Frames Transmitted Ok Register  */
#define SAM_EMAC_SCF_OFFSET       0x0044 /* Single Collision Frames Register  */
#define SAM_EMAC_MCF_OFFSET       0x0048 /* Multiple Collision Frames Register  */
#define SAM_EMAC_FRO_OFFSET       0x004c /* Frames Received Ok Register  */
#define SAM_EMAC_FCSE_OFFSET      0x0050 /* Frame Check Sequence Errors Register  */
#define SAM_EMAC_ALE_OFFSET       0x0054 /* Alignment Errors Register  */
#define SAM_EMAC_DTF_OFFSET       0x0058 /* Deferred Transmission Frames Register  */
#define SAM_EMAC_LCOL_OFFSET      0x005c /* Late Collisions Register  */
#define SAM_EMAC_ECOL_OFFSET      0x0060 /* Excessive Collisions Register  */
#define SAM_EMAC_TUND_OFFSET      0x0064 /* Transmit Underrun Errors Register  */
#define SAM_EMAC_CSE_OFFSET       0x0068 /* Carrier Sense Errors Register  */
#define SAM_EMAC_RRE_OFFSET       0x006c /* Receive Resource Errors Register  */
#define SAM_EMAC_ROV_OFFSET       0x0070 /* Receive Overrun Errors Register  */
#define SAM_EMAC_RSE_OFFSET       0x0074 /* Receive Symbol Errors Register  */
#define SAM_EMAC_ELE_OFFSET       0x0078 /* Excessive Length Errors Register  */
#define SAM_EMAC_RJA_OFFSET       0x007c /* Receive Jabbers Register  */
#define SAM_EMAC_USF_OFFSET       0x0080 /* Undersize Frames Register  */
#define SAM_EMAC_STE_OFFSET       0x0084 /* SQE Test Errors Register  */
#define SAM_EMAC_RLE_OFFSET       0x0088 /* Received Length Field Mismatch Register  */
#define SAM_EMAC_HRB_OFFSET       0x0090 /* Hash Register Bottom [31:0] Register  */
#define SAM_EMAC_HRT_OFFSET       0x0094 /* Hash Register Top [63:32] Register  */
#define SAM_EMAC_SA1B_OFFSET      0x0098 /* Specific Address 1 Bottom Register  */
#define SAM_EMAC_SA1T_OFFSET      0x009c /* Specific Address 1 Top Register  */
#define SAM_EMAC_SA2B_OFFSET      0x00a0 /* Specific Address 2 Bottom Register  */
#define SAM_EMAC_SA2T_OFFSET      0x00a4 /* Specific Address 2 Top Register  */
#define SAM_EMAC_SA3B_OFFSET      0x00a8 /* Specific Address 3 Bottom Register  */
#define SAM_EMAC_SA3T_OFFSET      0x00ac /* Specific Address 3 Top Register  */
#define SAM_EMAC_SA4B_OFFSET      0x00b0 /* Specific Address 4 Bottom Register  */
#define SAM_EMAC_SA4T_OFFSET      0x00b4 /* Specific Address 4 Top Register  */
#define SAM_EMAC_TID_OFFSET       0x00b8 /* Type ID Checking Register  */
#define SAM_EMAC_USRIO_OFFSET     0x00c0 /* User Input/Output Register  */
#define SAM_EMAC_WOL_OFFSET       0x00c4 /* Wake on LAN Register  */
                                         /* 0x00c8-0x00fc Reserved */

/* EMAC Register Addresses **********************************************************/

#define SAM_EMAC_NCR              (SAM_EMAC_VBASE+SAM_EMAC_NCR_OFFSET)
#define SAM_EMAC_NCFGR            (SAM_EMAC_VBASE+SAM_EMAC_NCFGR_OFFSET)
#define SAM_EMAC_NSR              (SAM_EMAC_VBASE+SAM_EMAC_NSR_OFFSET)
#define SAM_EMAC_TSR              (SAM_EMAC_VBASE+SAM_EMAC_TSR_OFFSET)
#define SAM_EMAC_RBQP             (SAM_EMAC_VBASE+SAM_EMAC_RBQP_OFFSET)
#define SAM_EMAC_TBQP             (SAM_EMAC_VBASE+SAM_EMAC_TBQP_OFFSET)
#define SAM_EMAC_RSR              (SAM_EMAC_VBASE+SAM_EMAC_RSR_OFFSET)
#define SAM_EMAC_ISR              (SAM_EMAC_VBASE+SAM_EMAC_ISR_OFFSET)
#define SAM_EMAC_IER              (SAM_EMAC_VBASE+SAM_EMAC_IER_OFFSET)
#define SAM_EMAC_IDR              (SAM_EMAC_VBASE+SAM_EMAC_IDR_OFFSET)
#define SAM_EMAC_IMR              (SAM_EMAC_VBASE+SAM_EMAC_IMR_OFFSET)
#define SAM_EMAC_MAN              (SAM_EMAC_VBASE+SAM_EMAC_MAN_OFFSET)
#define SAM_EMAC_PTR              (SAM_EMAC_VBASE+SAM_EMAC_PTR_OFFSET)
#define SAM_EMAC_PFR              (SAM_EMAC_VBASE+SAM_EMAC_PFR_OFFSET)
#define SAM_EMAC_FTO              (SAM_EMAC_VBASE+SAM_EMAC_FTO_OFFSET)
#define SAM_EMAC_SCF              (SAM_EMAC_VBASE+SAM_EMAC_SCF_OFFSET)
#define SAM_EMAC_MCF              (SAM_EMAC_VBASE+SAM_EMAC_MCF_OFFSET)
#define SAM_EMAC_FRO              (SAM_EMAC_VBASE+SAM_EMAC_FRO_OFFSET)
#define SAM_EMAC_FCSE             (SAM_EMAC_VBASE+SAM_EMAC_FCSE_OFFSET)
#define SAM_EMAC_ALE              (SAM_EMAC_VBASE+SAM_EMAC_ALE_OFFSET)
#define SAM_EMAC_DTF              (SAM_EMAC_VBASE+SAM_EMAC_DTF_OFFSET)
#define SAM_EMAC_LCOL             (SAM_EMAC_VBASE+SAM_EMAC_LCOL_OFFSET)
#define SAM_EMAC_ECOL             (SAM_EMAC_VBASE+SAM_EMAC_ECOL_OFFSET)
#define SAM_EMAC_TUND             (SAM_EMAC_VBASE+SAM_EMAC_TUND_OFFSET)
#define SAM_EMAC_CSE              (SAM_EMAC_VBASE+SAM_EMAC_CSE_OFFSET)
#define SAM_EMAC_RRE              (SAM_EMAC_VBASE+SAM_EMAC_RRE_OFFSET)
#define SAM_EMAC_ROV              (SAM_EMAC_VBASE+SAM_EMAC_ROV_OFFSET)
#define SAM_EMAC_RSE              (SAM_EMAC_VBASE+SAM_EMAC_RSE_OFFSET)
#define SAM_EMAC_ELE              (SAM_EMAC_VBASE+SAM_EMAC_ELE_OFFSET)
#define SAM_EMAC_RJA              (SAM_EMAC_VBASE+SAM_EMAC_RJA_OFFSET)
#define SAM_EMAC_USF              (SAM_EMAC_VBASE+SAM_EMAC_USF_OFFSET)
#define SAM_EMAC_STE              (SAM_EMAC_VBASE+SAM_EMAC_STE_OFFSET)
#define SAM_EMAC_RLE              (SAM_EMAC_VBASE+SAM_EMAC_RLE_OFFSET)
#define SAM_EMAC_HRB              (SAM_EMAC_VBASE+SAM_EMAC_HRB_OFFSET)
#define SAM_EMAC_HRT              (SAM_EMAC_VBASE+SAM_EMAC_HRT_OFFSET)
#define SAM_EMAC_SA1B             (SAM_EMAC_VBASE+SAM_EMAC_SA1B_OFFSET)
#define SAM_EMAC_SA1T             (SAM_EMAC_VBASE+SAM_EMAC_SA1T_OFFSET)
#define SAM_EMAC_SA2B             (SAM_EMAC_VBASE+SAM_EMAC_SA2B_OFFSET)
#define SAM_EMAC_SA2T             (SAM_EMAC_VBASE+SAM_EMAC_SA2T_OFFSET)
#define SAM_EMAC_SA3B             (SAM_EMAC_VBASE+SAM_EMAC_SA3B_OFFSET)
#define SAM_EMAC_SA3T             (SAM_EMAC_VBASE+SAM_EMAC_SA3T_OFFSET)
#define SAM_EMAC_SA4B             (SAM_EMAC_VBASE+SAM_EMAC_SA4B_OFFSET)
#define SAM_EMAC_SA4T             (SAM_EMAC_VBASE+SAM_EMAC_SA4T_OFFSET)
#define SAM_EMAC_TID              (SAM_EMAC_VBASE+SAM_EMAC_TID_OFFSET)
#define SAM_EMAC_USRIO            (SAM_EMAC_VBASE+SAM_EMAC_USRIO_OFFSET)
#define SAM_EMAC_WOL              (SAM_EMAC_VBASE+SAM_EMAC_WOL_OFFSET)

/* EMAC Register Bit Definitions ****************************************************/

/* Network Control Register  */
#define EMAC_NCR_
/* Network Configuration Register  */
#define EMAC_NCFGR_
/* Network Status Register  */
#define EMAC_NSR_
/* Transmit Status Register  */
#define EMAC_TSR_
/* Receive Buffer Queue Pointer Register  */
#define EMAC_RBQP_
/* Transmit Buffer Queue Pointer Register  */
#define EMAC_TBQP_
/* Receive Status Register  */
#define EMAC_RSR_
/* Interrupt Status Register  */
#define EMAC_ISR_
/* Interrupt Enable Register  */
#define EMAC_IER_
/* Interrupt Disable Register  */
#define EMAC_IDR_
/* Interrupt Mask Register  */
#define EMAC_IMR_
/* Phy Maintenance Register  */
#define EMAC_MAN_
/* Pause Time Register  */
#define EMAC_PTR_
/* Pause Frames Received Register  */
#define EMAC_PFR_
/* Frames Transmitted Ok Register  */
#define EMAC_FTO_
/* Single Collision Frames Register  */
#define EMAC_SCF_
/* Multiple Collision Frames Register  */
#define EMAC_MCF_
/* Frames Received Ok Register  */
#define EMAC_FRO_
/* Frame Check Sequence Errors Register  */
#define EMAC_FCSE_
/* Alignment Errors Register  */
#define EMAC_ALE_
/* Deferred Transmission Frames Register  */
#define EMAC_DTF_
/* Late Collisions Register  */
#define EMAC_LCOL_
/* Excessive Collisions Register  */
#define EMAC_ECOL_
/* Transmit Underrun Errors Register  */
#define EMAC_TUND_
/* Carrier Sense Errors Register  */
#define EMAC_CSE_
/* Receive Resource Errors Register  */
#define EMAC_RRE_
/* Receive Overrun Errors Register  */
#define EMAC_ROV_
/* Receive Symbol Errors Register  */
#define EMAC_RSE_
/* Excessive Length Errors Register  */
#define EMAC_ELE_
/* Receive Jabbers Register  */
#define EMAC_RJA_
/* Undersize Frames Register  */
#define EMAC_USF_
/* SQE Test Errors Register  */
#define EMAC_STE_
/* Received Length Field Mismatch Register  */
#define EMAC_RLE_
/* Hash Register Bottom [31:0] Register  */
#define EMAC_HRB_
/* Hash Register Top [63:32] Register  */
#define EMAC_HRT_
/* Specific Address 1 Bottom Register  */
#define EMAC_SA1B_
/* Specific Address 1 Top Register  */
#define EMAC_SA1T_
/* Specific Address 2 Bottom Register  */
#define EMAC_SA2B_
/* Specific Address 2 Top Register  */
#define EMAC_SA2T_
/* Specific Address 3 Bottom Register  */
#define EMAC_SA3B_
/* Specific Address 3 Top Register  */
#define EMAC_SA3T_
/* Specific Address 4 Bottom Register  */
#define EMAC_SA4B_
/* Specific Address 4 Top Register  */
#define EMAC_SA4T_
/* Type ID Checking Register  */
#define EMAC_TID_
/* User Input/Output Register  */
#define EMAC_USRIO_
/* Wake on LAN Register  */
#define EMAC_WOL_

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_EMAC_H */
