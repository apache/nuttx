/************************************************************************************
 * arch/arm/src/sama5/chip/sam_gmac.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_GMAC_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_GMAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* GMAC Register Offsets ************************************************************/

#define SAM_GMAC_NCR_OFFSET        0x0000 /* Network Control Register */
#define SAM_GMAC_NCFGR_OFFSET      0x0004 /* Network Configuration Register */
#define SAM_GMAC_NSR_OFFSET        0x0008 /* Network Status Register */
#define SAM_GMAC_UR_OFFSET         0x000c /* User Register */
#define SAM_GMAC_DCFGR_OFFSET      0x0010 /* DMA Configuration Register */
#define SAM_GMAC_TSR_OFFSET        0x0014 /* Transmit Status Register */
#define SAM_GMAC_RBQB_OFFSET       0x0018 /* Receive Buffer Queue Base Address */
#define SAM_GMAC_TBQB_OFFSET       0x001c /* Transmit Buffer Queue Base Address */
#define SAM_GMAC_RSR_OFFSET        0x0020 /* Receive Status Register */
#define SAM_GMAC_ISR_OFFSET        0x0024 /* Interrupt Status Register */
#define SAM_GMAC_IER_OFFSET        0x0028 /* Interrupt Enable Register */
#define SAM_GMAC_IDR_OFFSET        0x002c /* Interrupt Disable Register */
#define SAM_GMAC_IMR_OFFSET        0x0030 /* Interrupt Mask Register */
#define SAM_GMAC_MAN_OFFSET        0x0034 /* PHY Maintenance Register */
#define SAM_GMAC_RPQ_OFFSET        0x0038 /* Received Pause Quantum Register */
#define SAM_GMAC_TPQ_OFFSET        0x003c /* Transmit Pause Quantum Register */
#define SAM_GMAC_TPSF_OFFSET       0x0040 /* TX Partial Store and Forward Register */
#define SAM_GMAC_RPSF_OFFSET       0x0044 /* RX Partial Store and Forward Register */
                                          /* 0x0048-0x007c Reserved */
#define SAM_GMAC_HRB_OFFSET        0x0080 /* Hash Register Bottom [31:0] */
#define SAM_GMAC_HRT_OFFSET        0x0084 /* Hash Register Top [63:32] */
#define SAM_GMAC_SAB_OFFSET(n)     (0x0088 + (((n)-1) << 3))  /* n=1..4 */
#define SAM_GMAC_SAT_OFFSET(n)     (0x008c + (((n)-1) << 3))  /* n=1..4 */
#define SAM_GMAC_SAB1_OFFSET       0x0088 /* Specific Address 1 Bottom [31:0] Register */
#define SAM_GMAC_SAT1_OFFSET       0x008c /* Specific Address 1 Top [47:32] Register */
#define SAM_GMAC_SAB2_OFFSET       0x0090 /* Specific Address 2 Bottom [31:0] Register */
#define SAM_GMAC_SAT2_OFFSET       0x0094 /* Specific Address 2 Top [47:32] Register */
#define SAM_GMAC_SAB3_OFFSET       0x0098 /* Specific Address 3 Bottom [31:0] Register */
#define SAM_GMAC_SAT3_OFFSET       0x009c /* Specific Address 3 Top [47:32] Register */
#define SAM_GMAC_SAB4_OFFSET       0x00a0 /* Specific Address 4 Bottom [31:0] Register */
#define SAM_GMAC_SAT4_OFFSET       0x00a4 /* Specific Address 4 Top [47:32] Register */
#define SAM_GMAC_TIDM_OFFSET(n)    (0x00a8 + (((n)-1) << 2))  /* n=1..4 */
#define SAM_GMAC_TIDM1_OFFSET      0x00a8 /* Type ID Match 1 Register */
#define SAM_GMAC_TIDM2_OFFSET      0x00ac /* Type ID Match 2 Register */
#define SAM_GMAC_TIDM3_OFFSET      0x00b0 /* Type ID Match 3 Register */
#define SAM_GMAC_TIDM4_OFFSET      0x00b4 /* Type ID Match 4 Register */
#define SAM_GMAC_WOL_OFFSET        0x00b8 /* Wake on LAN Register */
#define SAM_GMAC_IPGS_OFFSET       0x00bc /* IPG Stretch Register */
#define SAM_GMAC_SVLAN_OFFSET      0x00c0 /* Stacked VLAN Register */
#define SAM_GMAC_TPFCP_OFFSET      0x00c4 /* Transmit PFC Pause Register */
#define SAM_GMAC_SAMB1_OFFSET      0x00c8 /* Specific Address 1 Mask Bottom [31:0] Register */
#define SAM_GMAC_SAMT1_OFFSET      0x00cc /* Specific Address 1 Mask Top [47:32] Register */
                                          /* 0x00fc Reserved */
#define SAM_GMAC_OTLO_OFFSET       0x0100 /* Octets Transmitted [31:0] Register */
#define SAM_GMAC_OTHI_OFFSET       0x0104 /* Octets Transmitted [47:32] Register */
#define SAM_GMAC_FT_OFFSET         0x0108 /* Frames Transmitted Register */
#define SAM_GMAC_BCFT_OFFSET       0x010c /* Broadcast Frames Transmitted Register */
#define SAM_GMAC_MFT_OFFSET        0x0110 /* Multicast Frames Transmitted Register */
#define SAM_GMAC_PFT_OFFSET        0x0114 /* Pause Frames Transmitted Register */
#define SAM_GMAC_BFT64_OFFSET      0x0118 /* 64 Byte Frames Transmitted Register */
#define SAM_GMAC_TBFT127_OFFSET    0x011c /* 65 to 127 Byte Frames Transmitted Register */
#define SAM_GMAC_TBFT255_OFFSET    0x0120 /* 128 to 255 Byte Frames Transmitted Register */
#define SAM_GMAC_TBFT511_OFFSET    0x0124 /* 256 to 511 Byte Frames Transmitted Register */
#define SAM_GMAC_TBFT1023_OFFSET   0x0128 /* 512 to 1023 Byte Frames Transmitted Register */
#define SAM_GMAC_TBFT1518_OFFSET   0x012c /* 1024 to 1518 Byte Frames Transmitted Register */
#define SAM_GMAC_GTBFT1518_OFFSET  0x0130 /* Greater Than 1518 Byte Frames Transmitted Register */
#define SAM_GMAC_TUR_OFFSET        0x0134 /* Transmit Under Runs Register */
#define SAM_GMAC_SCF_OFFSET        0x0138 /* Single Collision Frames Register */
#define SAM_GMAC_MCF_OFFSET        0x013c /* Multiple Collision Frames Register */
#define SAM_GMAC_EC_OFFSET         0x0140 /* Excessive Collisions Register */
#define SAM_GMAC_LC_OFFSET         0x0144 /* Late Collisions Register */
#define SAM_GMAC_DTF_OFFSET        0x0148 /* Deferred Transmission Frames Register */
#define SAM_GMAC_CSE_OFFSET        0x014c /* Carrier Sense Errors Register */
#define SAM_GMAC_ORLO_OFFSET       0x0150 /* Octets Received [31:0] Received */
#define SAM_GMAC_ORHI_OFFSET       0x0154 /* Octets Received [47:32] Received */
#define SAM_GMAC_FR_OFFSET         0x0158 /* Frames Received Register */
#define SAM_GMAC_BCFR_OFFSET       0x015c /* Broadcast Frames Received Register */
#define SAM_GMAC_MFR_OFFSET        0x0160 /* Multicast Frames Received Register */
#define SAM_GMAC_PFR_OFFSET        0x0164 /* Pause Frames Received Register */
#define SAM_GMAC_BFR64_OFFSET      0x0168 /* 64 Byte Frames Received Register */
#define SAM_GMAC_TBFR127_OFFSET    0x016c /* 65 to 127 Byte Frames Received Register */
#define SAM_GMAC_TBFR255_OFFSET    0x0170 /* 128 to 255 Byte Frames Received Register */
#define SAM_GMAC_TBFR511_OFFSET    0x0174 /* 256 to 511Byte Frames Received Register */
#define SAM_GMAC_TBFR1023_OFFSET   0x0178 /* 512 to 1023 Byte Frames Received Register */
#define SAM_GMAC_TBFR1518_OFFSET   0x017c /* 1024 to 1518 Byte Frames Received Register */
#define SAM_GMAC_TMXBFR_OFFSET     0x0180 /* 1519 to Maximum Byte Frames Received Register */
#define SAM_GMAC_UFR_OFFSET        0x0184 /* Undersize Frames Received Register */
#define SAM_GMAC_OFR_OFFSET        0x0188 /* Oversize Frames Received Register */
#define SAM_GMAC_JR_OFFSET         0x018c /* Jabbers Received Register */
#define SAM_GMAC_FCSE_OFFSET       0x0190 /* Frame Check Sequence Errors Register */
#define SAM_GMAC_LFFE_OFFSET       0x0194 /* Length Field Frame Errors Register */
#define SAM_GMAC_RSE_OFFSET        0x0198 /* Receive Symbol Errors Register */
#define SAM_GMAC_AE_OFFSET         0x019c /* Alignment Errors Register */
#define SAM_GMAC_RRE_OFFSET        0x01a0 /* Receive Resource Errors Register */
#define SAM_GMAC_ROE_OFFSET        0x01a4 /* Receive Overrun Register */
#define SAM_GMAC_IHCE_OFFSET       0x01a8 /* IP Header Checksum Errors Register */
#define SAM_GMAC_TCE_OFFSET        0x01ac /* TCP Checksum Errors Register */
#define SAM_GMAC_UCE_OFFSET        0x01b0 /* UDP Checksum Errors Register */
#define SAM_GMAC_TSSS_OFFSET       0x01c8 /* 1588 Timer Sync Strobe Seconds Register */
#define SAM_GMAC_TSSN_OFFSET       0x01cc /* 1588 Timer Sync Strobe Nanoseconds Register */
#define SAM_GMAC_TS_OFFSET         0x01d0 /* 1588 Timer Seconds Register */
#define SAM_GMAC_TN_OFFSET         0x01d4 /* 1588 Timer Nanoseconds Register */
#define SAM_GMAC_TA_OFFSET         0x01d8 /* 1588 Timer Adjust Register */
#define SAM_GMAC_TI_OFFSET         0x01dc /* 1588 Timer Increment Register */
#define SAM_GMAC_EFTS_OFFSET       0x01e0 /* PTP Event Frame Transmitted Seconds */
#define SAM_GMAC_EFTN_OFFSET       0x01e4 /* PTP Event Frame Transmitted Nanoseconds */
#define SAM_GMAC_EFRS_OFFSET       0x01e8 /* PTP Event Frame Received Seconds */
#define SAM_GMAC_EFRN_OFFSET       0x01ec /* PTP Event Frame Received Nanoseconds */
#define SAM_GMAC_PEFTS_OFFSET      0x01f0 /* PTP Peer Event Frame Transmitted Seconds */
#define SAM_GMAC_PEFTN_OFFSET      0x01f4 /* PTP Peer Event Frame Transmitted Nanoseconds */
#define SAM_GMAC_PEFRS_OFFSET      0x01f8 /* PTP Peer Event Frame ReceiveGMAC_PEFRNd Seconds */
#define SAM_GMAC_PEFRS_OFFSET      0x01fc /* PTP Peer Event Frame Received Nanoseconds */
                                          /* 0x0200-0x023c Reserved */
                                          /* 0x0280-0x0298 Reserved */
#define SAM_GMAC_ISRPQ_OFFSET(n)   (0x400 + ((n) << 2))  /* n=0..6 */
#define SAM_GMAC_ISRPQ0_OFFSET     0x400 /* Interrupt Status Register Priority Queue 0 */
#define SAM_GMAC_ISRPQ1_OFFSET     0x404 /* Interrupt Status Register Priority Queue 1 */
#define SAM_GMAC_ISRPQ2_OFFSET     0x408 /* Interrupt Status Register Priority Queue 2 */
#define SAM_GMAC_ISRPQ3_OFFSET     0x40c /* Interrupt Status Register Priority Queue 3 */
#define SAM_GMAC_ISRPQ4_OFFSET     0x410 /* Interrupt Status Register Priority Queue 4 */
#define SAM_GMAC_ISRPQ5_OFFSET     0x414 /* Interrupt Status Register Priority Queue 5 */
#define SAM_GMAC_ISRPQ6_OFFSET     0x418 /* Interrupt Status Register Priority Queue 6 */
#define SAM_GMAC_TBQBAPQ_OFFSET(n) (0x440 + ((n) << 2))  /* n=0..6 */
#define SAM_GMAC_TBQBAPQ0_OFFSET   0x440 /* Transmit Buffer Queue Base Address Priority Queue 0 */
#define SAM_GMAC_TBQBAPQ1_OFFSET   0x444 /* Transmit Buffer Queue Base Address Priority Queue 1 */
#define SAM_GMAC_TBQBAPQ2_OFFSET   0x448 /* Transmit Buffer Queue Base Address Priority Queue 2 */
#define SAM_GMAC_TBQBAPQ3_OFFSET   0x44c /* Transmit Buffer Queue Base Address Priority Queue 3 */
#define SAM_GMAC_TBQBAPQ4_OFFSET   0x450 /* Transmit Buffer Queue Base Address Priority Queue 4 */
#define SAM_GMAC_TBQBAPQ5_OFFSET   0x454 /* Transmit Buffer Queue Base Address Priority Queue 5 */
#define SAM_GMAC_TBQBAPQ6_OFFSET   0x458 /* Transmit Buffer Queue Base Address Priority Queue 6 */
#define SAM_GMAC_RBQBAPQ_OFFSET(n) (0x480 + ((n) << 2))  /* n=0..6 */
#define SAM_GMAC_RBQBAPQ0_OFFSET   0x480 /* Receive Buffer Queue Base Address Priority Queue 0 */
#define SAM_GMAC_RBQBAPQ1_OFFSET   0x484 /* Receive Buffer Queue Base Address Priority Queue 1 */
#define SAM_GMAC_RBQBAPQ2_OFFSET   0x488 /* Receive Buffer Queue Base Address Priority Queue 2 */
#define SAM_GMAC_RBQBAPQ3_OFFSET   0x48c /* Receive Buffer Queue Base Address Priority Queue 3 */
#define SAM_GMAC_RBQBAPQ4_OFFSET   0x490 /* Receive Buffer Queue Base Address Priority Queue 4 */
#define SAM_GMAC_RBQBAPQ5_OFFSET   0x494 /* Receive Buffer Queue Base Address Priority Queue 5 */
#define SAM_GMAC_RBQBAPQ6_OFFSET   0x498 /* Receive Buffer Queue Base Address Priority Queue 6 */
#define SAM_GMAC_RBSRPQ_OFFSET(n)  (0x4a0 + ((n) << 2))  /* n=0..6 */
#define SAM_GMAC_RBSRPQ0_OFFSET    0x4a0 /* Receive Buffer Size Register Priority Queue 0 */
#define SAM_GMAC_RBSRPQ1_OFFSET    0x4a4 /* Receive Buffer Size Register Priority Queue 1 */
#define SAM_GMAC_RBSRPQ2_OFFSET    0x4a8 /* Receive Buffer Size Register Priority Queue 2 */
#define SAM_GMAC_RBSRPQ3_OFFSET    0x4ac /* Receive Buffer Size Register Priority Queue 3 */
#define SAM_GMAC_RBSRPQ4_OFFSET    0x4b0 /* Receive Buffer Size Register Priority Queue 4 */
#define SAM_GMAC_RBSRPQ5_OFFSET    0x4b4 /* Receive Buffer Size Register Priority Queue 5 */
#define SAM_GMAC_RBSRPQ6_OFFSET    0x4b8 /* Receive Buffer Size Register Priority Queue 6 */
#define SAM_GMAC_ST1RPQ_OFFSET(n)  (0x500 + ((n) << 2))  /* n=0..15 */
#define SAM_GMAC_ST1RPQ0_OFFSET    0x500 /* Screening Type1 Register Priority Queue 0 */
#define SAM_GMAC_ST1RPQ1_OFFSET    0x504 /* Screening Type1 Register Priority Queue 1 */
#define SAM_GMAC_ST1RPQ2_OFFSET    0x508 /* Screening Type1 Register Priority Queue 2 */
#define SAM_GMAC_ST1RPQ3_OFFSET    0x50c /* Screening Type1 Register Priority Queue 3 */
#define SAM_GMAC_ST1RPQ4_OFFSET    0x510 /* Screening Type1 Register Priority Queue 4 */
#define SAM_GMAC_ST1RPQ5_OFFSET    0x514 /* Screening Type1 Register Priority Queue 5 */
#define SAM_GMAC_ST1RPQ6_OFFSET    0x518 /* Screening Type1 Register Priority Queue 6 */
#define SAM_GMAC_ST1RPQ7_OFFSET    0x51c /* Screening Type1 Register Priority Queue 7 */
#define SAM_GMAC_ST1RPQ8_OFFSET    0x520 /* Screening Type1 Register Priority Queue 8 */
#define SAM_GMAC_ST1RPQ9_OFFSET    0x524 /* Screening Type1 Register Priority Queue 9 */
#define SAM_GMAC_ST1RPQ10_OFFSET   0x528 /* Screening Type1 Register Priority Queue 10 */
#define SAM_GMAC_ST1RPQ11_OFFSET   0x52c /* Screening Type1 Register Priority Queue 11 */
#define SAM_GMAC_ST1RPQ12_OFFSET   0x530 /* Screening Type1 Register Priority Queue 12 */
#define SAM_GMAC_ST1RPQ13_OFFSET   0x534 /* Screening Type1 Register Priority Queue 13 */
#define SAM_GMAC_ST1RPQ14_OFFSET   0x538 /* Screening Type1 Register Priority Queue 14 */
#define SAM_GMAC_ST1RPQ15_OFFSET   0x53c /* Screening Type1 Register Priority Queue 15 */
#define SAM_GMAC_ST2RPQ_OFFSET(n)  (0x540 + ((n) << 2))  /* n=0..15 */
#define SAM_GMAC_ST2RPQ0_OFFSET    0x540 /* Screening Type2 Register Priority Queue 0 */
#define SAM_GMAC_ST2RPQ1_OFFSET    0x544 /* Screening Type2 Register Priority Queue 1 */
#define SAM_GMAC_ST2RPQ2_OFFSET    0x548 /* Screening Type2 Register Priority Queue 2 */
#define SAM_GMAC_ST2RPQ3_OFFSET    0x54c /* Screening Type2 Register Priority Queue 3 */
#define SAM_GMAC_ST2RPQ4_OFFSET    0x550 /* Screening Type2 Register Priority Queue 4 */
#define SAM_GMAC_ST2RPQ5_OFFSET    0x554 /* Screening Type2 Register Priority Queue 5 */
#define SAM_GMAC_ST2RPQ6_OFFSET    0x558 /* Screening Type2 Register Priority Queue 6 */
#define SAM_GMAC_ST2RPQ7_OFFSET    0x55c /* Screening Type2 Register Priority Queue 7 */
#define SAM_GMAC_ST2RPQ8_OFFSET    0x560 /* Screening Type2 Register Priority Queue 8 */
#define SAM_GMAC_ST2RPQ9_OFFSET    0x564 /* Screening Type2 Register Priority Queue 9 */
#define SAM_GMAC_ST2RPQ10_OFFSET   0x568 /* Screening Type2 Register Priority Queue 10 */
#define SAM_GMAC_ST2RPQ11_OFFSET   0x56c /* Screening Type2 Register Priority Queue 11 */
#define SAM_GMAC_ST2RPQ12_OFFSET   0x570 /* Screening Type2 Register Priority Queue 12 */
#define SAM_GMAC_ST2RPQ13_OFFSET   0x574 /* Screening Type2 Register Priority Queue 13 */
#define SAM_GMAC_ST2RPQ14_OFFSET   0x578 /* Screening Type2 Register Priority Queue 14 */
#define SAM_GMAC_ST2RPQ15_OFFSET   0x57c /* Screening Type2 Register Priority Queue 15 */
#define SAM_GMAC_IERPQ_OFFSET(n)   (0x600 + ((n) << 2))  /* n=0..6 */
#define SAM_GMAC_IERPQ0_OFFSET     0x600 /* Interrupt Enable Register Priority Queue 0 */
#define SAM_GMAC_IERPQ1_OFFSET     0x604 /* Interrupt Enable Register Priority Queue 1 */
#define SAM_GMAC_IERPQ2_OFFSET     0x608 /* Interrupt Enable Register Priority Queue 2 */
#define SAM_GMAC_IERPQ3_OFFSET     0x60c /* Interrupt Enable Register Priority Queue 3 */
#define SAM_GMAC_IERPQ4_OFFSET     0x610 /* Interrupt Enable Register Priority Queue 4 */
#define SAM_GMAC_IERPQ5_OFFSET     0x614 /* Interrupt Enable Register Priority Queue 5 */
#define SAM_GMAC_IERPQ6_OFFSET     0x618 /* Interrupt Enable Register Priority Queue 6 */
#define SAM_GMAC_IDRPQ_OFFSET(n)   (0x620 + ((n) << 2))  /* n=0..6 */
#define SAM_GMAC_IDRPQ0_OFFSET     0x620 /* Interrupt Disable Register Priority Queue 0 */
#define SAM_GMAC_IDRPQ1_OFFSET     0x624 /* Interrupt Disable Register Priority Queue 1 */
#define SAM_GMAC_IDRPQ2_OFFSET     0x628 /* Interrupt Disable Register Priority Queue 2 */
#define SAM_GMAC_IDRPQ3_OFFSET     0x62c /* Interrupt Disable Register Priority Queue 3 */
#define SAM_GMAC_IDRPQ4_OFFSET     0x630 /* Interrupt Disable Register Priority Queue 4 */
#define SAM_GMAC_IDRPQ5_OFFSET     0x630 /* Interrupt Disable Register Priority Queue 5 */
#define SAM_GMAC_IDRPQ6_OFFSET     0x638 /* Interrupt Disable Register Priority Queue 6 */
#define SAM_GMAC_IMRPQ_OFFSET(n)   (0x640 + ((n) << 2))  /* n=0..6 */
#define SAM_GMAC_IMRPQ0_OFFSET     0x640 /* Interrupt Mask Register Priority Queue 0 */
#define SAM_GMAC_IMRPQ1_OFFSET     0x644 /* Interrupt Mask Register Priority Queue 1 */
#define SAM_GMAC_IMRPQ2_OFFSET     0x648 /* Interrupt Mask Register Priority Queue 2 */
#define SAM_GMAC_IMRPQ3_OFFSET     0x64c /* Interrupt Mask Register Priority Queue 3 */
#define SAM_GMAC_IMRPQ4_OFFSET     0x650 /* Interrupt Mask Register Priority Queue 4 */
#define SAM_GMAC_IMRPQ5_OFFSET     0x654 /* Interrupt Mask Register Priority Queue 5 */
#define SAM_GMAC_IMRPQ6_OFFSET     0x658 /* Interrupt Mask Register Priority Queue 6 */

/* GMAC Register Addresses *********************************************************/

#define SAM_GMAC_NCR               (SAM_GMAC_VBASE+SAM_GMAC_NCR_OFFSET)
#define SAM_GMAC_NCFGR             (SAM_GMAC_VBASE+SAM_GMAC_NCFGR_OFFSET)
#define SAM_GMAC_NSR               (SAM_GMAC_VBASE+SAM_GMAC_NSR_OFFSET)
#define SAM_GMAC_UR                (SAM_GMAC_VBASE+SAM_GMAC_UR_OFFSET)
#define SAM_GMAC_DCFGR             (SAM_GMAC_VBASE+SAM_GMAC_DCFGR_OFFSET)
#define SAM_GMAC_TSR               (SAM_GMAC_VBASE+SAM_GMAC_TSR_OFFSET)
#define SAM_GMAC_RBQB              (SAM_GMAC_VBASE+SAM_GMAC_RBQB_OFFSET)
#define SAM_GMAC_TBQB              (SAM_GMAC_VBASE+SAM_GMAC_TBQB_OFFSET)
#define SAM_GMAC_RSR               (SAM_GMAC_VBASE+SAM_GMAC_RSR_OFFSET)
#define SAM_GMAC_ISR               (SAM_GMAC_VBASE+SAM_GMAC_ISR_OFFSET)
#define SAM_GMAC_IER               (SAM_GMAC_VBASE+SAM_GMAC_IER_OFFSET)
#define SAM_GMAC_IDR               (SAM_GMAC_VBASE+SAM_GMAC_IDR_OFFSET)
#define SAM_GMAC_IMR               (SAM_GMAC_VBASE+SAM_GMAC_IMR_OFFSET)
#define SAM_GMAC_MAN               (SAM_GMAC_VBASE+SAM_GMAC_MAN_OFFSET)
#define SAM_GMAC_RPQ               (SAM_GMAC_VBASE+SAM_GMAC_RPQ_OFFSET)
#define SAM_GMAC_TPQ               (SAM_GMAC_VBASE+SAM_GMAC_TPQ_OFFSET)
#define SAM_GMAC_TPSF              (SAM_GMAC_VBASE+SAM_GMAC_TPSF_OFFSET)
#define SAM_GMAC_RPSF              (SAM_GMAC_VBASE+SAM_GMAC_RPSF_OFFSET)
#define SAM_GMAC_HRB               (SAM_GMAC_VBASE+SAM_GMAC_HRB_OFFSET)
#define SAM_GMAC_HRT               (SAM_GMAC_VBASE+SAM_GMAC_HRT_OFFSET)
#define SAM_GMAC_SAB(n)            (SAM_GMAC_VBASE+SAM_GMAC_SAB_OFFSET(n))
#define SAM_GMAC_SAT(n)            (SAM_GMAC_VBASE+SAM_GMAC_SAT_OFFSET(n))
#define SAM_GMAC_SAB1              (SAM_GMAC_VBASE+SAM_GMAC_SAB1_OFFSET)
#define SAM_GMAC_SAT1              (SAM_GMAC_VBASE+SAM_GMAC_SAT1_OFFSET)
#define SAM_GMAC_SAB2              (SAM_GMAC_VBASE+SAM_GMAC_SAB2_OFFSET)
#define SAM_GMAC_SAT2              (SAM_GMAC_VBASE+SAM_GMAC_SAT2_OFFSET)
#define SAM_GMAC_SAB3              (SAM_GMAC_VBASE+SAM_GMAC_SAB3_OFFSET)
#define SAM_GMAC_SAT3              (SAM_GMAC_VBASE+SAM_GMAC_SAT3_OFFSET)
#define SAM_GMAC_SAB4              (SAM_GMAC_VBASE+SAM_GMAC_SAB4_OFFSET)
#define SAM_GMAC_SAT4              (SAM_GMAC_VBASE+SAM_GMAC_SAT4_OFFSET)
#define SAM_GMAC_TIDM(n)           (SAM_GMAC_VBASE+SAM_GMAC_TIDM_OFFSET(n))
#define SAM_GMAC_TIDM1             (SAM_GMAC_VBASE+SAM_GMAC_TIDM1_OFFSET)
#define SAM_GMAC_TIDM2             (SAM_GMAC_VBASE+SAM_GMAC_TIDM2_OFFSET)
#define SAM_GMAC_TIDM3             (SAM_GMAC_VBASE+SAM_GMAC_TIDM3_OFFSET)
#define SAM_GMAC_TIDM4             (SAM_GMAC_VBASE+SAM_GMAC_TIDM4_OFFSET)
#define SAM_GMAC_WOL               (SAM_GMAC_VBASE+SAM_GMAC_WOL_OFFSET)
#define SAM_GMAC_IPGS              (SAM_GMAC_VBASE+SAM_GMAC_IPGS_OFFSET)
#define SAM_GMAC_SVLAN             (SAM_GMAC_VBASE+SAM_GMAC_SVLAN_OFFSET)
#define SAM_GMAC_TPFCP             (SAM_GMAC_VBASE+SAM_GMAC_TPFCP_OFFSET)
#define SAM_GMAC_SAMB1             (SAM_GMAC_VBASE+SAM_GMAC_SAMB1_OFFSET)
#define SAM_GMAC_SAMT1             (SAM_GMAC_VBASE+SAM_GMAC_SAMT1_OFFSET)
#define SAM_GMAC_OTLO              (SAM_GMAC_VBASE+SAM_GMAC_OTLO_OFFSET)
#define SAM_GMAC_OTHI              (SAM_GMAC_VBASE+SAM_GMAC_OTHI_OFFSET)
#define SAM_GMAC_FT                (SAM_GMAC_VBASE+SAM_GMAC_FT_OFFSET)
#define SAM_GMAC_BCFT              (SAM_GMAC_VBASE+SAM_GMAC_BCFT_OFFSET)
#define SAM_GMAC_MFT               (SAM_GMAC_VBASE+SAM_GMAC_MFT_OFFSET)
#define SAM_GMAC_PFT               (SAM_GMAC_VBASE+SAM_GMAC_PFT_OFFSET)
#define SAM_GMAC_BFT64             (SAM_GMAC_VBASE+SAM_GMAC_BFT64_OFFSET)
#define SAM_GMAC_TBFT127           (SAM_GMAC_VBASE+SAM_GMAC_TBFT127_OFFSET)
#define SAM_GMAC_TBFT255           (SAM_GMAC_VBASE+SAM_GMAC_TBFT255_OFFSET)
#define SAM_GMAC_TBFT511           (SAM_GMAC_VBASE+SAM_GMAC_TBFT511_OFFSET)
#define SAM_GMAC_TBFT1023          (SAM_GMAC_VBASE+SAM_GMAC_TBFT1023_OFFSET)
#define SAM_GMAC_TBFT1518          (SAM_GMAC_VBASE+SAM_GMAC_TBFT1518_OFFSET)
#define SAM_GMAC_GTBFT1518         (SAM_GMAC_VBASE+SAM_GMAC_GTBFT1518_OFFSET)
#define SAM_GMAC_TUR               (SAM_GMAC_VBASE+SAM_GMAC_TUR_OFFSET)
#define SAM_GMAC_SCF               (SAM_GMAC_VBASE+SAM_GMAC_SCF_OFFSET)
#define SAM_GMAC_MCF               (SAM_GMAC_VBASE+SAM_GMAC_MCF_OFFSET)
#define SAM_GMAC_EC                (SAM_GMAC_VBASE+SAM_GMAC_EC_OFFSET)
#define SAM_GMAC_LC                (SAM_GMAC_VBASE+SAM_GMAC_LC_OFFSET)
#define SAM_GMAC_DTF               (SAM_GMAC_VBASE+SAM_GMAC_DTF_OFFSET)
#define SAM_GMAC_CSE               (SAM_GMAC_VBASE+SAM_GMAC_CSE_OFFSET)
#define SAM_GMAC_ORLO              (SAM_GMAC_VBASE+SAM_GMAC_ORLO_OFFSET)
#define SAM_GMAC_ORHI              (SAM_GMAC_VBASE+SAM_GMAC_ORHI_OFFSET)
#define SAM_GMAC_FR                (SAM_GMAC_VBASE+SAM_GMAC_FR_OFFSET)
#define SAM_GMAC_BCFR              (SAM_GMAC_VBASE+SAM_GMAC_BCFR_OFFSET)
#define SAM_GMAC_MFR               (SAM_GMAC_VBASE+SAM_GMAC_MFR_OFFSET)
#define SAM_GMAC_PFR               (SAM_GMAC_VBASE+SAM_GMAC_PFR_OFFSET)
#define SAM_GMAC_BFR64             (SAM_GMAC_VBASE+SAM_GMAC_BFR64_OFFSET)
#define SAM_GMAC_TBFR127           (SAM_GMAC_VBASE+SAM_GMAC_TBFR127_OFFSET)
#define SAM_GMAC_TBFR255           (SAM_GMAC_VBASE+SAM_GMAC_TBFR255_OFFSET)
#define SAM_GMAC_TBFR511           (SAM_GMAC_VBASE+SAM_GMAC_TBFR511_OFFSET)
#define SAM_GMAC_TBFR1023          (SAM_GMAC_VBASE+SAM_GMAC_TBFR1023_OFFSET)
#define SAM_GMAC_TBFR1518          (SAM_GMAC_VBASE+SAM_GMAC_TBFR1518_OFFSET)
#define SAM_GMAC_TMXBFR            (SAM_GMAC_VBASE+SAM_GMAC_TMXBFR_OFFSET)
#define SAM_GMAC_UFR               (SAM_GMAC_VBASE+SAM_GMAC_UFR_OFFSET)
#define SAM_GMAC_OFR               (SAM_GMAC_VBASE+SAM_GMAC_OFR_OFFSET)
#define SAM_GMAC_JR                (SAM_GMAC_VBASE+SAM_GMAC_JR_OFFSET)
#define SAM_GMAC_FCSE              (SAM_GMAC_VBASE+SAM_GMAC_FCSE_OFFSET)
#define SAM_GMAC_LFFE              (SAM_GMAC_VBASE+SAM_GMAC_LFFE_OFFSET)
#define SAM_GMAC_RSE               (SAM_GMAC_VBASE+SAM_GMAC_RSE_OFFSET)
#define SAM_GMAC_AE                (SAM_GMAC_VBASE+SAM_GMAC_AE_OFFSET)
#define SAM_GMAC_RRE               (SAM_GMAC_VBASE+SAM_GMAC_RRE_OFFSET)
#define SAM_GMAC_ROE               (SAM_GMAC_VBASE+SAM_GMAC_ROE_OFFSET)
#define SAM_GMAC_IHCE              (SAM_GMAC_VBASE+SAM_GMAC_IHCE_OFFSET)
#define SAM_GMAC_TCE               (SAM_GMAC_VBASE+SAM_GMAC_TCE_OFFSET)
#define SAM_GMAC_UCE               (SAM_GMAC_VBASE+SAM_GMAC_UCE_OFFSET)
#define SAM_GMAC_TSSS              (SAM_GMAC_VBASE+SAM_GMAC_TSSS_OFFSET)
#define SAM_GMAC_TSSN              (SAM_GMAC_VBASE+SAM_GMAC_TSSN_OFFSET)
#define SAM_GMAC_TS                (SAM_GMAC_VBASE+SAM_GMAC_TS_OFFSET)
#define SAM_GMAC_TN                (SAM_GMAC_VBASE+SAM_GMAC_TN_OFFSET)
#define SAM_GMAC_TA                (SAM_GMAC_VBASE+SAM_GMAC_TA_OFFSET)
#define SAM_GMAC_TI                (SAM_GMAC_VBASE+SAM_GMAC_TI_OFFSET)
#define SAM_GMAC_EFTS              (SAM_GMAC_VBASE+SAM_GMAC_EFTS_OFFSET)
#define SAM_GMAC_EFTN              (SAM_GMAC_VBASE+SAM_GMAC_EFTN_OFFSET)
#define SAM_GMAC_EFRS              (SAM_GMAC_VBASE+SAM_GMAC_EFRS_OFFSET)
#define SAM_GMAC_EFRN              (SAM_GMAC_VBASE+SAM_GMAC_EFRN_OFFSET)
#define SAM_GMAC_PEFTS             (SAM_GMAC_VBASE+SAM_GMAC_PEFTS_OFFSET)
#define SAM_GMAC_PEFTN             (SAM_GMAC_VBASE+SAM_GMAC_PEFTN_OFFSET)
#define SAM_GMAC_PEFRS             (SAM_GMAC_VBASE+SAM_GMAC_PEFRS_OFFSET)
#define SAM_GMAC_PEFRS             (SAM_GMAC_VBASE+SAM_GMAC_PEFRS_OFFSET)
#define SAM_GMAC_ISRPQ(n)          (SAM_GMAC_VBASE+SAM_GMAC_ISRPQ_OFFSET(n))
#define SAM_GMAC_ISRPQ0            (SAM_GMAC_VBASE+SAM_GMAC_ISRPQ0_OFFSET)
#define SAM_GMAC_ISRPQ1            (SAM_GMAC_VBASE+SAM_GMAC_ISRPQ1_OFFSET)
#define SAM_GMAC_ISRPQ2            (SAM_GMAC_VBASE+SAM_GMAC_ISRPQ2_OFFSET)
#define SAM_GMAC_ISRPQ3            (SAM_GMAC_VBASE+SAM_GMAC_ISRPQ3_OFFSET)
#define SAM_GMAC_ISRPQ4            (SAM_GMAC_VBASE+SAM_GMAC_ISRPQ4_OFFSET)
#define SAM_GMAC_ISRPQ5            (SAM_GMAC_VBASE+SAM_GMAC_ISRPQ5_OFFSET)
#define SAM_GMAC_ISRPQ6            (SAM_GMAC_VBASE+SAM_GMAC_ISRPQ6_OFFSET)
#define SAM_GMAC_TBQBAPQ(n)        (SAM_GMAC_VBASE+SAM_GMAC_TBQBAPQ_OFFSET(n))
#define SAM_GMAC_TBQBAPQ0          (SAM_GMAC_VBASE+SAM_GMAC_TBQBAPQ0_OFFSET)
#define SAM_GMAC_TBQBAPQ1          (SAM_GMAC_VBASE+SAM_GMAC_TBQBAPQ1_OFFSET)
#define SAM_GMAC_TBQBAPQ2          (SAM_GMAC_VBASE+SAM_GMAC_TBQBAPQ2_OFFSET)
#define SAM_GMAC_TBQBAPQ3          (SAM_GMAC_VBASE+SAM_GMAC_TBQBAPQ3_OFFSET)
#define SAM_GMAC_TBQBAPQ4          (SAM_GMAC_VBASE+SAM_GMAC_TBQBAPQ4_OFFSET)
#define SAM_GMAC_TBQBAPQ5          (SAM_GMAC_VBASE+SAM_GMAC_TBQBAPQ5_OFFSET)
#define SAM_GMAC_TBQBAPQ6          (SAM_GMAC_VBASE+SAM_GMAC_TBQBAPQ6_OFFSET)
#define SAM_GMAC_RBQBAPQ(n)        (SAM_GMAC_VBASE+SAM_GMAC_RBQBAPQ_OFFSET(n))
#define SAM_GMAC_RBQBAPQ0          (SAM_GMAC_VBASE+SAM_GMAC_RBQBAPQ0_OFFSET)
#define SAM_GMAC_RBQBAPQ1          (SAM_GMAC_VBASE+SAM_GMAC_RBQBAPQ1_OFFSET)
#define SAM_GMAC_RBQBAPQ2          (SAM_GMAC_VBASE+SAM_GMAC_RBQBAPQ2_OFFSET)
#define SAM_GMAC_RBQBAPQ3          (SAM_GMAC_VBASE+SAM_GMAC_RBQBAPQ3_OFFSET)
#define SAM_GMAC_RBQBAPQ4          (SAM_GMAC_VBASE+SAM_GMAC_RBQBAPQ4_OFFSET)
#define SAM_GMAC_RBQBAPQ5          (SAM_GMAC_VBASE+SAM_GMAC_RBQBAPQ5_OFFSET)
#define SAM_GMAC_RBQBAPQ6          (SAM_GMAC_VBASE+SAM_GMAC_RBQBAPQ6_OFFSET)
#define SAM_GMAC_RBSRPQ(n)         (SAM_GMAC_VBASE+SAM_GMAC_RBSRPQ_OFFSET(n))
#define SAM_GMAC_RBSRPQ0           (SAM_GMAC_VBASE+SAM_GMAC_RBSRPQ0_OFFSET)
#define SAM_GMAC_RBSRPQ1           (SAM_GMAC_VBASE+SAM_GMAC_RBSRPQ1_OFFSET)
#define SAM_GMAC_RBSRPQ2           (SAM_GMAC_VBASE+SAM_GMAC_RBSRPQ2_OFFSET)
#define SAM_GMAC_RBSRPQ3           (SAM_GMAC_VBASE+SAM_GMAC_RBSRPQ3_OFFSET)
#define SAM_GMAC_RBSRPQ4           (SAM_GMAC_VBASE+SAM_GMAC_RBSRPQ4_OFFSET)
#define SAM_GMAC_RBSRPQ5           (SAM_GMAC_VBASE+SAM_GMAC_RBSRPQ5_OFFSET)
#define SAM_GMAC_RBSRPQ6           (SAM_GMAC_VBASE+SAM_GMAC_RBSRPQ6_OFFSET)
#define SAM_GMAC_ST1RPQ(n)         (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ_OFFSET(n))
#define SAM_GMAC_ST1RPQ0           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ0_OFFSET)
#define SAM_GMAC_ST1RPQ1           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ1_OFFSET)
#define SAM_GMAC_ST1RPQ2           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ2_OFFSET)
#define SAM_GMAC_ST1RPQ3           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ3_OFFSET)
#define SAM_GMAC_ST1RPQ4           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ4_OFFSET)
#define SAM_GMAC_ST1RPQ5           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ5_OFFSET)
#define SAM_GMAC_ST1RPQ6           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ6_OFFSET)
#define SAM_GMAC_ST1RPQ7           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ7_OFFSET)
#define SAM_GMAC_ST1RPQ8           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ8_OFFSET)
#define SAM_GMAC_ST1RPQ9           (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ9_OFFSET)
#define SAM_GMAC_ST1RPQ10          (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ10_OFFSET)
#define SAM_GMAC_ST1RPQ11          (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ11_OFFSET)
#define SAM_GMAC_ST1RPQ12          (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ12_OFFSET)
#define SAM_GMAC_ST1RPQ13          (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ13_OFFSET)
#define SAM_GMAC_ST1RPQ14          (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ14_OFFSET)
#define SAM_GMAC_ST1RPQ15          (SAM_GMAC_VBASE+SAM_GMAC_ST1RPQ15_OFFSET)
#define SAM_GMAC_ST2RPQ(n)         (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ_OFFSET(n))
#define SAM_GMAC_ST2RPQ0           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ0_OFFSET)
#define SAM_GMAC_ST2RPQ1           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ1_OFFSET)
#define SAM_GMAC_ST2RPQ2           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ2_OFFSET)
#define SAM_GMAC_ST2RPQ3           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ3_OFFSET)
#define SAM_GMAC_ST2RPQ4           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ4_OFFSET)
#define SAM_GMAC_ST2RPQ5           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ5_OFFSET)
#define SAM_GMAC_ST2RPQ6           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ6_OFFSET)
#define SAM_GMAC_ST2RPQ7           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ7_OFFSET)
#define SAM_GMAC_ST2RPQ8           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ8_OFFSET)
#define SAM_GMAC_ST2RPQ9           (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ9_OFFSET)
#define SAM_GMAC_ST2RPQ10          (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ10_OFFSET)
#define SAM_GMAC_ST2RPQ11          (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ11_OFFSET)
#define SAM_GMAC_ST2RPQ12          (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ12_OFFSET)
#define SAM_GMAC_ST2RPQ13          (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ13_OFFSET)
#define SAM_GMAC_ST2RPQ14          (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ14_OFFSET)
#define SAM_GMAC_ST2RPQ15          (SAM_GMAC_VBASE+SAM_GMAC_ST2RPQ15_OFFSET)
#define SAM_GMAC_IERPQ(n)          (SAM_GMAC_VBASE+SAM_GMAC_IERPQ_OFFSET(n))
#define SAM_GMAC_IERPQ0            (SAM_GMAC_VBASE+SAM_GMAC_IERPQ0_OFFSET)
#define SAM_GMAC_IERPQ1            (SAM_GMAC_VBASE+SAM_GMAC_IERPQ1_OFFSET)
#define SAM_GMAC_IERPQ2            (SAM_GMAC_VBASE+SAM_GMAC_IERPQ2_OFFSET)
#define SAM_GMAC_IERPQ3            (SAM_GMAC_VBASE+SAM_GMAC_IERPQ3_OFFSET)
#define SAM_GMAC_IERPQ4            (SAM_GMAC_VBASE+SAM_GMAC_IERPQ4_OFFSET)
#define SAM_GMAC_IERPQ5            (SAM_GMAC_VBASE+SAM_GMAC_IERPQ5_OFFSET)
#define SAM_GMAC_IERPQ6            (SAM_GMAC_VBASE+SAM_GMAC_IERPQ6_OFFSET)
#define SAM_GMAC_IDRPQ(n)          (SAM_GMAC_VBASE+SAM_GMAC_IDRPQ_OFFSET(n))
#define SAM_GMAC_IDRPQ0            (SAM_GMAC_VBASE+SAM_GMAC_IDRPQ0_OFFSET)
#define SAM_GMAC_IDRPQ1            (SAM_GMAC_VBASE+SAM_GMAC_IDRPQ1_OFFSET)
#define SAM_GMAC_IDRPQ2            (SAM_GMAC_VBASE+SAM_GMAC_IDRPQ2_OFFSET)
#define SAM_GMAC_IDRPQ3            (SAM_GMAC_VBASE+SAM_GMAC_IDRPQ3_OFFSET)
#define SAM_GMAC_IDRPQ4            (SAM_GMAC_VBASE+SAM_GMAC_IDRPQ4_OFFSET)
#define SAM_GMAC_IDRPQ5            (SAM_GMAC_VBASE+SAM_GMAC_IDRPQ5_OFFSET)
#define SAM_GMAC_IDRPQ6            (SAM_GMAC_VBASE+SAM_GMAC_IDRPQ6_OFFSET)
#define SAM_GMAC_IMRPQ(n)          (SAM_GMAC_VBASE+SAM_GMAC_IMRPQ_OFFSET(n))
#define SAM_GMAC_IMRPQ0            (SAM_GMAC_VBASE+SAM_GMAC_IMRPQ0_OFFSET)
#define SAM_GMAC_IMRPQ1            (SAM_GMAC_VBASE+SAM_GMAC_IMRPQ1_OFFSET)
#define SAM_GMAC_IMRPQ2            (SAM_GMAC_VBASE+SAM_GMAC_IMRPQ2_OFFSET)
#define SAM_GMAC_IMRPQ3            (SAM_GMAC_VBASE+SAM_GMAC_IMRPQ3_OFFSET)
#define SAM_GMAC_IMRPQ4            (SAM_GMAC_VBASE+SAM_GMAC_IMRPQ4_OFFSET)
#define SAM_GMAC_IMRPQ5            (SAM_GMAC_VBASE+SAM_GMAC_IMRPQ5_OFFSET)
#define SAM_GMAC_IMRPQ6            (SAM_GMAC_VBASE+SAM_GMAC_IMRPQ6_OFFSET)

/* GMAC Register Bit Definitions ***************************************************/

/* Network Control Register */
#define GMAC_NCR_
/* Network Configuration Register */
#define GMAC_NCFGR_
/* Network Status Register */
#define GMAC_NSR_
/* User Register */
#define GMAC_UR_
/* DMA Configuration Register */
#define GMAC_DCFGR_
/* Transmit Status Register */
#define GMAC_TSR_
/* Receive Buffer Queue Base Address */
#define GMAC_RBQB_
/* Transmit Buffer Queue Base Address */
#define GMAC_TBQB_
/* Receive Status Register */
#define GMAC_RSR_
/* Interrupt Status Register */
#define GMAC_ISR_
/* Interrupt Enable Register */
#define GMAC_IER_
/* Interrupt Disable Register */
#define GMAC_IDR_
/* Interrupt Mask Register */
#define GMAC_IMR_
/* PHY Maintenance Register */
#define GMAC_MAN_
/* Received Pause Quantum Register */
#define GMAC_RPQ_
/* Transmit Pause Quantum Register */
#define GMAC_TPQ_
/* TX Partial Store and Forward Register */
#define GMAC_TPSF_
/* RX Partial Store and Forward Register */
#define GMAC_RPSF_
/* Hash Register Bottom [31:0] */
#define GMAC_HRB_
/* Hash Register Top [63:32] */
#define GMAC_HRT_
/* Specific Address 1 Bottom [31:0] Register */
#define GMAC_SAB1_
/* Specific Address 1 Top [47:32] Register */
#define GMAC_SAT1_
/* Specific Address 2 Bottom [31:0] Register */
#define GMAC_SAB2_
/* Specific Address 2 Top [47:32] Register */
#define GMAC_SAT2_
/* Specific Address 3 Bottom [31:0] Register */
#define GMAC_SAB3_
/* Specific Address 3 Top [47:32] Register */
#define GMAC_SAT3_
/* Specific Address 4 Bottom [31:0] Register */
#define GMAC_SAB4_
/* Specific Address 4 Top [47:32] Register */
#define GMAC_SAT4_
/* Type ID Match 1 Register */
#define GMAC_TIDM1_
/* Type ID Match 2 Register */
#define GMAC_TIDM2_
/* Type ID Match 3 Register */
#define GMAC_TIDM3_
/* Type ID Match 4 Register */
#define GMAC_TIDM4_
/* Wake on LAN Register */
#define GMAC_WOL_
/* IPG Stretch Register */
#define GMAC_IPGS_
/* Stacked VLAN Register */
#define GMAC_SVLAN_
/* Transmit PFC Pause Register */
#define GMAC_TPFCP_
/* Specific Address 1 Mask Bottom [31:0] Register */
#define GMAC_SAMB1_
/* Specific Address 1 Mask Top [47:32] Register */
#define GMAC_SAMT1_
/* Octets Transmitted [31:0] Register */
#define GMAC_OTLO_
/* Octets Transmitted [47:32] Register */
#define GMAC_OTHI_
/* Frames Transmitted Register */
#define GMAC_FT_
/* Broadcast Frames Transmitted Register */
#define GMAC_BCFT_
/* Multicast Frames Transmitted Register */
#define GMAC_MFT_
/* Pause Frames Transmitted Register */
#define GMAC_PFT_
/* 64 Byte Frames Transmitted Register */
#define GMAC_BFT64_
/* 65 to 127 Byte Frames Transmitted Register */
#define GMAC_TBFT127_
/* 128 to 255 Byte Frames Transmitted Register */
#define GMAC_TBFT255_
/* 256 to 511 Byte Frames Transmitted Register */
#define GMAC_TBFT511_
/* 512 to 1023 Byte Frames Transmitted Register */
#define GMAC_TBFT1023_
/* 1024 to 1518 Byte Frames Transmitted Register */
#define GMAC_TBFT1518_
/* Greater Than 1518 Byte Frames Transmitted Register */
#define GMAC_GTBFT1518_
/* Transmit Under Runs Register */
#define GMAC_TUR_
/* Single Collision Frames Register */
#define GMAC_SCF_
/* Multiple Collision Frames Register */
#define GMAC_MCF_
/* Excessive Collisions Register */
#define GMAC_EC_
/* Late Collisions Register */
#define GMAC_LC_
/* Deferred Transmission Frames Register */
#define GMAC_DTF_
/* Carrier Sense Errors Register */
#define GMAC_CSE_
/* Octets Received [31:0] Received */
#define GMAC_ORLO_
/* Octets Received [47:32] Received */
#define GMAC_ORHI_
/* Frames Received Register */
#define GMAC_FR_
/* Broadcast Frames Received Register */
#define GMAC_BCFR_
/* Multicast Frames Received Register */
#define GMAC_MFR_
/* Pause Frames Received Register */
#define GMAC_PFR_
/* 64 Byte Frames Received Register */
#define GMAC_BFR64_
/* 65 to 127 Byte Frames Received Register */
#define GMAC_TBFR127_
/* 128 to 255 Byte Frames Received Register */
#define GMAC_TBFR255_
/* 256 to 511Byte Frames Received Register */
#define GMAC_TBFR511_
/* 512 to 1023 Byte Frames Received Register */
#define GMAC_TBFR1023_
/* 1024 to 1518 Byte Frames Received Register */
#define GMAC_TBFR1518_
/* 1519 to Maximum Byte Frames Received Register */
#define GMAC_TMXBFR_
/* Undersize Frames Received Register */
#define GMAC_UFR_
/* Oversize Frames Received Register */
#define GMAC_OFR_
/* Jabbers Received Register */
#define GMAC_JR_
/* Frame Check Sequence Errors Register */
#define GMAC_FCSE_
/* Length Field Frame Errors Register */
#define GMAC_LFFE_
/* Receive Symbol Errors Register */
#define GMAC_RSE_
/* Alignment Errors Register */
#define GMAC_AE_
/* Receive Resource Errors Register */
#define GMAC_RRE_
/* Receive Overrun Register */
#define GMAC_ROE_
/* IP Header Checksum Errors Register */
#define GMAC_IHCE_
/* TCP Checksum Errors Register */
#define GMAC_TCE_
/* UDP Checksum Errors Register */
#define GMAC_UCE_
/* 1588 Timer Sync Strobe Seconds Register */
#define GMAC_TSSS_
/* 1588 Timer Sync Strobe Nanoseconds Register */
#define GMAC_TSSN_
/* 1588 Timer Seconds Register */
#define GMAC_TS_
/* 1588 Timer Nanoseconds Register */
#define GMAC_TN_
/* 1588 Timer Adjust Register */
#define GMAC_TA_
/* 1588 Timer Increment Register */
#define GMAC_TI_
/* PTP Event Frame Transmitted Seconds */
#define GMAC_EFTS_
/* PTP Event Frame Transmitted Nanoseconds */
#define GMAC_EFTN_
/* PTP Event Frame Received Seconds */
#define GMAC_EFRS_
/* PTP Event Frame Received Nanoseconds */
#define GMAC_EFRN_
/* PTP Peer Event Frame Transmitted Seconds */
#define GMAC_PEFTS_
/* PTP Peer Event Frame Transmitted Nanoseconds */
#define GMAC_PEFTN_
/* PTP Peer Event Frame ReceiveGMAC_PEFRNd Seconds */
#define GMAC_PEFRS_
/* PTP Peer Event Frame Received Nanoseconds */
#define GMAC_PEFRS_
/* Interrupt Status Register Priority Queue 0-6 */
#define GMAC_ISRPQ0_
/* Transmit Buffer Queue Base Address Priority Queue 0-6 */
#define GMAC_TBQBAPQ0_
/* Receive Buffer Queue Base Address Priority Queue 0-6 */
#define GMAC_RBQBAPQ0_
/* Receive Buffer Size Register Priority Queue 0-6 */
#define GMAC_RBSRPQ0_
/* Screening Type1 Register Priority Queue 0-15 */
#define GMAC_ST1RPQ0_
/* Screening Type2 Register Priority Queue 0-15 */
#define GMAC_ST2RPQ0_
/* Interrupt Enable Register Priority Queue 0-6 */
#define GMAC_IERPQ0_
/* Interrupt Disable Register Priority Queue 0-6 */
#define GMAC_IDRPQ0_
/* Interrupt Mask Register Priority Queue 0-6 */
#define GMAC_IMRPQ0_

        (1 << nn)  /* Bit nn: 
_SHIFT  (nn)       /* Bits nn-nn: 
_MASK   (xx << yy)

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_GMAC_H */
