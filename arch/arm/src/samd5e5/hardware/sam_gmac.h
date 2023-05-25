/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_gmac.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5E5_HARDWARE_SAM_GMAC_H
#define __ARCH_ARM_SRC_SAMA5E5_HARDWARE_SAM_GMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GMAC Register Offsets ****************************************************/

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
#define SAM_GMAC_PEFRS_OFFSET      0x01f8 /* PTP Peer Event Frame Received Seconds */
#define SAM_GMAC_PEFRN_OFFSET      0x01fc /* PTP Peer Event Frame Received Nanoseconds */
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

/* GMAC Register Addresses **************************************************/

#define SAM_GMAC_NCR               (SAM_GMAC_BASE+SAM_GMAC_NCR_OFFSET)
#define SAM_GMAC_NCFGR             (SAM_GMAC_BASE+SAM_GMAC_NCFGR_OFFSET)
#define SAM_GMAC_NSR               (SAM_GMAC_BASE+SAM_GMAC_NSR_OFFSET)
#define SAM_GMAC_UR                (SAM_GMAC_BASE+SAM_GMAC_UR_OFFSET)
#define SAM_GMAC_DCFGR             (SAM_GMAC_BASE+SAM_GMAC_DCFGR_OFFSET)
#define SAM_GMAC_TSR               (SAM_GMAC_BASE+SAM_GMAC_TSR_OFFSET)
#define SAM_GMAC_RBQB              (SAM_GMAC_BASE+SAM_GMAC_RBQB_OFFSET)
#define SAM_GMAC_TBQB              (SAM_GMAC_BASE+SAM_GMAC_TBQB_OFFSET)
#define SAM_GMAC_RSR               (SAM_GMAC_BASE+SAM_GMAC_RSR_OFFSET)
#define SAM_GMAC_ISR               (SAM_GMAC_BASE+SAM_GMAC_ISR_OFFSET)
#define SAM_GMAC_IER               (SAM_GMAC_BASE+SAM_GMAC_IER_OFFSET)
#define SAM_GMAC_IDR               (SAM_GMAC_BASE+SAM_GMAC_IDR_OFFSET)
#define SAM_GMAC_IMR               (SAM_GMAC_BASE+SAM_GMAC_IMR_OFFSET)
#define SAM_GMAC_MAN               (SAM_GMAC_BASE+SAM_GMAC_MAN_OFFSET)
#define SAM_GMAC_RPQ               (SAM_GMAC_BASE+SAM_GMAC_RPQ_OFFSET)
#define SAM_GMAC_TPQ               (SAM_GMAC_BASE+SAM_GMAC_TPQ_OFFSET)
#define SAM_GMAC_TPSF              (SAM_GMAC_BASE+SAM_GMAC_TPSF_OFFSET)
#define SAM_GMAC_RPSF              (SAM_GMAC_BASE+SAM_GMAC_RPSF_OFFSET)
#define SAM_GMAC_HRB               (SAM_GMAC_BASE+SAM_GMAC_HRB_OFFSET)
#define SAM_GMAC_HRT               (SAM_GMAC_BASE+SAM_GMAC_HRT_OFFSET)
#define SAM_GMAC_SAB(n)            (SAM_GMAC_BASE+SAM_GMAC_SAB_OFFSET(n))
#define SAM_GMAC_SAT(n)            (SAM_GMAC_BASE+SAM_GMAC_SAT_OFFSET(n))
#define SAM_GMAC_SAB1              (SAM_GMAC_BASE+SAM_GMAC_SAB1_OFFSET)
#define SAM_GMAC_SAT1              (SAM_GMAC_BASE+SAM_GMAC_SAT1_OFFSET)
#define SAM_GMAC_SAB2              (SAM_GMAC_BASE+SAM_GMAC_SAB2_OFFSET)
#define SAM_GMAC_SAT2              (SAM_GMAC_BASE+SAM_GMAC_SAT2_OFFSET)
#define SAM_GMAC_SAB3              (SAM_GMAC_BASE+SAM_GMAC_SAB3_OFFSET)
#define SAM_GMAC_SAT3              (SAM_GMAC_BASE+SAM_GMAC_SAT3_OFFSET)
#define SAM_GMAC_SAB4              (SAM_GMAC_BASE+SAM_GMAC_SAB4_OFFSET)
#define SAM_GMAC_SAT4              (SAM_GMAC_BASE+SAM_GMAC_SAT4_OFFSET)
#define SAM_GMAC_TIDM(n)           (SAM_GMAC_BASE+SAM_GMAC_TIDM_OFFSET(n))
#define SAM_GMAC_TIDM1             (SAM_GMAC_BASE+SAM_GMAC_TIDM1_OFFSET)
#define SAM_GMAC_TIDM2             (SAM_GMAC_BASE+SAM_GMAC_TIDM2_OFFSET)
#define SAM_GMAC_TIDM3             (SAM_GMAC_BASE+SAM_GMAC_TIDM3_OFFSET)
#define SAM_GMAC_TIDM4             (SAM_GMAC_BASE+SAM_GMAC_TIDM4_OFFSET)
#define SAM_GMAC_WOL               (SAM_GMAC_BASE+SAM_GMAC_WOL_OFFSET)
#define SAM_GMAC_IPGS              (SAM_GMAC_BASE+SAM_GMAC_IPGS_OFFSET)
#define SAM_GMAC_SVLAN             (SAM_GMAC_BASE+SAM_GMAC_SVLAN_OFFSET)
#define SAM_GMAC_TPFCP             (SAM_GMAC_BASE+SAM_GMAC_TPFCP_OFFSET)
#define SAM_GMAC_SAMB1             (SAM_GMAC_BASE+SAM_GMAC_SAMB1_OFFSET)
#define SAM_GMAC_SAMT1             (SAM_GMAC_BASE+SAM_GMAC_SAMT1_OFFSET)
#define SAM_GMAC_OTLO              (SAM_GMAC_BASE+SAM_GMAC_OTLO_OFFSET)
#define SAM_GMAC_OTHI              (SAM_GMAC_BASE+SAM_GMAC_OTHI_OFFSET)
#define SAM_GMAC_FT                (SAM_GMAC_BASE+SAM_GMAC_FT_OFFSET)
#define SAM_GMAC_BCFT              (SAM_GMAC_BASE+SAM_GMAC_BCFT_OFFSET)
#define SAM_GMAC_MFT               (SAM_GMAC_BASE+SAM_GMAC_MFT_OFFSET)
#define SAM_GMAC_PFT               (SAM_GMAC_BASE+SAM_GMAC_PFT_OFFSET)
#define SAM_GMAC_BFT64             (SAM_GMAC_BASE+SAM_GMAC_BFT64_OFFSET)
#define SAM_GMAC_TBFT127           (SAM_GMAC_BASE+SAM_GMAC_TBFT127_OFFSET)
#define SAM_GMAC_TBFT255           (SAM_GMAC_BASE+SAM_GMAC_TBFT255_OFFSET)
#define SAM_GMAC_TBFT511           (SAM_GMAC_BASE+SAM_GMAC_TBFT511_OFFSET)
#define SAM_GMAC_TBFT1023          (SAM_GMAC_BASE+SAM_GMAC_TBFT1023_OFFSET)
#define SAM_GMAC_TBFT1518          (SAM_GMAC_BASE+SAM_GMAC_TBFT1518_OFFSET)
#define SAM_GMAC_GTBFT1518         (SAM_GMAC_BASE+SAM_GMAC_GTBFT1518_OFFSET)
#define SAM_GMAC_TUR               (SAM_GMAC_BASE+SAM_GMAC_TUR_OFFSET)
#define SAM_GMAC_SCF               (SAM_GMAC_BASE+SAM_GMAC_SCF_OFFSET)
#define SAM_GMAC_MCF               (SAM_GMAC_BASE+SAM_GMAC_MCF_OFFSET)
#define SAM_GMAC_EC                (SAM_GMAC_BASE+SAM_GMAC_EC_OFFSET)
#define SAM_GMAC_LC                (SAM_GMAC_BASE+SAM_GMAC_LC_OFFSET)
#define SAM_GMAC_DTF               (SAM_GMAC_BASE+SAM_GMAC_DTF_OFFSET)
#define SAM_GMAC_CSE               (SAM_GMAC_BASE+SAM_GMAC_CSE_OFFSET)
#define SAM_GMAC_ORLO              (SAM_GMAC_BASE+SAM_GMAC_ORLO_OFFSET)
#define SAM_GMAC_ORHI              (SAM_GMAC_BASE+SAM_GMAC_ORHI_OFFSET)
#define SAM_GMAC_FR                (SAM_GMAC_BASE+SAM_GMAC_FR_OFFSET)
#define SAM_GMAC_BCFR              (SAM_GMAC_BASE+SAM_GMAC_BCFR_OFFSET)
#define SAM_GMAC_MFR               (SAM_GMAC_BASE+SAM_GMAC_MFR_OFFSET)
#define SAM_GMAC_PFR               (SAM_GMAC_BASE+SAM_GMAC_PFR_OFFSET)
#define SAM_GMAC_BFR64             (SAM_GMAC_BASE+SAM_GMAC_BFR64_OFFSET)
#define SAM_GMAC_TBFR127           (SAM_GMAC_BASE+SAM_GMAC_TBFR127_OFFSET)
#define SAM_GMAC_TBFR255           (SAM_GMAC_BASE+SAM_GMAC_TBFR255_OFFSET)
#define SAM_GMAC_TBFR511           (SAM_GMAC_BASE+SAM_GMAC_TBFR511_OFFSET)
#define SAM_GMAC_TBFR1023          (SAM_GMAC_BASE+SAM_GMAC_TBFR1023_OFFSET)
#define SAM_GMAC_TBFR1518          (SAM_GMAC_BASE+SAM_GMAC_TBFR1518_OFFSET)
#define SAM_GMAC_TMXBFR            (SAM_GMAC_BASE+SAM_GMAC_TMXBFR_OFFSET)
#define SAM_GMAC_UFR               (SAM_GMAC_BASE+SAM_GMAC_UFR_OFFSET)
#define SAM_GMAC_OFR               (SAM_GMAC_BASE+SAM_GMAC_OFR_OFFSET)
#define SAM_GMAC_JR                (SAM_GMAC_BASE+SAM_GMAC_JR_OFFSET)
#define SAM_GMAC_FCSE              (SAM_GMAC_BASE+SAM_GMAC_FCSE_OFFSET)
#define SAM_GMAC_LFFE              (SAM_GMAC_BASE+SAM_GMAC_LFFE_OFFSET)
#define SAM_GMAC_RSE               (SAM_GMAC_BASE+SAM_GMAC_RSE_OFFSET)
#define SAM_GMAC_AE                (SAM_GMAC_BASE+SAM_GMAC_AE_OFFSET)
#define SAM_GMAC_RRE               (SAM_GMAC_BASE+SAM_GMAC_RRE_OFFSET)
#define SAM_GMAC_ROE               (SAM_GMAC_BASE+SAM_GMAC_ROE_OFFSET)
#define SAM_GMAC_IHCE              (SAM_GMAC_BASE+SAM_GMAC_IHCE_OFFSET)
#define SAM_GMAC_TCE               (SAM_GMAC_BASE+SAM_GMAC_TCE_OFFSET)
#define SAM_GMAC_UCE               (SAM_GMAC_BASE+SAM_GMAC_UCE_OFFSET)
#define SAM_GMAC_TSSS              (SAM_GMAC_BASE+SAM_GMAC_TSSS_OFFSET)
#define SAM_GMAC_TSSN              (SAM_GMAC_BASE+SAM_GMAC_TSSN_OFFSET)
#define SAM_GMAC_TS                (SAM_GMAC_BASE+SAM_GMAC_TS_OFFSET)
#define SAM_GMAC_TN                (SAM_GMAC_BASE+SAM_GMAC_TN_OFFSET)
#define SAM_GMAC_TA                (SAM_GMAC_BASE+SAM_GMAC_TA_OFFSET)
#define SAM_GMAC_TI                (SAM_GMAC_BASE+SAM_GMAC_TI_OFFSET)
#define SAM_GMAC_EFTS              (SAM_GMAC_BASE+SAM_GMAC_EFTS_OFFSET)
#define SAM_GMAC_EFTN              (SAM_GMAC_BASE+SAM_GMAC_EFTN_OFFSET)
#define SAM_GMAC_EFRS              (SAM_GMAC_BASE+SAM_GMAC_EFRS_OFFSET)
#define SAM_GMAC_EFRN              (SAM_GMAC_BASE+SAM_GMAC_EFRN_OFFSET)
#define SAM_GMAC_PEFTS             (SAM_GMAC_BASE+SAM_GMAC_PEFTS_OFFSET)
#define SAM_GMAC_PEFTN             (SAM_GMAC_BASE+SAM_GMAC_PEFTN_OFFSET)
#define SAM_GMAC_PEFRS             (SAM_GMAC_BASE+SAM_GMAC_PEFRS_OFFSET)
#define SAM_GMAC_PEFRN             (SAM_GMAC_BASE+SAM_GMAC_PEFRN_OFFSET)
#define SAM_GMAC_ISRPQ(n)          (SAM_GMAC_BASE+SAM_GMAC_ISRPQ_OFFSET(n))
#define SAM_GMAC_ISRPQ0            (SAM_GMAC_BASE+SAM_GMAC_ISRPQ0_OFFSET)
#define SAM_GMAC_ISRPQ1            (SAM_GMAC_BASE+SAM_GMAC_ISRPQ1_OFFSET)
#define SAM_GMAC_ISRPQ2            (SAM_GMAC_BASE+SAM_GMAC_ISRPQ2_OFFSET)
#define SAM_GMAC_ISRPQ3            (SAM_GMAC_BASE+SAM_GMAC_ISRPQ3_OFFSET)
#define SAM_GMAC_ISRPQ4            (SAM_GMAC_BASE+SAM_GMAC_ISRPQ4_OFFSET)
#define SAM_GMAC_ISRPQ5            (SAM_GMAC_BASE+SAM_GMAC_ISRPQ5_OFFSET)
#define SAM_GMAC_ISRPQ6            (SAM_GMAC_BASE+SAM_GMAC_ISRPQ6_OFFSET)
#define SAM_GMAC_TBQBAPQ(n)        (SAM_GMAC_BASE+SAM_GMAC_TBQBAPQ_OFFSET(n))
#define SAM_GMAC_TBQBAPQ0          (SAM_GMAC_BASE+SAM_GMAC_TBQBAPQ0_OFFSET)
#define SAM_GMAC_TBQBAPQ1          (SAM_GMAC_BASE+SAM_GMAC_TBQBAPQ1_OFFSET)
#define SAM_GMAC_TBQBAPQ2          (SAM_GMAC_BASE+SAM_GMAC_TBQBAPQ2_OFFSET)
#define SAM_GMAC_TBQBAPQ3          (SAM_GMAC_BASE+SAM_GMAC_TBQBAPQ3_OFFSET)
#define SAM_GMAC_TBQBAPQ4          (SAM_GMAC_BASE+SAM_GMAC_TBQBAPQ4_OFFSET)
#define SAM_GMAC_TBQBAPQ5          (SAM_GMAC_BASE+SAM_GMAC_TBQBAPQ5_OFFSET)
#define SAM_GMAC_TBQBAPQ6          (SAM_GMAC_BASE+SAM_GMAC_TBQBAPQ6_OFFSET)
#define SAM_GMAC_RBQBAPQ(n)        (SAM_GMAC_BASE+SAM_GMAC_RBQBAPQ_OFFSET(n))
#define SAM_GMAC_RBQBAPQ0          (SAM_GMAC_BASE+SAM_GMAC_RBQBAPQ0_OFFSET)
#define SAM_GMAC_RBQBAPQ1          (SAM_GMAC_BASE+SAM_GMAC_RBQBAPQ1_OFFSET)
#define SAM_GMAC_RBQBAPQ2          (SAM_GMAC_BASE+SAM_GMAC_RBQBAPQ2_OFFSET)
#define SAM_GMAC_RBQBAPQ3          (SAM_GMAC_BASE+SAM_GMAC_RBQBAPQ3_OFFSET)
#define SAM_GMAC_RBQBAPQ4          (SAM_GMAC_BASE+SAM_GMAC_RBQBAPQ4_OFFSET)
#define SAM_GMAC_RBQBAPQ5          (SAM_GMAC_BASE+SAM_GMAC_RBQBAPQ5_OFFSET)
#define SAM_GMAC_RBQBAPQ6          (SAM_GMAC_BASE+SAM_GMAC_RBQBAPQ6_OFFSET)
#define SAM_GMAC_RBSRPQ(n)         (SAM_GMAC_BASE+SAM_GMAC_RBSRPQ_OFFSET(n))
#define SAM_GMAC_RBSRPQ0           (SAM_GMAC_BASE+SAM_GMAC_RBSRPQ0_OFFSET)
#define SAM_GMAC_RBSRPQ1           (SAM_GMAC_BASE+SAM_GMAC_RBSRPQ1_OFFSET)
#define SAM_GMAC_RBSRPQ2           (SAM_GMAC_BASE+SAM_GMAC_RBSRPQ2_OFFSET)
#define SAM_GMAC_RBSRPQ3           (SAM_GMAC_BASE+SAM_GMAC_RBSRPQ3_OFFSET)
#define SAM_GMAC_RBSRPQ4           (SAM_GMAC_BASE+SAM_GMAC_RBSRPQ4_OFFSET)
#define SAM_GMAC_RBSRPQ5           (SAM_GMAC_BASE+SAM_GMAC_RBSRPQ5_OFFSET)
#define SAM_GMAC_RBSRPQ6           (SAM_GMAC_BASE+SAM_GMAC_RBSRPQ6_OFFSET)
#define SAM_GMAC_ST1RPQ(n)         (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ_OFFSET(n))
#define SAM_GMAC_ST1RPQ0           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ0_OFFSET)
#define SAM_GMAC_ST1RPQ1           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ1_OFFSET)
#define SAM_GMAC_ST1RPQ2           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ2_OFFSET)
#define SAM_GMAC_ST1RPQ3           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ3_OFFSET)
#define SAM_GMAC_ST1RPQ4           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ4_OFFSET)
#define SAM_GMAC_ST1RPQ5           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ5_OFFSET)
#define SAM_GMAC_ST1RPQ6           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ6_OFFSET)
#define SAM_GMAC_ST1RPQ7           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ7_OFFSET)
#define SAM_GMAC_ST1RPQ8           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ8_OFFSET)
#define SAM_GMAC_ST1RPQ9           (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ9_OFFSET)
#define SAM_GMAC_ST1RPQ10          (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ10_OFFSET)
#define SAM_GMAC_ST1RPQ11          (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ11_OFFSET)
#define SAM_GMAC_ST1RPQ12          (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ12_OFFSET)
#define SAM_GMAC_ST1RPQ13          (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ13_OFFSET)
#define SAM_GMAC_ST1RPQ14          (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ14_OFFSET)
#define SAM_GMAC_ST1RPQ15          (SAM_GMAC_BASE+SAM_GMAC_ST1RPQ15_OFFSET)
#define SAM_GMAC_ST2RPQ(n)         (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ_OFFSET(n))
#define SAM_GMAC_ST2RPQ0           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ0_OFFSET)
#define SAM_GMAC_ST2RPQ1           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ1_OFFSET)
#define SAM_GMAC_ST2RPQ2           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ2_OFFSET)
#define SAM_GMAC_ST2RPQ3           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ3_OFFSET)
#define SAM_GMAC_ST2RPQ4           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ4_OFFSET)
#define SAM_GMAC_ST2RPQ5           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ5_OFFSET)
#define SAM_GMAC_ST2RPQ6           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ6_OFFSET)
#define SAM_GMAC_ST2RPQ7           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ7_OFFSET)
#define SAM_GMAC_ST2RPQ8           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ8_OFFSET)
#define SAM_GMAC_ST2RPQ9           (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ9_OFFSET)
#define SAM_GMAC_ST2RPQ10          (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ10_OFFSET)
#define SAM_GMAC_ST2RPQ11          (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ11_OFFSET)
#define SAM_GMAC_ST2RPQ12          (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ12_OFFSET)
#define SAM_GMAC_ST2RPQ13          (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ13_OFFSET)
#define SAM_GMAC_ST2RPQ14          (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ14_OFFSET)
#define SAM_GMAC_ST2RPQ15          (SAM_GMAC_BASE+SAM_GMAC_ST2RPQ15_OFFSET)
#define SAM_GMAC_IERPQ(n)          (SAM_GMAC_BASE+SAM_GMAC_IERPQ_OFFSET(n))
#define SAM_GMAC_IERPQ0            (SAM_GMAC_BASE+SAM_GMAC_IERPQ0_OFFSET)
#define SAM_GMAC_IERPQ1            (SAM_GMAC_BASE+SAM_GMAC_IERPQ1_OFFSET)
#define SAM_GMAC_IERPQ2            (SAM_GMAC_BASE+SAM_GMAC_IERPQ2_OFFSET)
#define SAM_GMAC_IERPQ3            (SAM_GMAC_BASE+SAM_GMAC_IERPQ3_OFFSET)
#define SAM_GMAC_IERPQ4            (SAM_GMAC_BASE+SAM_GMAC_IERPQ4_OFFSET)
#define SAM_GMAC_IERPQ5            (SAM_GMAC_BASE+SAM_GMAC_IERPQ5_OFFSET)
#define SAM_GMAC_IERPQ6            (SAM_GMAC_BASE+SAM_GMAC_IERPQ6_OFFSET)
#define SAM_GMAC_IDRPQ(n)          (SAM_GMAC_BASE+SAM_GMAC_IDRPQ_OFFSET(n))
#define SAM_GMAC_IDRPQ0            (SAM_GMAC_BASE+SAM_GMAC_IDRPQ0_OFFSET)
#define SAM_GMAC_IDRPQ1            (SAM_GMAC_BASE+SAM_GMAC_IDRPQ1_OFFSET)
#define SAM_GMAC_IDRPQ2            (SAM_GMAC_BASE+SAM_GMAC_IDRPQ2_OFFSET)
#define SAM_GMAC_IDRPQ3            (SAM_GMAC_BASE+SAM_GMAC_IDRPQ3_OFFSET)
#define SAM_GMAC_IDRPQ4            (SAM_GMAC_BASE+SAM_GMAC_IDRPQ4_OFFSET)
#define SAM_GMAC_IDRPQ5            (SAM_GMAC_BASE+SAM_GMAC_IDRPQ5_OFFSET)
#define SAM_GMAC_IDRPQ6            (SAM_GMAC_BASE+SAM_GMAC_IDRPQ6_OFFSET)
#define SAM_GMAC_IMRPQ(n)          (SAM_GMAC_BASE+SAM_GMAC_IMRPQ_OFFSET(n))
#define SAM_GMAC_IMRPQ0            (SAM_GMAC_BASE+SAM_GMAC_IMRPQ0_OFFSET)
#define SAM_GMAC_IMRPQ1            (SAM_GMAC_BASE+SAM_GMAC_IMRPQ1_OFFSET)
#define SAM_GMAC_IMRPQ2            (SAM_GMAC_BASE+SAM_GMAC_IMRPQ2_OFFSET)
#define SAM_GMAC_IMRPQ3            (SAM_GMAC_BASE+SAM_GMAC_IMRPQ3_OFFSET)
#define SAM_GMAC_IMRPQ4            (SAM_GMAC_BASE+SAM_GMAC_IMRPQ4_OFFSET)
#define SAM_GMAC_IMRPQ5            (SAM_GMAC_BASE+SAM_GMAC_IMRPQ5_OFFSET)
#define SAM_GMAC_IMRPQ6            (SAM_GMAC_BASE+SAM_GMAC_IMRPQ6_OFFSET)

/* GMAC Register Bit Definitions ********************************************/

/* Network Control Register */

#define GMAC_NCR_LBL               (1 << 1)  /* Bit 1:  Loopback local */
#define GMAC_NCR_RXEN              (1 << 2)  /* Bit 2:  Receive enable */
#define GMAC_NCR_TXEN              (1 << 3)  /* Bit 3:  Transmit enable */
#define GMAC_NCR_MPE               (1 << 4)  /* Bit 4:  Management port enable */
#define GMAC_NCR_CLRSTAT           (1 << 5)  /* Bit 5:  Clear statistics registers */
#define GMAC_NCR_INCSTAT           (1 << 6)  /* Bit 6:  Increment statistics registers */
#define GMAC_NCR_WESTAT            (1 << 7)  /* Bit 7:  Write enable for statistics registers */
#define GMAC_NCR_BP                (1 << 8)  /* Bit 8:  Back pressure */
#define GMAC_NCR_TSTART            (1 << 9)  /* Bit 9:  Start transmission */
#define GMAC_NCR_THALT             (1 << 10) /* Bit 10: Transmit halt */
#define GMAC_NCR_TXPF              (1 << 11) /* Bit 11: Transmit Pause Frame */
#define GMAC_NCR_TXZQPF            (1 << 12) /* Bit 12: Transmit Zero Quantum Pause Frame */
#define GMAC_NCR_RDS               (1 << 14) /* Bit 14: Read Snapshot */
#define GMAC_NCR_SRTSM             (1 << 15) /* Bit 15: Store Receive Time Stamp to Memory */
#define GMAC_NCR_ENPBPR            (1 << 16) /* Bit 16: Enable PFC Priority-based Pause Reception */
#define GMAC_NCR_TXPBPF            (1 << 17) /* Bit 17: Transmit PFC Priority-based Pause Frame */
#define GMAC_NCR_FNP               (1 << 18) /* Bit 18: Flush Next Packet */

/* Network Configuration Register */

#define GMAC_NCFGR_SPD            (1 << 0)  /* Bit 0:  Speed */
#define GMAC_NCFGR_FD             (1 << 1)  /* Bit 1:  Full Duplex */
#define GMAC_NCFGR_DNVLAN         (1 << 2)  /* Bit 2:  Discard Non-VLAN FRAMES */
#define GMAC_NCFGR_JFRAME         (1 << 3)  /* Bit 3:  Jumbo Frames */
#define GMAC_NCFGR_CAF            (1 << 4)  /* Bit 4:  Copy All Frames */
#define GMAC_NCFGR_NBC            (1 << 5)  /* Bit 5:  No Broadcast */
#define GMAC_NCFGR_MTIHEN         (1 << 6)  /* Bit 6:  Multicast Hash Enable */
#define GMAC_NCFGR_UNIHEN         (1 << 7)  /* Bit 7:  Unicast Hash Enable */
#define GMAC_NCFGR_MAXFS          (1 << 8)  /* Bit 8:  Receive 1536 bytes frames */
#define GMAC_NCFGR_RTY            (1 << 12) /* Bit 12: Retry test */
#define GMAC_NCFGR_PEN            (1 << 13) /* Bit 13: Pause Enable */
#define GMAC_NCFGR_RXBUFO_SHIFT   (14)      /* Bits 14-15: Receive Buffer Offset */
#define GMAC_NCFGR_RXBUFO_MASK    (3 << GMAC_NCFGR_RXBUFO_SHIFT)
#define GMAC_NCFGR_LFERD          (1 << 16) /* Bit 16: Length Field Error Frame Discard */
#define GMAC_NCFGR_RFCS           (1 << 17) /* Bit 17: Remove FCS */
#define GMAC_NCFGR_CLK_SHIFT      (18)      /* Bits 18-20: MDC clock division */
#define GMAC_NCFGR_CLK_MASK       (7 << GMAC_NCFGR_CLK_SHIFT)
#  define GMAC_NCFGR_CLK_DIV8     (0 << GMAC_NCFGR_CLK_SHIFT) /* MCK divided by 8 (MCK up to 20 MHz) */
#  define GMAC_NCFGR_CLK_DIV16    (1 << GMAC_NCFGR_CLK_SHIFT) /* MCK divided by 16 (MCK up to 40 MHz) */
#  define GMAC_NCFGR_CLK_DIV32    (2 << GMAC_NCFGR_CLK_SHIFT) /* MCK divided by 32 (MCK up to 80 MHz) */
#  define GMAC_NCFGR_CLK_DIV48    (3 << GMAC_NCFGR_CLK_SHIFT) /* MCK divided by 48 (MCK up to 120 MHz) */
#  define GMAC_NCFGR_CLK_DIV64    (4 << GMAC_NCFGR_CLK_SHIFT) /* MCK divided by 64 (MCK up to 160 MHz) */
#  define GMAC_NCFGR_CLK_DIV96    (5 << GMAC_NCFGR_CLK_SHIFT) /* MCK divided by 96 (MCK up to 240 MHz) */

#define GMAC_NCFGR_DBW_SHIFT      (21)      /* Bits 21-22: Data Bus Width */
#define GMAC_NCFGR_DBW_MASK       (3 << GMAC_NCFGR_DBW_SHIFT)
#  define GMAC_NCFGR_DBW_32       (0 << GMAC_NCFGR_DBW_SHIFT) /* 32-bit data bus width */
#  define GMAC_NCFGR_DBW_64       (1 << GMAC_NCFGR_DBW_SHIFT) /* 64-bit data bus width */

#define GMAC_NCFGR_DCPF           (1 << 23) /* Bit 23: Disable Copy of Pause Frames */
#define GMAC_NCFGR_RXCOEN         (1 << 24) /* Bit 24: Receive Checksum Offload Enable */
#define GMAC_NCFGR_EFRHD          (1 << 25) /* Bit 25: Enable Frames Received in Half Duplex */
#define GMAC_NCFGR_IRXFCS         (1 << 26) /* Bit 26: Ignore RX FCS */
#define GMAC_NCFGR_IPGSEN         (1 << 28) /* Bit 28: IP Stretch Enable */
#define GMAC_NCFGR_RXBP           (1 << 29) /* Bit 29: Receive Bad Preamble */
#define GMAC_NCFGR_IRXER          (1 << 30) /* Bit 30: Ignore IPG GRXER */

/* Network Status Register */

#define GMAC_NSR_MDIO             (1 << 1)  /* Bit 1:  MDIO Input Status */
#define GMAC_NSR_IDLE             (1 << 2)  /* Bit 2:  PHY management logic idle */

/* User Register */

#define GMAC_UR_MII               (1 << 0)  /* Bit 0:  MII Mode */

/* DMA Configuration Register */

#define GMAC_DCFGR_FBLDO_SHIFT    (0)       /* Bits 0-4: Fixed Burst Length for DMA Data Operations */
#define GMAC_DCFGR_FBLDO_MASK     (31 << GMAC_DCFGR_FBLDO_SHIFT)
#  define GMAC_DCFGR_FBLDO_SINGLE (1 << GMAC_DCFGR_FBLDO_SHIFT)  /* 00001: Always use SINGLE AHB bursts */
#  define GMAC_DCFGR_FBLDO_INCR4  (4 << GMAC_DCFGR_FBLDO_SHIFT)  /* 001xx: Attempt to use INCR4 AHB bursts */
#  define GMAC_DCFGR_FBLDO_INCR8  (8 << GMAC_DCFGR_FBLDO_SHIFT)  /* 01xxx: Attempt to use INCR8 AHB bursts */
#  define GMAC_DCFGR_FBLDO_INCR16 (16 << GMAC_DCFGR_FBLDO_SHIFT) /* 1xxxx: Attempt to use INCR16 AHB bursts */

#define GMAC_DCFGR_ESMA           (1 << 6)  /* Bit 6:  Endian Swap Mode Enable for Management Descriptor Accesses */
#define GMAC_DCFGR_ESPA           (1 << 7)  /* Bit 7:  Endian Swap Mode Enable for Packet Data Accesses */
#define GMAC_DCFGR_RXBMS_SHIFT    (8)       /* Bits 8-9: Receiver Packet Buffer Memory Size Select */
#define GMAC_DCFGR_RXBMS_MASK     (3 << GMAC_DCFGR_RXBMS_SHIFT)
#  define GMAC_DCFGR_RXBMS_EIGHTH (0 << GMAC_DCFGR_RXBMS_SHIFT) /* 1/2 Kbyte Memory Size */
#  define GMAC_DCFGR_RXBMS_QTR    (1 << GMAC_DCFGR_RXBMS_SHIFT) /* 1Kbyte Memory Size */
#  define GMAC_DCFGR_RXBMS_HALF   (2 << GMAC_DCFGR_RXBMS_SHIFT) /* 2 Kbytes Memory Size */
#  define GMAC_DCFGR_RXBMS_FULL   (3 << GMAC_DCFGR_RXBMS_SHIFT) /* 4 Kbytes Memory Size */

#define GMAC_DCFGR_TXPBMS         (1 << 10) /* Bit 10: Transmitter Packet Buffer Memory Size Select */
#define GMAC_DCFGR_TXCOEN         (1 << 11) /* Bit 11: Transmitter Checksum Generation Offload Enable */
#define GMAC_DCFGR_DRBS_SHIFT     (16)      /* Bits 16-23: DMA Receive Buffer Size */
#define GMAC_DCFGR_DRBS_MASK      (0xff << GMAC_DCFGR_DRBS_SHIFT)
#  define GMAC_DCFGR_DRBS(n)      ((uint32_t)(n) << GMAC_DCFGR_DRBS_SHIFT)
#define GMAC_DCFGR_DDRP           (1 << 24) /* Bit 24: DMA Discard Receive Packets */

/* Transmit Status Register */

#define GMAC_TSR_UBR              (1 << 0)  /* Bit 0:  Used Bit Read */
#define GMAC_TSR_COL              (1 << 1)  /* Bit 1:  Collision Occurred */
#define GMAC_TSR_RLE              (1 << 2)  /* Bit 2:  Retry Limit exceeded */
#define GMAC_TSR_TXGO             (1 << 3)  /* Bit 3:  Transmit Go */
#define GMAC_TSR_TFC              (1 << 4)  /* Bit 4:  Transmit Frame Corruption due to AHB error */
#define GMAC_TSR_TXCOMP           (1 << 5)  /* Bit 5:  Transmit Complete */
#define GMAC_TSR_UND              (1 << 6)  /* Bit 6:  Transmit Underrun */
#define GMAC_TSR_LCO              (1 << 7)  /* Bit 7:  Late Collision Occurred */
#define GMAC_TSR_HRESP            (1 << 8)  /* Bit 8:  HRESP Not OK */

/* Receive Buffer Queue Base Address */

#define GMAC_RBQB_MASK            (0xfffffffc)  /* Bits 2-31: Receive buffer queue base address */

/* Transmit Buffer Queue Base Address */

#define GMAC_TBQB_MASK            (0xfffffffc)  /* Bits 2-31: Transmit buffer queue base address */

/* Receive Status Register */

#define GMAC_RSR_BNA              (1 << 0)  /* Bit 0:  Buffer Not Available */
#define GMAC_RSR_REC              (1 << 1)  /* Bit 1:  Frame Received */
#define GMAC_RSR_RXOVR            (1 << 2)  /* Bit 2:  Receive Overrun */
#define GMAC_RSR_HNO              (1 << 3)  /* Bit 3:  HRESP Not OK */

/* Interrupt Status Register, Interrupt Enable Register,
 * Interrupt Disable Register
 */

#define GMAC_INT_MFS              (1 << 0)  /* Bit 0:  Management Frame Sent */
#define GMAC_INT_RCOMP            (1 << 1)  /* Bit 1:  Receive Complete */
#define GMAC_INT_RXUBR            (1 << 2)  /* Bit 2:  Receive Used Bit Read */
#define GMAC_INT_TXUBR            (1 << 3)  /* Bit 3:  Transmit Used Bit Read */
#define GMAC_INT_TUR              (1 << 4)  /* Bit 4:  Transmit Under Run */
#define GMAC_INT_RLEX             (1 << 5)  /* Bit 5:  Retry Limit Exceeded or Late Collision */
#define GMAC_INT_TFC              (1 << 6)  /* Bit 6:  Transmit Frame Corruption due to AHB error */
#define GMAC_INT_TCOMP            (1 << 7)  /* Bit 7:  Transmit Complete */
#define GMAC_INT_ROVR             (1 << 10) /* Bit 10: Receive Overrun */
#define GMAC_INT_HRESP            (1 << 11) /* Bit 11: HRESP not OK */
#define GMAC_INT_PFNZ             (1 << 12) /* Bit 12: Pause Frame with Non-zero Pause Quantum */
#define GMAC_INT_PTZ              (1 << 13) /* Bit 13: Pause Time Zero */
#define GMAC_INT_PFTR             (1 << 14) /* Bit 14: Pause Frame Transmitted */
#define GMAC_INT_EXINT            (1 << 15) /* Bit 15: External Interrupt (not in ISR) */
#define GMAC_INT_DRQFR            (1 << 18) /* Bit 18: PTP Delay Request Frame Received */
#define GMAC_INT_SFR              (1 << 19) /* Bit 19: PTP Sync Frame Received */
#define GMAC_INT_DRQFT            (1 << 20) /* Bit 20: PTP Delay Request Frame Transmitted */
#define GMAC_INT_SFT              (1 << 21) /* Bit 21: PTP Sync Frame Transmitted */
#define GMAC_INT_PDRQFR           (1 << 22) /* Bit 22: PDelay Request Frame Received */
#define GMAC_INT_PDRSFR           (1 << 23) /* Bit 23: PDelay Response Frame Received */
#define GMAC_INT_PDRQFT           (1 << 24) /* Bit 24: PDelay Request Frame Transmitted */
#define GMAC_INT_PDRSFT           (1 << 25) /* Bit 25: PDelay Response Frame Transmitted */
#define GMAC_INT_SRI              (1 << 26) /* Bit 26: TSU Seconds Register Increment (not in IMR) */
#define GMAC_INT_WOL              (1 << 28) /* Bit 28: Wake On LAN (not in IMR) */

#define GMAC_INT_ALL              (0x17fcfcff)
#define GMAC_INT_UNUSED           (0xe8030300)

/* PHY Maintenance Register */

#define GMAC_MAN_DATA_SHIFT       (0)       /* Bits 0-15: PHY data */
#define GMAC_MAN_DATA_MASK        (0x0000ffff << GMAC_MAN_DATA_SHIFT)
#  define GMAC_MAN_DATA(n)        ((uint32_t)(n) << GMAC_MAN_DATA_SHIFT)
#define GMAC_MAN_WTN_SHIFT        (16)      /* Bits 16-17:  Must be written to b10 */
#define GMAC_MAN_WTN_MASK         (3 << GMAC_MAN_WTN_SHIFT)
#  define GMAC_MAN_WTN            (2 << GMAC_MAN_WTN_SHIFT)
#define GMAC_MAN_REGA_SHIFT       (18)      /* Bits 18-22: Register Address */
#define GMAC_MAN_REGA_MASK        (31 << GMAC_MAN_REGA_SHIFT)
#  define GMAC_MAN_REGA(n)        ((uint32_t)(n) << GMAC_MAN_REGA_SHIFT)
#define GMAC_MAN_PHYA_SHIFT       (23)      /* Bits 23-27: PHY Address */
#define GMAC_MAN_PHYA_MASK        (31 << GMAC_MAN_PHYA_SHIFT)
#  define GMAC_MAN_PHYA(n)        ((uint32_t)(n) << GMAC_MAN_PHYA_SHIFT)
#define GMAC_MAN_OP_SHIFT         (28)      /* Bits 28-29: Operation */
#define GMAC_MAN_OP_MASK          (3 << GMAC_MAN_OP_SHIFT)
#  define GMAC_MAN_READ           (2 << GMAC_MAN_OP_SHIFT)
#  define GMAC_MAN_WRITE          (1 << GMAC_MAN_OP_SHIFT)
#define GMAC_MAN_CLTTO            (1 << 30) /* Bit 30: Clause 22 Operation */
#define GMAC_MAN_WZO              (1 << 31) /* Bit 31: Write ZERO */

/* Received Pause Quantum Register */

#define GMAC_RPQ_MASK             (0x0000ffff) /* Bits 0-15: Received Pause Quantum */

/* Transmit Pause Quantum Register */

#define GMAC_TPQ_MASK             (0x0000ffff) /* Bits 0-15: Transmit Pause Quantum */

/* TX Partial Store and Forward Register */

#define GMAC_TPSF_TPB1ADR_SHIFT   (0)       /* Bits 0-11: Transmit Partial Store and Forward Address */
#define GMAC_TPSF_TPB1ADR_MASK    (0xfff << GMAC_TPSF_TPB1ADR_SHIFT)
#  define GMAC_TPSF_TPB1ADR(n)    ((uint32_t)(n) << GMAC_TPSF_TPB1ADR_SHIFT)
#define GMAC_TPSF_ENTXP           (1 << 31) /* Bit 31: Enable TX Partial Store and Forward Operation */

/* RX Partial Store and Forward Register */

#define GMAC_RPSF_RPB1ADR_SHIFT   (0)       /* Bits 0-11: Receive Partial Store and Forward Address */
#define GMAC_RPSF_RPB1ADR_MASK    (0xfff << GMAC_RPSF_RPB1ADR_SHIFT)
#  define GMAC_RPSF_RPB1ADR(n)    ((uint32_t)(n) << GMAC_RPSF_RPB1ADR_SHIFT)
#define GMAC_RPSF_ENRXP           (1 << 31) /* Bit 31: Enable RX Partial Store and Forward Operation */

/* Hash Register Bottom [31:0] (32-bit value) */

/* Hash Register Top [63:32] (32-bit value) */

/* Specific Address 1 Bottom [31:0] Register (32-bit value) */

/* Specific Address 1 Top [47:32] Register */

#define GMAC_SAT1_MASK            (0x0000ffff) /* Bits 0-15: Specific Address 1 [47:32]  */

/* Specific Address 2 Bottom [31:0] Register (32-bit value) */

/* Specific Address 2 Top [47:32] Register */

#define GMAC_SAT2_MASK            (0x0000ffff) /* Bits 0-15: Specific Address 2 [47:32]  */

/* Specific Address 3 Bottom [31:0] Register (32-bit value) */

/* Specific Address 3 Top [47:32] Register */

#define GMAC_SAT3_MASK            (0x0000ffff) /* Bits 0-15: Specific Address 3 [47:32]  */

/* Specific Address 4 Bottom [31:0] Register (32-bit value) */

/* Specific Address 4 Top [47:32] Register */

#define GMAC_SAT4_MASK            (0x0000ffff) /* Bits 0-15: Specific Address 4 [47:32]  */

/* Type ID Match 1 Register */

#define GMAC_TIDM1_MASK           (0x0000ffff) /* Bits 0-15: Type ID Match 1 */

/* Type ID Match 2 Register */

#define GMAC_TIDM2_MASK           (0x0000ffff) /* Bits 0-15: Type ID Match 2 */

/* Type ID Match 3 Register */

#define GMAC_TIDM3_MASK           (0x0000ffff) /* Bits 0-15: Type ID Match 3 */

/* Type ID Match 4 Register */

#define GMAC_TIDM4_MASK           (0x0000ffff) /* Bits 0-15: Type ID Match 4 */

/* Wake on LAN Register */

#define GMAC_WOL_IP_SHIFT         (0)       /* Bits 0-15: ARP request IP address */
#define GMAC_WOL_IP_MASK          (0x0000ffff << GMAC_WOL_IP_SHIFT)
#define GMAC_WOL_MAG              (1 << 16) /* Bit 16: Magic packet event enable */
#define GMAC_WOL_ARP              (1 << 17) /* Bit 17: ARP request event enable */
#define GMAC_WOL_SA1              (1 << 18) /* Bit 18: Specific address register 1 event enable */
#define GMAC_WOL_MTI              (1 << 19) /* Bit 19: Multicast hash event enable */

/* IPG Stretch Register */

#define GMAC_IPGS_MASK            (0x0000ffff) /* Bits 0-15: Frame Length */

/* Stacked VLAN Register */

#define GMAC_SVLAN_VLANTYP_SHIFT  (0)       /* Bits 0-15: User Defined VLAN_TYPE Field */
#define GMAC_SVLAN_VLANTYP_MASK   (0xffff << GMAC_SVLAN_VLANTYP_SHIFT)
#  define GMAC_SVLAN_VLANTYP(n)   ((uint32_t)(n) << GMAC_SVLAN_VLANTYP_SHIFT)
#define GMAC_SVLAN_ESVLAN         (1 << 31) /* Bit 31: Enable Stacked VLAN Processing Mode */

/* Transmit PFC Pause Register */

#define GMAC_TPFCP_PEV_SHIFT      (0)       /* Bits 0-7: Priority Enable Vector */
#define GMAC_TPFCP_PEV_MASK       (0xff << GMAC_TPFCP_PEV_SHIFT)
#define GMAC_TPFCP_PQ_SHIFT       (8)       /* Bits 8-15: Pause Quantum */
#define GMAC_TPFCP_PQ_MASK        (0xff << GMAC_TPFCP_PQ_SHIFT)

/* Specific Address 1 Mask Bottom [31:0] Register (32-bit mask) */

/* Specific Address 1 Mask Top [47:32] Register */

#define GMAC_SAMT1_MASK           (0x0000ffff) /* Bits 0-15: Specific Address 1 Mask [47:32] */

/* Octets Transmitted [31:0] Register (32-bit value) */

/* Octets Transmitted [47:32] Register */

#define GMAC_OTHI_MASK            (0x0000ffff) /* Bits 0-15: Transmitted Octets [47:32] */

/* Frames Transmitted Register (32-bit value) */

/* Broadcast Frames Transmitted Register (32-bit value) */

/* Multicast Frames Transmitted Register (32-bit value) */

/* Pause Frames Transmitted Register */

#define GMAC_PFT_MASK             (0x0000ffff) /* Bits 0-15: Pause Frames Transmitted */

/* 64 Byte Frames Transmitted Register (32-bit value) */

/* 65 to 127 Byte Frames Transmitted Register (32-bit value) */

/* 128 to 255 Byte Frames Transmitted Register (32-bit value) */

/* 256 to 511 Byte Frames Transmitted Register (32-bit value) */

/* 512 to 1023 Byte Frames Transmitted Register (32-bit value) */

/* 1024 to 1518 Byte Frames Transmitted Register (32-bit value) */

/* Greater Than 1518 Byte Frames Transmitted Register (32-bit value) */

/* Transmit Under Runs Register */

#define GMAC_TUR_MASK             (0x000003ff) /* Bits 0-9: Transmit Under Runs */

/* Single Collision Frames Register */

#define GMAC_SCF_MASK             (0x0003ffff) /* Bits 0-17: Single Collisions */

/* Multiple Collision Frames Register */

#define GMAC_MCF_MASK             (0x0003ffff) /* Bits 0-17: Multiple Collisions */

/* Excessive Collisions Register */

#define GMAC_EC_MASK              (0x000003ff) /* Bits 0-9: Excessive Collisions */

/* Late Collisions Register */

#define GMAC_LC_MASK              (0x000003ff) /* Bits 0-9: Late Collisions */

/* Deferred Transmission Frames Register */

#define GMAC_DTF_MASK             (0x0003ffff) /* Bits 0-17: Deferred Transmission */

/* Carrier Sense Errors Register */

#define GMAC_CSE_MASK             (0x000003ff) /* Bits 0-9: Carrier Sense Error */

/* Octets Received [31:0] Received (32-bit value) */

/* Octets Received [47:32] Received */

#define GMAC_ORHI_MASK            (0x0000ffff) /* Bits 0-15: Received Octets [47:32] */

/* Frames Received Register (32-bit value) */

/* Broadcast Frames Received Register (32-bit value) */

/* Multicast Frames Received Register (32-bit value) */

/* Pause Frames Received Register */

#define GMAC_PFR_MASK             (0x0000ffff) /* Bits 0-15: Pause Frames Received */

/* 64 Byte Frames Received Register (32-bit value) */

/* 65 to 127 Byte Frames Received Register (32-bit value) */

/* 128 to 255 Byte Frames Received Register (32-bit value) */

/* 256 to 511Byte Frames Received Register (32-bit value) */

/* 512 to 1023 Byte Frames Received Register (32-bit value) */

/* 1024 to 1518 Byte Frames Received Register (32-bit value) */

/* 1519 to Maximum Byte Frames Received Register (32-bit value) */

/* Undersize Frames Received Register */

#define GMAC_UFR_MASK             (0x000003ff) /* Bits 0-9: Undersize Frames Received */

/* Oversize Frames Received Register */

#define GMAC_OFR_MASK             (0x000003ff) /* Bits 0-9: Oversized Frames Received */

/* Jabbers Received Register */

#define GMAC_JR_MASK              (0x000003ff) /* Bits 0-9: Jabbers Received */

/* Frame Check Sequence Errors Register */

#define GMAC_FCSE_MASK            (0x000003ff) /* Bits 0-9: Frame Check Sequence Errors */

/* Length Field Frame Errors Register */

#define GMAC_LFFE_MASK            (0x000003ff) /* Bits 0-9: Length Field Frame Errors */

/* Receive Symbol Errors Register */

#define GMAC_RSE_MASK             (0x000003ff) /* Bits 0-9: Receive Symbol Errors */

/* Alignment Errors Register */

#define GMAC_AE_MASK              (0x000003ff) /* Bits 0-9: Alignment Errors */

/* Receive Resource Errors Register */

#define GMAC_RRE_MASK             (0x0003ffff) /* Bits 0-17: Receive Resource Errors */

/* Receive Overrun Register */

#define GMAC_ROE_MASK             (0x000003ff) /* Bits 0-9: Receive Overruns */

/* IP Header Checksum Errors Register */

#define GMAC_IHCE_MASK            (0x000000ff) /* Bits 0-7: IP Header Checksum Errors */

/* TCP Checksum Errors Register */

#define GMAC_TCE_MASK             (0x000000ff) /* Bits 0-7: TCP Header Checksum Errors */

/* UDP Checksum Errors Register */

#define GMAC_UCE_MASK             (0x000000ff) /* Bits 0-7: UDP Header Checksum Errors */

/* 1588 Timer Sync Strobe Seconds Register (32-bit value) */

/* 1588 Timer Sync Strobe Nanoseconds Register */

#define GMAC_TSSN_MASK            (0x3fffffff) /* Bits 0-29: Value Timer Nanoseconds Register Capture */

/* 1588 Timer Seconds Register (32-bit value) */

/* 1588 Timer Nanoseconds Register */

#define GMAC_TN_MASK              (0x3fffffff) /* Bits 0-29: Timer Count in Nanoseconds */

/* 1588 Timer Adjust Register */

#define GMAC_TA_ITDT_SHIFT        (0)       /* Bits 0-29: Increment/Decrement */
#define GMAC_TA_ITDT_MASK         (0x3fffffff)
#define GMAC_TA_ADJ               (1 << 31) /* Bit 31: Adjust 1588 Timer */

/* 1588 Timer Increment Register */

#define GMAC_TI_CNS_SHIFT         (0)       /* Bits 0-7: Count Nanoseconds */
#define GMAC_TI_CNS_MASK          (0xff << GMAC_TI_CNS_SHIFT)
#  define GMAC_TI_CNS(n)          ((uint32_t)(n) << GMAC_TI_CNS_SHIFT)
#define GMAC_TI_ACNS_SHIFT        (8)       /* Bits 8-15: Alternative Count Nanoseconds */
#define GMAC_TI_ACNS_MASK         (0xff << GMAC_TI_ACNS_SHIFT)
#  define GMAC_TI_ACNS(n)         ((uint32_t)(n) << GMAC_TI_ACNS_SHIFT)
#define GMAC_TI_NIT_SHIFT         (16)      /* Bits 16-23: Number of Increments */
#define GMAC_TI_NIT_MASK          (0xff << GMAC_TI_NIT_SHIFT)
#  define GMAC_TI_NIT(n)          ((uint32_t)(n) << GMAC_TI_NIT_SHIFT)

/* PTP Event Frame Transmitted Seconds (32-bit value) */

/* PTP Event Frame Transmitted Nanoseconds */

#define GMAC_EFTN_MASK              (0x3fffffff) /* Bits 0-29: Register Update */

/* PTP Event Frame Received Seconds (32-bit value) */

/* PTP Event Frame Received Nanoseconds */

#define GMAC_EFRN_MASK              (0x3fffffff) /* Bits 0-29: Register Update */

/* PTP Peer Event Frame Transmitted Seconds (32-bit value) */

/* PTP Peer Event Frame Transmitted Nanoseconds */

#define GMAC_PEFTN_MASK              (0x3fffffff) /* Bits 0-29: Register Update */

/* PTP Peer Event Frame Received Seconds (32-bit value) */

/* PTP Peer Event Frame Received Nanoseconds */

#define GMAC_PEFRS_MASK              (0x3fffffff) /* Bits 0-29: Register Update */

/* Interrupt Status Register Priority Queue 0-6
 * Interrupt Enable Register Priority Queue 0-6
 * Interrupt Disable Register Priority Queue 0-6
 * Interrupt Mask Register Priority Queue 0-6
 *
 * Use these definitions:
 *
 *      GMAC_INT_RCOMP          Bit 1:  Receive Complete
 *      GMAC_INT_RXUBR          Bit 2:  Receive Used Bit Read
 *      GMAC_INT_RLEX           Bit 5:  Retry Limit Exceeded or
 *                                      Late Collision
 *      GMAC_INT_TFC            Bit 6:  Transmit Frame Corruption
 *                                      due to AHB error
 *      GMAC_INT_TCOMP          Bit 7:  Transmit Complete
 *      GMAC_INT_ROVR          Bit 10: Receive Overrun
 *      GMAC_INT_HRESP         Bit 11: HRESP not OK
 */

/* Transmit Buffer Queue Base Address Priority Queue 0-6 */

#define GMAC_TBQBAPQ0_MASK        (0xfffffffc)  /* Bits 2-31: Transmit Buffer Queue Base Address */

/* Receive Buffer Queue Base Address Priority Queue 0-6 */

#define GMAC_RBQBAPQ0_MASK        (0xfffffffc)  /* Bits 2-31: Receive Buffer Queue Base Address */

/* Receive Buffer Size Register Priority Queue 0-6 */

#define GMAC_RBSRPQ0_MASK         (0x0000ffff)  /* Bits 0-15: Receive Buffer Size */

/* Screening Type1 Register Priority Queue 0-15 */

#define GMAC_ST1RPQ0_QNB_SHIFT    (0)       /* Bits 0-3: Queue Number (0->7) */
#define GMAC_ST1RPQ0_QNB_MASK     (15 << GMAC_ST1RPQ0_QNB_SHIFT)
#  define GMAC_ST1RPQ0_QNB(n)     ((uint32_t)(n) << GMAC_ST1RPQ0_QNB_SHIFT)
#define GMAC_ST1RPQ0_DSTCM_SHIFT  (4)       /* Bits 4-11: Differentiated Services or Traffic Class Match */
#define GMAC_ST1RPQ0_DSTCM_MASK   (0xff << GMAC_ST1RPQ0_DSTCM_SHIFT)
#  define GMAC_ST1RPQ0_DSTCM(n)   ((uint32_t)(n) << GMAC_ST1RPQ0_DSTCM_SHIFT)
#define GMAC_ST1RPQ0_UDPM_SHIFT   (12)      /* Bits 12-27: UDP Port Match */
#define GMAC_ST1RPQ0_UDPM_MASK    (0xffff << GMAC_ST1RPQ0_UDPM_SHIFT)
#  define GMAC_ST1RPQ0_UDPM(n)    ((uint32_t)(n) << GMAC_ST1RPQ0_UDPM_SHIFT)
#define GMAC_ST1RPQ0_DSTCE        (1 << 28) /* Bit 28: Differentiated Services or Traffic Class Match Enable */
#define GMAC_ST1RPQ0_UDPE         (1 << 29) /* Bit 29: UDP Port Match Enable */

/* Screening Type2 Register Priority Queue 0-15 */

#define GMAC_ST2RPQ0_QNB_SHIFT    (0)       /* Bits 0-3: Queue Number (0->7) */
#define GMAC_ST2RPQ0_QNB_MASK     (15 << GMAC_ST2RPQ0_QNB_SHIFT)
#  define GMAC_ST2RPQ0_QNB(n)     ((uint32_t)(n) << GMAC_ST2RPQ0_QNB_SHIFT)
#define GMAC_ST2RPQ0_VLANP_SHIFT  (4)       /* Bits 4-7: VLAN Priority */
#define GMAC_ST2RPQ0_VLANP_MASK   (15 << GMAC_ST2RPQ0_VLANP_SHIFT)
#  define GMAC_ST2RPQ0_VLANP(n)   ((uint32_t)(n) << GMAC_ST2RPQ0_VLANP_SHIFT)
#define GMAC_ST2RPQ0_VLANE        (1 << 8)  /* Bit 8:  VLAN Enable */

/* Descriptors **************************************************************/

/* Receive buffer descriptor:  Address word */

#define GMACRXD_ADDR_OWNER        (1 << 0)     /* Bit 0:  1=Software owns; 0=GMAC owns */
#define GMACRXD_ADDR_WRAP         (1 << 1)     /* Bit 1:  Last descriptor in list */
#define GMACRXD_ADDR_MASK         (0xfffffffc) /* Bits 2-31: Aligned buffer address */

/* Receive buffer descriptor:  Control word */

#define GMACRXD_STA_FRLEN_SHIFT   (0)       /* Bits 0-12: Length of frame */
#define GMACRXD_STA_FRLEN_MASK    (0x00000fff << GMACRXD_STA_FRLEN_SHIFT)
#define GMACRXD_STA_JFRLEN_SHIFT  (0)      /* Bits 0-13: Length of jumbo frame */
#define GMACRXD_STA_JFRLEN_MASK   (0x00001fff << GMACRXD_STA_JFRLEN_SHIFT)
#define GMACRXD_STA_BADFCS        (1 << 13) /* Bit 13: Frame had bad FCS */
#define GMACRXD_STA_SOF           (1 << 14) /* Bit 14: Start of frame */
#define GMACRXD_STA_EOF           (1 << 15) /* Bit 15: End of frame */
#define GMACRXD_STA_CFI           (1 << 16) /* Bit 16: Canonical format indicator (CFI) bit */
#define GMACRXD_STA_VLPRIO_SHIFT  (17)      /* Bits 17-19: VLAN priority */
#define GMACRXD_STA_VLPRIO_MASK   (7 << GMACRXD_STA_VLANPRIO_SHIFT)
#define GMACRXD_STA_PRIODET       (1 << 20) /* Bit 20: Priority tag detected */
#define GMACRXD_STA_VLANTAG       (1 << 21) /* Bit 21: VLAN tag detected */

#define GMACRXD_STA_TYPID_SHIFT   (22)      /* Bits 22-23: Type ID register match */
#define GMACRXD_STA_TYPID_MASK    (3 << GMACRXD_STA_TYPID_SHIFT)
#  define GMACRXD_STA_TYPID1      (0 << GMACRXD_STA_TYPID_SHIFT) /* Type ID register 1 match */
#  define GMACRXD_STA_TYPID2      (1 << GMACRXD_STA_TYPID_SHIFT) /* Type ID register 2 match */
#  define GMACRXD_STA_TYPID3      (2 << GMACRXD_STA_TYPID_SHIFT) /* Type ID register 3 match */
#  define GMACRXD_STA_TYPID4      (3 << GMACRXD_STA_TYPID_SHIFT) /* Type ID register 4 match */

#define GMACRXD_STA_SNAP_SHIFT    (22) /* Bits 22-23: Specific Address Register match */
#define GMACRXD_STA_SNAP_MASK     (3 << GMACRXD_STA_SNAP_SHIFT)
#  define GMACRXD_STA_SNAP_NOCHK  (0 << GMACRXD_STA_SNAP_SHIFT) /* Checksum not checked */
#  define GMACRXD_STA_SNAP_IPCHK  (1 << GMACRXD_STA_SNAP_SHIFT) /* IP header checksum checked */
#  define GMACRXD_STA_SNAP_TCPCHK (2 << GMACRXD_STA_SNAP_SHIFT) /* IP header and TCP checksum checked */
#  define GMACRXD_STA_SNAP_UDPCHK (3 << GMACRXD_STA_SNAP_SHIFT) /* IP header and UDP checksum checked */

#define GMACRXD_STA_TYPID         (1 << 24) /* Bit 24: Type ID match found */
#define GMACRXD_STA_SNAP          (1 << 24) /* Bit 24: Frame was SNAP encoded */
#define GMACRXD_STA_ADDR_SHIFT    (25)      /* Bits 25-26: Specific Address Register match */
#define GMACRXD_STA_ADDR_MASK     (3 << GMACRXD_STA_ADDR_SHIFT)
#  define GMACRXD_STA_ADDR1_MATCH (0 << GMACRXD_STA_ADDR_SHIFT) /* Specific address register 1 match */
#  define GMACRXD_STA_ADDR2_MATCH (1 << GMACRXD_STA_ADDR_SHIFT) /* Specific address register 2 match */
#  define GMACRXD_STA_ADDR3_MATCH (2 << GMACRXD_STA_ADDR_SHIFT) /* Specific address register 3 match */
#  define GMACRXD_STA_ADDR4_MATCH (3 << GMACRXD_STA_ADDR_SHIFT) /* Specific address register 4 match */

#define GMACRXD_STA_ADDRMATCH     (1 << 27) /* Bit 27: Specific Address Register match found */
                                            /* Bit 28: Reserved */
#define GMACRXD_STA_UCAST         (1 << 29) /* Bit 29: Unicast hash match */
#define GMACRXD_STA_MCAST         (1 << 30) /* Bit 30: Multicast hash match */
#define GMACRXD_STA_BCAST         (1 << 31) /* Bit 31: Global all ones broadcast address detected */

/* Transmit buffer descriptor:  Address word (un-aligned, 32-bit address */

/* Transmit buffer descriptor:  Control word */

#define GMACTXD_STA_BUFLEN_SHIFT  (0)       /* Bits 0-13: Length of buffer */
#define GMACTXD_STA_BUFLEN_MASK   (0x00003fff << GMACTXD_STA_BUFLEN_SHIFT)
                                            /* Bit 14: Reserved */
#define GMACTXD_STA_LAST          (1 << 15) /* Bit 15: Last buffer in the current frame */
#define GMACTXD_STA_NOCRC         (1 << 16) /* Bit 16: No CRC */
                                            /* Bits 17-19: Reserved */
#define GMACTXD_STA_CKERR_SHIFT   (20)      /* Bits 20-22: Transmit checksum generation errors */
#define GMACTXD_STA_CKERR_MASK    (7 << GMACTXD_STA_CKERR_SHIFT)
#  define GMACTXD_STA_CKERR_OK    (0 << GMACTXD_STA_CKERR_SHIFT) /* No Error */
#  define GMACTXD_STA_CKERR_VLAN  (1 << GMACTXD_STA_CKERR_SHIFT) /* VLAN header error */
#  define GMACTXD_STA_CKERR_SNAP  (2 << GMACTXD_STA_CKERR_SHIFT) /* SNAP header error */
#  define GMACTXD_STA_CKERR_IP    (3 << GMACTXD_STA_CKERR_SHIFT) /* Bad IP type */
#  define GMACTXD_STA_CKERR_UNK   (4 << GMACTXD_STA_CKERR_SHIFT) /* Not VLAN, SNAP or IP */
#  define GMACTXD_STA_CKERR_FRAG  (5 << GMACTXD_STA_CKERR_SHIFT) /* Bad packet fragmentation */
#  define GMACTXD_STA_CKERR_PROTO (6 << GMACTXD_STA_CKERR_SHIFT) /* Not TCP or UDP */
#  define GMACTXD_STA_CKERR_END   (7 << GMACTXD_STA_CKERR_SHIFT) /* Premature end of packet */

                                            /* Bits 23-25: Reserved */
#define GMACTXD_STA_LCOL          (1 << 26) /* Bit 26: Late collision */
#define GMACTXD_STA_TFC           (1 << 27) /* Bit 27: Transmit Frame Corruption due to AHB error */
#define GMACTXD_STA_TXUR          (1 << 28) /* Bit 28: Transmit underrun */
#define GMACTXD_STA_TXERR         (1 << 29) /* Bit 29: Retry limit exceeded, transmit error detected */
#define GMACTXD_STA_WRAP          (1 << 30) /* Bit 30: Last descriptor in descriptor list */
#define GMACTXD_STA_USED          (1 << 31) /* Bit 31: Zero for the GMAC to read from buffer */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Receive buffer descriptor */

struct gmac_rxdesc_s
{
  uint32_t addr;     /* Buffer address */
  uint32_t status;   /* RX status and controls */
};

/* Transmit buffer descriptor */

struct gmac_txdesc_s
{
  uint32_t addr;     /* Buffer address */
  uint32_t status;   /* TX status and controls */
};

#endif /* __ARCH_ARM_SRC_SAMA5E5_HARDWARE_SAM_GMAC_H */
