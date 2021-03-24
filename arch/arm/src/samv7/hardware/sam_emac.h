/****************************************************************************
 * arch/arm/src/samv7/hardware/sam_emac.h
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

/* This is the form of the EMAC interface used the SAMV7.
 * This is referred as GMAC in the documentation even though it does not
 * support Gibabit Ethernet.
 */

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_EMAC_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_EMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "hardware/sam_memorymap.h"

#if SAMV7_NEMAC > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EMAC Register Offsets ****************************************************/

#define SAM_EMAC_NCR_OFFSET       0x0000 /* Network Control Register */
#define SAM_EMAC_NCFGR_OFFSET     0x0004 /* Network Configuration Register */
#define SAM_EMAC_NSR_OFFSET       0x0008 /* Network Status Register */
#define SAM_EMAC_UR_OFFSET        0x000c /* User Register */
#define SAM_EMAC_DCFGR_OFFSET     0x0010 /* DMA Configuration Register */
#define SAM_EMAC_TSR_OFFSET       0x0014 /* Transmit Status Register */
#define SAM_EMAC_RBQB_OFFSET      0x0018 /* Receive Buffer Queue Base Address */
#define SAM_EMAC_TBQB_OFFSET      0x001c /* Transmit Buffer Queue Base Address */
#define SAM_EMAC_RSR_OFFSET       0x0020 /* Receive Status Register */
#define SAM_EMAC_ISR_OFFSET       0x0024 /* Interrupt Status Register */
#define SAM_EMAC_IER_OFFSET       0x0028 /* Interrupt Enable Register */
#define SAM_EMAC_IDR_OFFSET       0x002c /* Interrupt Disable Register */
#define SAM_EMAC_IMR_OFFSET       0x0030 /* Interrupt Mask Register */
#define SAM_EMAC_MAN_OFFSET       0x0034 /* PHY Maintenance Register */
#define SAM_EMAC_RPQ_OFFSET       0x0038 /* Received Pause Quantum Register */
#define SAM_EMAC_TPQ_OFFSET       0x003c /* Transmit Pause Quantum Register */
#define SAM_EMAC_TPSF_OFFSET      0x0040 /* TX Partial Store and Forward Register */
#define SAM_EMAC_RPSF_OFFSET      0x0044 /* RX Partial Store and Forward Register */
#define SAM_EMAC_RJFML_OFFSET     0x0048 /* RX Jumbo Frame Max Length Register */
                                         /* 0x004c-0x007c: Reserved */
#define SAM_EMAC_HRB_OFFSET       0x0080 /* Hash Register Bottom [31:0] Register */
#define SAM_EMAC_HRT_OFFSET       0x0084 /* Hash Register Top [63:32] Register */
#define SAM_EMAC_SAB1_OFFSET      0x0088 /* Specific Address 1 Bottom Register */
#define SAM_EMAC_SAT1_OFFSET      0x008c /* Specific Address 1 Top Register */
#define SAM_EMAC_SAB2_OFFSET      0x0090 /* Specific Address 2 Bottom Register */
#define SAM_EMAC_SAT2_OFFSET      0x0094 /* Specific Address 2 Top Register */
#define SAM_EMAC_SAB3_OFFSET      0x0098 /* Specific Address 3 Bottom Register */
#define SAM_EMAC_SAT3_OFFSET      0x009c /* Specific Address 3 Top Register */
#define SAM_EMAC_SAB4_OFFSET      0x00a0 /* Specific Address 4 Bottom Register */
#define SAM_EMAC_SAT4_OFFSET      0x00a4 /* Specific Address 4 Top Register */
#define SAM_EMAC_TIDM1_OFFSET     0x00a8 /* Type ID Match 1 Register */
#define SAM_EMAC_TIDM2_OFFSET     0x00ac /* Type ID Match 2 Register */
#define SAM_EMAC_TIDM3_OFFSET     0x00b0 /* Type ID Match 3 Register */
#define SAM_EMAC_TIDM4_OFFSET     0x00b4 /* Type ID Match 4 Register */
#define SAM_EMAC_WOL_OFFSET       0x00b8 /* Wake on LAN Register */
#define SAM_EMAC_IPGS_OFFSET      0x00bc /* IPG Stretch Register */
#define SAM_EMAC_SVLAN_OFFSET     0x00c0 /* Stacked VLAN Register */
#define SAM_EMAC_TPFCP_OFFSET     0x00c4 /* Transmit PFC Pause Register */
#define SAM_EMAC_SAMB1_OFFSET     0x00c8 /* Specific Address 1 Mask Bottom [31:0] Register */
#define SAM_EMAC_SAMT1_OFFSET     0x00cc /* Specific Address 1 Mask Top [47:32] Register */
                                         /* 0x00d0-0xd8: Reserved */

/* PTP/1588 Timer Registers */

#define SAM_EMAC_NSC_OFFSET       0x00dc /* 1588 Timer Nanosecond Comparison Register */
#define SAM_EMAC_SCL_OFFSET       0x00e0 /* 1588 Timer Second Comparison Low Register */
#define SAM_EMAC_SCH_OFFSET       0x00e4 /* 1588 Timer Second Comparison High Register */
#define SAM_EMAC_EFTSH_OFFSET     0x00e8 /* PTP Event Frame Transmitted Seconds High Register */
#define SAM_EMAC_EFRSH_OFFSET     0x00ec /* PTP Event Frame Received Seconds High Register */
#define SAM_EMAC_PEFTSH_OFFSET    0x00f0 /* PTP Peer Event Frame Transmitted Seconds High Register */
#define SAM_EMAC_PEFRSH_OFFSET    0x00f4 /* PTP Peer Event Frame Received Seconds High Register */
                                         /* 0x00f8-0x00fc: Reserved */

/* Statistics registers */

#define SAM_EMAC_OTLO_OFFSET      0x0100 /* Octets Transmitted [31:0] Register */
#define SAM_EMAC_OTHI_OFFSET      0x0104 /* Octets Transmitted [47:32] Register */
#define SAM_EMAC_FT_OFFSET        0x0108 /* Frames Transmitted Register */
#define SAM_EMAC_BCFT_OFFSET      0x010c /* Broadcast Frames Transmitted Register */
#define SAM_EMAC_MFT_OFFSET       0x0110 /* Multicast Frames Transmitted Register */
#define SAM_EMAC_PFT_OFFSET       0x0114 /* Pause Frames Transmitted Register */
#define SAM_EMAC_BFT64_OFFSET     0x0118 /* 64 Byte Frames Transmitted Register */
#define SAM_EMAC_TBFT127_OFFSET   0x011c /* 65 to 127 Byte Frames Transmitted Register */
#define SAM_EMAC_TBFT255_OFFSET   0x0120 /* 128 to 255 Byte Frames Transmitted Register */
#define SAM_EMAC_TBFT511_OFFSET   0x0124 /* 256 to 511 Byte Frames Transmitted Register */
#define SAM_EMAC_TBFT1023_OFFSET  0x0128 /* 512 to 1023 Byte Frames Transmitted Register */
#define SAM_EMAC_TBFT1518_OFFSET  0x012c /* 1024 to 1518 Byte Frames Transmitted Register */
#define SAM_EMAC_GTBFT1518_OFFSET 0x0130 /* Greater Than 1518 Byte Frames Transmitted Register */
#define SAM_EMAC_TUR_OFFSET       0x0134 /* Transmit Under Runs Register */
#define SAM_EMAC_SCF_OFFSET       0x0138 /* Single Collision Frames Register */
#define SAM_EMAC_MCF_OFFSET       0x013c /* Multiple Collision Frames Register */
#define SAM_EMAC_EC_OFFSET        0x0140 /* Excessive Collisions Register */
#define SAM_EMAC_LC_OFFSET        0x0144 /* Late Collisions Register */
#define SAM_EMAC_DTF_OFFSET       0x0148 /* Deferred Transmission Frames Register */
#define SAM_EMAC_CSE_OFFSET       0x014c /* Carrier Sense Errors Register */
#define SAM_EMAC_ORLO_OFFSET      0x0150 /* Octets Received [31:0] Received */
#define SAM_EMAC_ORHI_OFFSET      0x0154 /* Octets Received [47:32] Received */
#define SAM_EMAC_FR_OFFSET        0x0158 /* Frames Received Register */
#define SAM_EMAC_BCFR_OFFSET      0x015C /* Broadcast Frames Received Register */
#define SAM_EMAC_MFR_OFFSET       0x0160 /* Multicast Frames Received Register */
#define SAM_EMAC_PFR_OFFSET       0x0164 /* Pause Frames Received Register */
#define SAM_EMAC_BFR64_OFFSET     0x0168 /* 64 Byte Frames Received Register */
#define SAM_EMAC_TBFR127_OFFSET   0x016c /* 65 to 127 Byte Frames Received Register */
#define SAM_EMAC_TBFR255_OFFSET   0x0170 /* 128 to 255 Byte Frames Received Register */
#define SAM_EMAC_TBFR511_OFFSET   0x0174 /* 256 to 511Byte Frames Received Register */
#define SAM_EMAC_TBFR1023_OFFSET  0x0178 /* 512 to 1023 Byte Frames Received Register */
#define SAM_EMAC_TBFR1518_OFFSET  0x017c /* 1024 to 1518 Byte Frames Received Register */
#define SAM_EMAC_TMXBFR_OFFSET    0x0180 /* 1519 to Maximum Byte Frames Received Register */
#define SAM_EMAC_UFR_OFFSET       0x0184 /* Undersize Frames Received Register */
#define SAM_EMAC_OFR_OFFSET       0x0188 /* Oversize Frames Received Register */
#define SAM_EMAC_JR_OFFSET        0x018c /* Jabbers Received Register */
#define SAM_EMAC_FCSE_OFFSET      0x0190 /* Frame Check Sequence Errors Register */
#define SAM_EMAC_LFFE_OFFSET      0x0194 /* Length Field Frame Errors Register */
#define SAM_EMAC_RSE_OFFSET       0x0198 /* Receive Symbol Errors Register */
#define SAM_EMAC_AE_OFFSET        0x019c /* Alignment Errors Register */
#define SAM_EMAC_RRE_OFFSET       0x01a0 /* Receive Resource Errors Register */
#define SAM_EMAC_ROE_OFFSET       0x01a4 /* Receive Overrun Register */
#define SAM_EMAC_IHCE_OFFSET      0x01a8 /* IP Header Checksum Errors Register */
#define SAM_EMAC_TCE_OFFSET       0x01ac /* TCP Checksum Errors Register */
#define SAM_EMAC_UCE_OFFSET       0x01b0 /* UDP Checksum Errors Register */
                                         /* 0x01b4-0x01b8: Reserved */

/* PTP/1588 Timer Registers */

#define SAM_EMAC_TISUBN_OFFSET    0x01bc /* 1588 Timer Increment Sub-nanoseconds Register */
#define SAM_EMAC_TSH_OFFSET       0x01c0 /* 1588 Timer Seconds High Register */
#define SAM_EMAC_TSL_OFFSET       0x01d0 /* 1588 Timer Seconds Low Register */
#define SAM_EMAC_TN_OFFSET        0x01d4 /* 1588 Timer Nanoseconds Register */
#define SAM_EMAC_TA_OFFSET        0x01d8 /* 1588 Timer Adjust Register */
#define SAM_EMAC_TI_OFFSET        0x01dc /* 1588 Timer Increment Register */
#define SAM_EMAC_EFTSL_OFFSET     0x01e0 /* PTP Event Frame Transmitted Seconds Low */
#define SAM_EMAC_EFTN_OFFSET      0x01e4 /* PTP Event Frame Transmitted Nanoseconds */
#define SAM_EMAC_EFRSL_OFFSET     0x01e8 /* PTP Event Frame Received Seconds Low */
#define SAM_EMAC_EFRN_OFFSET      0x01ec /* PTP Event Frame Received Nanoseconds */
#define SAM_EMAC_PEFTSL_OFFSET    0x01f0 /* PTP Peer Event Frame Transmitted Seconds Low */
#define SAM_EMAC_PEFTN_OFFSET     0x01f4 /* PTP Peer Event Frame Transmitted Nanoseconds */
#define SAM_EMAC_PEFRSL_OFFSET    0x01f8 /* PTP Peer Event Frame Received Seconds Low */
#define SAM_EMAC_PEFRN_OFFSET     0x01fc /* PTP Peer Event Frame Received Nanoseconds */
                                         /* 0x0200-0x03fc: Reserved */

/* Priority Queue */

#define SAM_EMAC_ISRPQ_ISRPQ_OFFSET(n)    (0x03fc+((n)<<2)) /* Interrupt Status Register Priority Queue, n=1-3 */
#define SAM_EMAC_ISRPQ_TBQBAPQ_OFFSET(n)  (0x043c+((n)<<2)) /* Transmit Buffer Queue Base Address Register Priority Queue, n=1-3 */
#define SAM_EMAC_ISRPQ_RBQBAPQ_OFFSET(n)  (0x047c+((n)<<2)) /* Receive Buffer Queue Base Address Register Priority Queue, n=1-3 */
#define SAM_EMAC_ISRPQ_RBSRPQ_OFFSET(n)   (0x049c+((n)<<2)) /* Receive Buffer Size Register Priority Queue, n=1-3 */
#define SAM_EMAC_ISRPQ_CBSCR_OFFSET       0x04bc            /* Credit-Based Shaping Control Register */
#define SAM_EMAC_ISRPQ_CBSISQA_OFFSET     0x04c0            /* Credit-Based Shaping IdleSlope Register for Queue A */
#define SAM_EMAC_ISRPQ_CBSISQB_OFFSET     0x04c4            /* Credit-Based Shaping IdleSlope Register for Queue B */
#define SAM_EMAC_ISRPQ_ST1RPQ_OFFSET(n)   (0x0500+((n)<<2)) /* Screening Type 1 Register Priority Queue, 0=1-3 */
#define SAM_EMAC_ISRPQ_ST2RPQ_OFFSET(n)   (0x0540+((n)<<2)) /* Screening Type 2 Register Priority Queue, 0=1-7 */
#define SAM_EMAC_ISRPQ_IERPQ_OFFSET(n)    (0x05fc+((n)<<2)) /* Interrupt Enable Register Priority Queue, n=1-3 */
#define SAM_EMAC_ISRPQ_IDRPQ_OFFSET(n)    (0x061c+((n)<<2)) /* Interrupt Disable Register Priority Queue, n=1-3 */
#define SAM_EMAC_ISRPQ_IMRPQ_OFFSET(n)    (0x063c+((n)<<2)) /* Interrupt Mask Register Priority Queue, n=1-3 */
#define SAM_EMAC_ISRPQ_ST2ER_OFFSET(n)    (0x06e0+((n)<<2)) /* Screening Type 2 Ethertype Register, n=0-3 */
#define SAM_EMAC_ISRPQ_ST2CW0_OFFSET(n)   (0x0700+((n)<<3)) /* Screening Type 2 Compare Word 0 Registerm, n=0-23 */
#define SAM_EMAC_ISRPQ_ST2CW1_OFFSET(n)   (0x0704+((n)<<3)) /* Screening Type 2 Compare Word 1 Register, n=0-23 */

/* EMAC Register Addresses **************************************************/

/* EMAC0 base addresses */

#define SAM_EMAC0_NCR             (SAM_EMAC0_BASE+SAM_EMAC_NCR_OFFSET)
#define SAM_EMAC0_NCFGR           (SAM_EMAC0_BASE+SAM_EMAC_NCFGR_OFFSET)
#define SAM_EMAC0_NSR             (SAM_EMAC0_BASE+SAM_EMAC_NSR_OFFSET)
#define SAM_EMAC0_TSR             (SAM_EMAC0_BASE+SAM_EMAC_TSR_OFFSET)
#define SAM_EMAC0_UR              (SAM_EMAC0_BASE+SAM_EMAC_UR_OFFSET)
#define SAM_EMAC0_DCFGR           (SAM_EMAC0_BASE+SAM_EMAC_DCFGR_OFFSET)
#define SAM_EMAC0_RBQB            (SAM_EMAC0_BASE+SAM_EMAC_RBQB_OFFSET)
#define SAM_EMAC0_TBQB            (SAM_EMAC0_BASE+SAM_EMAC_TBQB_OFFSET)
#define SAM_EMAC0_RSR             (SAM_EMAC0_BASE+SAM_EMAC_RSR_OFFSET)
#define SAM_EMAC0_ISR             (SAM_EMAC0_BASE+SAM_EMAC_ISR_OFFSET)
#define SAM_EMAC0_IER             (SAM_EMAC0_BASE+SAM_EMAC_IER_OFFSET)
#define SAM_EMAC0_IDR             (SAM_EMAC0_BASE+SAM_EMAC_IDR_OFFSET)
#define SAM_EMAC0_IMR             (SAM_EMAC0_BASE+SAM_EMAC_IMR_OFFSET)
#define SAM_EMAC0_MAN             (SAM_EMAC0_BASE+SAM_EMAC_MAN_OFFSET)
#define SAM_EMAC0_RPQ             (SAM_EMAC0_BASE+SAM_EMAC_RPQ_OFFSET)
#define SAM_EMAC0_TPQ             (SAM_EMAC0_BASE+SAM_EMAC_TPQ_OFFSET)
#define SAM_EMAC0_TPSF            (SAM_EMAC0_BASE+SAM_EMAC_TPSF_OFFSET)
#define SAM_EMAC0_RPSF            (SAM_EMAC0_BASE+SAM_EMAC_RPSF_OFFSET)
#define SAM_EMAC0_RJFML           (SAM_EMAC0_BASE+SAM_EMAC_RJFML_OFFSET)
#define SAM_EMAC0_HRB             (SAM_EMAC0_BASE+SAM_EMAC_HRB_OFFSET)
#define SAM_EMAC0_HRT             (SAM_EMAC0_BASE+SAM_EMAC_HRT_OFFSET)
#define SAM_EMAC0_SAB1            (SAM_EMAC0_BASE+SAM_EMAC_SAB1_OFFSET)
#define SAM_EMAC0_SAT1            (SAM_EMAC0_BASE+SAM_EMAC_SAT1_OFFSET)
#define SAM_EMAC0_SAB2            (SAM_EMAC0_BASE+SAM_EMAC_SAB2_OFFSET)
#define SAM_EMAC0_SAT2            (SAM_EMAC0_BASE+SAM_EMAC_SAT2_OFFSET)
#define SAM_EMAC0_SAB3            (SAM_EMAC0_BASE+SAM_EMAC_SAB3_OFFSET)
#define SAM_EMAC0_SAT3            (SAM_EMAC0_BASE+SAM_EMAC_SAT3_OFFSET)
#define SAM_EMAC0_SAB4            (SAM_EMAC0_BASE+SAM_EMAC_SAB4_OFFSET)
#define SAM_EMAC0_SAT4            (SAM_EMAC0_BASE+SAM_EMAC_SAT4_OFFSET)
#define SAM_EMAC0_TIDM1           (SAM_EMAC0_BASE+SAM_EMAC_TIDM1_OFFSET)
#define SAM_EMAC0_TIDM2           (SAM_EMAC0_BASE+SAM_EMAC_TIDM2_OFFSET)
#define SAM_EMAC0_TIDM3           (SAM_EMAC0_BASE+SAM_EMAC_TIDM3_OFFSET)
#define SAM_EMAC0_TIDM4           (SAM_EMAC0_BASE+SAM_EMAC_TIDM4_OFFSET)
#define SAM_EMAC0_IPGS            (SAM_EMAC0_BASE+SAM_EMAC_IPGS_OFFSET)
#define SAM_EMAC0_SVLAN           (SAM_EMAC0_BASE+SAM_EMAC_SVLAN_OFFSET)
#define SAM_EMAC0_TPFCP           (SAM_EMAC0_BASE+SAM_EMAC_TPFCP_OFFSET)
#define SAM_EMAC0_SAMB1           (SAM_EMAC0_BASE+SAM_EMAC_SAMB1_OFFSET)
#define SAM_EMAC0_SAMT1           (SAM_EMAC0_BASE+SAM_EMAC_SAMT1_OFFSET)

/* PTP/1588 Timer Registers */

#define SAM_EMAC0_NSC             (SAM_EMAC0_BASE+SAM_EMAC_NSC_OFFSET)
#define SAM_EMAC0_SCL             (SAM_EMAC0_BASE+SAM_EMAC_SCL_OFFSET)
#define SAM_EMAC0_SCH             (SAM_EMAC0_BASE+SAM_EMAC_SCH_OFFSET)
#define SAM_EMAC0_EFTSH           (SAM_EMAC0_BASE+SAM_EMAC_EFTSH_OFFSET)
#define SAM_EMAC0_EFRSH           (SAM_EMAC0_BASE+SAM_EMAC_EFRSH_OFFSET)
#define SAM_EMAC0_PEFTSH          (SAM_EMAC0_BASE+SAM_EMAC_PEFTSH_OFFSET)
#define SAM_EMAC0_PEFRSH          (SAM_EMAC0_BASE+SAM_EMAC_PEFRSH_OFFSET)

/* Statistics registers */

#define SAM_EMAC0_OTLO            (SAM_EMAC0_BASE+SAM_EMAC_OTLO_OFFSET)
#define SAM_EMAC0_OTHI            (SAM_EMAC0_BASE+SAM_EMAC_OTHI_OFFSET)
#define SAM_EMAC0_FT              (SAM_EMAC0_BASE+SAM_EMAC_FT_OFFSET)
#define SAM_EMAC0_BCFT            (SAM_EMAC0_BASE+SAM_EMAC_BCFT_OFFSET)
#define SAM_EMAC0_MFT             (SAM_EMAC0_BASE+SAM_EMAC_MFT_OFFSET)
#define SAM_EMAC0_PFT             (SAM_EMAC0_BASE+SAM_EMAC_PFT_OFFSET)
#define SAM_EMAC0_BFT64           (SAM_EMAC0_BASE+SAM_EMAC_BFT64_OFFSET)
#define SAM_EMAC0_TBFT127         (SAM_EMAC0_BASE+SAM_EMAC_TBFT127_OFFSET)
#define SAM_EMAC0_TBFT255         (SAM_EMAC0_BASE+SAM_EMAC_TBFT255_OFFSET)
#define SAM_EMAC0_TBFT511         (SAM_EMAC0_BASE+SAM_EMAC_TBFT511_OFFSET)
#define SAM_EMAC0_TBFT1023        (SAM_EMAC0_BASE+SAM_EMAC_TBFT1023_OFFSET)
#define SAM_EMAC0_TBFT1518        (SAM_EMAC0_BASE+SAM_EMAC_TBFT1518_OFFSET)
#define SAM_EMAC0_GTBFT1518       (SAM_EMAC0_BASE+SAM_EMAC_GTBFT1518_OFFSET)
#define SAM_EMAC0_TUR             (SAM_EMAC0_BASE+SAM_EMAC_TUR_OFFSET)
#define SAM_EMAC0_SCF             (SAM_EMAC0_BASE+SAM_EMAC_SCF_OFFSET)
#define SAM_EMAC0_MCF             (SAM_EMAC0_BASE+SAM_EMAC_MCF_OFFSET)
#define SAM_EMAC0_EC              (SAM_EMAC0_BASE+SAM_EMAC_EC_OFFSET)
#define SAM_EMAC0_LC              (SAM_EMAC0_BASE+SAM_EMAC_LC_OFFSET)
#define SAM_EMAC0_DTF             (SAM_EMAC0_BASE+SAM_EMAC_DTF_OFFSET)
#define SAM_EMAC0_CSE             (SAM_EMAC0_BASE+SAM_EMAC_CSE_OFFSET)
#define SAM_EMAC0_ORLO            (SAM_EMAC0_BASE+SAM_EMAC_ORLO_OFFSET)
#define SAM_EMAC0_ORHI            (SAM_EMAC0_BASE+SAM_EMAC_ORHI_OFFSET)
#define SAM_EMAC0_FR              (SAM_EMAC0_BASE+SAM_EMAC_FR_OFFSET)
#define SAM_EMAC0_BCFR            (SAM_EMAC0_BASE+SAM_EMAC_BCFR_OFFSET)
#define SAM_EMAC0_MFR             (SAM_EMAC0_BASE+SAM_EMAC_MFR_OFFSET)
#define SAM_EMAC0_PFR             (SAM_EMAC0_BASE+SAM_EMAC_PFR_OFFSET)
#define SAM_EMAC0_BFR64           (SAM_EMAC0_BASE+SAM_EMAC_BFR64_OFFSET)
#define SAM_EMAC0_TBFR127         (SAM_EMAC0_BASE+SAM_EMAC_TBFR127_OFFSET)
#define SAM_EMAC0_TBFR255         (SAM_EMAC0_BASE+SAM_EMAC_TBFR255_OFFSET)
#define SAM_EMAC0_TBFR511         (SAM_EMAC0_BASE+SAM_EMAC_TBFR511_OFFSET)
#define SAM_EMAC0_TBFR1023        (SAM_EMAC0_BASE+SAM_EMAC_TBFR1023_OFFSET)
#define SAM_EMAC0_TBFR1518        (SAM_EMAC0_BASE+SAM_EMAC_TBFR1518_OFFSET)
#define SAM_EMAC0_TMXBFR          (SAM_EMAC0_BASE+SAM_EMAC_TMXBFR_OFFSET)
#define SAM_EMAC0_UFR             (SAM_EMAC0_BASE+SAM_EMAC_UFR_OFFSET)
#define SAM_EMAC0_OFR             (SAM_EMAC0_BASE+SAM_EMAC_OFR_OFFSET)
#define SAM_EMAC0_JR              (SAM_EMAC0_BASE+SAM_EMAC_JR_OFFSET)
#define SAM_EMAC0_FCSE            (SAM_EMAC0_BASE+SAM_EMAC_FCSE_OFFSET)
#define SAM_EMAC0_LFFE            (SAM_EMAC0_BASE+SAM_EMAC_LFFE_OFFSET)
#define SAM_EMAC0_RSE             (SAM_EMAC0_BASE+SAM_EMAC_RSE_OFFSET)
#define SAM_EMAC0_AE              (SAM_EMAC0_BASE+SAM_EMAC_AE_OFFSET)
#define SAM_EMAC0_RRE             (SAM_EMAC0_BASE+SAM_EMAC_RRE_OFFSET)
#define SAM_EMAC0_ROE             (SAM_EMAC0_BASE+SAM_EMAC_ROE_OFFSET)
#define SAM_EMAC0_IHCE            (SAM_EMAC0_BASE+SAM_EMAC_IHCE_OFFSET)
#define SAM_EMAC0_TCE             (SAM_EMAC0_BASE+SAM_EMAC_TCE_OFFSET)
#define SAM_EMAC0_UCE             (SAM_EMAC0_BASE+SAM_EMAC_UCE_OFFSET)

/* More PTP/1588 Timer Registers */

#define SAM_EMAC0_TISUBN          (SAM_EMAC0_BASE+SAM_EMAC_TISUBN_OFFSET)
#define SAM_EMAC0_TSH             (SAM_EMAC0_BASE+SAM_EMAC_TSH_OFFSET)
#define SAM_EMAC0_TSL             (SAM_EMAC0_BASE+SAM_EMAC_TSL_OFFSET)
#define SAM_EMAC0_TN              (SAM_EMAC0_BASE+SAM_EMAC_TN_OFFSET)
#define SAM_EMAC0_TA              (SAM_EMAC0_BASE+SAM_EMAC_TA_OFFSET)
#define SAM_EMAC0_TI              (SAM_EMAC0_BASE+SAM_EMAC_TI_OFFSET)
#define SAM_EMAC0_EFTSL           (SAM_EMAC0_BASE+SAM_EMAC_EFTSL_OFFSET)
#define SAM_EMAC0_EFTN            (SAM_EMAC0_BASE+SAM_EMAC_EFTN_OFFSET)
#define SAM_EMAC0_EFRSL           (SAM_EMAC0_BASE+SAM_EMAC_EFRS_OFFSET)
#define SAM_EMAC0_EFRN            (SAM_EMAC0_BASE+SAM_EMAC_EFRN_OFFSET)
#define SAM_EMAC0_PEFTSL          (SAM_EMAC0_BASE+SAM_EMAC_PEFTSL_OFFSET)
#define SAM_EMAC0_PEFTN           (SAM_EMAC0_BASE+SAM_EMAC_PEFTN_OFFSET)
#define SAM_EMAC0_PEFRSL          (SAM_EMAC0_BASE+SAM_EMAC_PEFRSL_OFFSET)
#define SAM_EMAC0_PEFRN           (SAM_EMAC0_BASE+SAM_EMAC_PEFRN_OFFSET)

/* Priority Queue */

#define SAM_EMAC0_ISRPQ_ISRPQ(n)   (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_ISRPQ_OFFSET(n))
#define SAM_EMAC0_ISRPQ_TBQBAPQ(n) (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_TBQBAPQ_OFFSET(n))
#define SAM_EMAC0_ISRPQ_RBQBAPQ(n) (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_RBQBAPQ_OFFSET(n))
#define SAM_EMAC0_ISRPQ_RBSRPQ(n)  (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_RBSRPQ_OFFSET(n))
#define SAM_EMAC0_ISRPQ_CBSCR      (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_CBSCR_OFFSET)
#define SAM_EMAC0_ISRPQ_CBSISQA    (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_CBSISQA_OFFSET)
#define SAM_EMAC0_ISRPQ_CBSISQB    (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_CBSISQB_OFFSET)
#define SAM_EMAC0_ISRPQ_ST1RPQ(n)  (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_ST1RPQ_OFFSET(n))
#define SAM_EMAC0_ISRPQ_ST2RPQ(n)  (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_ST2RPQ_OFFSET(n))
#define SAM_EMAC0_ISRPQ_IERPQ(n)   (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_IERPQ_OFFSET(n))
#define SAM_EMAC0_ISRPQ_IDRPQ(n)   (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_IDRPQ_OFFSET(n))
#define SAM_EMAC0_ISRPQ_IMRPQ(n)   (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_IMRPQ_OFFSET(n))
#define SAM_EMAC0_ISRPQ_ST2ER(n)   (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_ST2ER_OFFSET(n))
#define SAM_EMAC0_ISRPQ_ST2CW0(n)  (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_ST2CW0_OFFSET(n))
#define SAM_EMAC0_ISRPQ_ST2CW1(n)  (SAM_EMAC0_BASE+SAM_EMAC_ISRPQ_ST2CW1_OFFSET(n))

#if SAMV7_NEMAC > 1
/* EMAC1 base addresses */

#  define SAM_EMAC1_NCR           (SAM_EMAC1_BASE+SAM_EMAC_NCR_OFFSET)
#  define SAM_EMAC1_NCFGR         (SAM_EMAC1_BASE+SAM_EMAC_NCFGR_OFFSET)
#  define SAM_EMAC1_NSR           (SAM_EMAC1_BASE+SAM_EMAC_NSR_OFFSET)
#  define SAM_EMAC1_TSR           (SAM_EMAC1_BASE+SAM_EMAC_TSR_OFFSET)
#  define SAM_EMAC1_UR            (SAM_EMAC1_BASE+SAM_EMAC_UR_OFFSET)
#  define SAM_EMAC1_DCFGR         (SAM_EMAC1_BASE+SAM_EMAC_DCFGR_OFFSET)
#  define SAM_EMAC1_RBQB          (SAM_EMAC1_BASE+SAM_EMAC_RBQB_OFFSET)
#  define SAM_EMAC1_TBQB          (SAM_EMAC1_BASE+SAM_EMAC_TBQB_OFFSET)
#  define SAM_EMAC1_RSR           (SAM_EMAC1_BASE+SAM_EMAC_RSR_OFFSET)
#  define SAM_EMAC1_ISR           (SAM_EMAC1_BASE+SAM_EMAC_ISR_OFFSET)
#  define SAM_EMAC1_IER           (SAM_EMAC1_BASE+SAM_EMAC_IER_OFFSET)
#  define SAM_EMAC1_IDR           (SAM_EMAC1_BASE+SAM_EMAC_IDR_OFFSET)
#  define SAM_EMAC1_IMR           (SAM_EMAC1_BASE+SAM_EMAC_IMR_OFFSET)
#  define SAM_EMAC1_MAN           (SAM_EMAC1_BASE+SAM_EMAC_MAN_OFFSET)
#  define SAM_EMAC1_RPQ           (SAM_EMAC1_BASE+SAM_EMAC_RPQ_OFFSET)
#  define SAM_EMAC1_TPQ           (SAM_EMAC1_BASE+SAM_EMAC_TPQ_OFFSET)
#  define SAM_EMAC1_TPSF          (SAM_EMAC1_BASE+SAM_EMAC_TPSF_OFFSET)
#  define SAM_EMAC1_RPSF          (SAM_EMAC1_BASE+SAM_EMAC_RPSF_OFFSET)
#  define SAM_EMAC1_RJFML         (SAM_EMAC1_BASE+SAM_EMAC_RJFML_OFFSET)
#  define SAM_EMAC1_HRB           (SAM_EMAC1_BASE+SAM_EMAC_HRB_OFFSET)
#  define SAM_EMAC1_HRT           (SAM_EMAC1_BASE+SAM_EMAC_HRT_OFFSET)
#  define SAM_EMAC1_SAB1          (SAM_EMAC1_BASE+SAM_EMAC_SAB1_OFFSET)
#  define SAM_EMAC1_SAT1          (SAM_EMAC1_BASE+SAM_EMAC_SAT1_OFFSET)
#  define SAM_EMAC1_SAB2          (SAM_EMAC1_BASE+SAM_EMAC_SAB2_OFFSET)
#  define SAM_EMAC1_SAT2          (SAM_EMAC1_BASE+SAM_EMAC_SAT2_OFFSET)
#  define SAM_EMAC1_SAB3          (SAM_EMAC1_BASE+SAM_EMAC_SAB3_OFFSET)
#  define SAM_EMAC1_SAT3          (SAM_EMAC1_BASE+SAM_EMAC_SAT3_OFFSET)
#  define SAM_EMAC1_SAB4          (SAM_EMAC1_BASE+SAM_EMAC_SAB4_OFFSET)
#  define SAM_EMAC1_SAT4          (SAM_EMAC1_BASE+SAM_EMAC_SAT4_OFFSET)
#  define SAM_EMAC1_TIDM1         (SAM_EMAC1_BASE+SAM_EMAC_TIDM1_OFFSET)
#  define SAM_EMAC1_TIDM2         (SAM_EMAC1_BASE+SAM_EMAC_TIDM2_OFFSET)
#  define SAM_EMAC1_TIDM3         (SAM_EMAC1_BASE+SAM_EMAC_TIDM3_OFFSET)
#  define SAM_EMAC1_TIDM4         (SAM_EMAC1_BASE+SAM_EMAC_TIDM4_OFFSET)
#  define SAM_EMAC1_IPGS          (SAM_EMAC1_BASE+SAM_EMAC_IPGS_OFFSET)
#  define SAM_EMAC1_SVLAN         (SAM_EMAC1_BASE+SAM_EMAC_SVLAN_OFFSET)
#  define SAM_EMAC1_TPFCP         (SAM_EMAC1_BASE+SAM_EMAC_TPFCP_OFFSET)
#  define SAM_EMAC1_SAMB1         (SAM_EMAC1_BASE+SAM_EMAC_SAMB1_OFFSET)
#  define SAM_EMAC1_SAMT1         (SAM_EMAC1_BASE+SAM_EMAC_SAMT1_OFFSET)

/* PTP/1588 Timer Registers */

#  define SAM_EMAC1_NSC           (SAM_EMAC1_BASE+SAM_EMAC_NSC_OFFSET)
#  define SAM_EMAC1_SCL           (SAM_EMAC1_BASE+SAM_EMAC_SCL_OFFSET)
#  define SAM_EMAC1_SCH           (SAM_EMAC1_BASE+SAM_EMAC_SCH_OFFSET)
#  define SAM_EMAC1_EFTSH         (SAM_EMAC1_BASE+SAM_EMAC_EFTSH_OFFSET)
#  define SAM_EMAC1_EFRSH         (SAM_EMAC1_BASE+SAM_EMAC_EFRSH_OFFSET)
#  define SAM_EMAC1_PEFTSH        (SAM_EMAC1_BASE+SAM_EMAC_PEFTSH_OFFSET)
#  define SAM_EMAC1_PEFRSH        (SAM_EMAC1_BASE+SAM_EMAC_PEFRSH_OFFSET)

/* Statistics registers */

#  define SAM_EMAC1_OTLO          (SAM_EMAC1_BASE+SAM_EMAC_OTLO_OFFSET)
#  define SAM_EMAC1_OTHI          (SAM_EMAC1_BASE+SAM_EMAC_OTHI_OFFSET)
#  define SAM_EMAC1_FT            (SAM_EMAC1_BASE+SAM_EMAC_FT_OFFSET)
#  define SAM_EMAC1_BCFT          (SAM_EMAC1_BASE+SAM_EMAC_BCFT_OFFSET)
#  define SAM_EMAC1_MFT           (SAM_EMAC1_BASE+SAM_EMAC_MFT_OFFSET)
#  define SAM_EMAC1_PFT           (SAM_EMAC1_BASE+SAM_EMAC_PFT_OFFSET)
#  define SAM_EMAC1_BFT64         (SAM_EMAC1_BASE+SAM_EMAC_BFT64_OFFSET)
#  define SAM_EMAC1_TBFT127       (SAM_EMAC1_BASE+SAM_EMAC_TBFT127_OFFSET)
#  define SAM_EMAC1_TBFT255       (SAM_EMAC1_BASE+SAM_EMAC_TBFT255_OFFSET)
#  define SAM_EMAC1_TBFT511       (SAM_EMAC1_BASE+SAM_EMAC_TBFT511_OFFSET)
#  define SAM_EMAC1_TBFT1023      (SAM_EMAC1_BASE+SAM_EMAC_TBFT1023_OFFSET)
#  define SAM_EMAC1_TBFT1518      (SAM_EMAC1_BASE+SAM_EMAC_TBFT1518_OFFSET)
#  define SAM_EMAC1_GTBFT1518     (SAM_EMAC1_BASE+SAM_EMAC_GTBFT1518_OFFSET)
#  define SAM_EMAC1_TUR           (SAM_EMAC1_BASE+SAM_EMAC_TUR_OFFSET)
#  define SAM_EMAC1_SCF           (SAM_EMAC1_BASE+SAM_EMAC_SCF_OFFSET)
#  define SAM_EMAC1_MCF           (SAM_EMAC1_BASE+SAM_EMAC_MCF_OFFSET)
#  define SAM_EMAC1_EC            (SAM_EMAC1_BASE+SAM_EMAC_EC_OFFSET)
#  define SAM_EMAC1_LC            (SAM_EMAC1_BASE+SAM_EMAC_LC_OFFSET)
#  define SAM_EMAC1_DTF           (SAM_EMAC1_BASE+SAM_EMAC_DTF_OFFSET)
#  define SAM_EMAC1_CSE           (SAM_EMAC1_BASE+SAM_EMAC_CSE_OFFSET)
#  define SAM_EMAC1_ORLO          (SAM_EMAC1_BASE+SAM_EMAC_ORLO_OFFSET)
#  define SAM_EMAC1_ORHI          (SAM_EMAC1_BASE+SAM_EMAC_ORHI_OFFSET)
#  define SAM_EMAC1_FR            (SAM_EMAC1_BASE+SAM_EMAC_FR_OFFSET)
#  define SAM_EMAC1_BCFR          (SAM_EMAC1_BASE+SAM_EMAC_BCFR_OFFSET)
#  define SAM_EMAC1_MFR           (SAM_EMAC1_BASE+SAM_EMAC_MFR_OFFSET)
#  define SAM_EMAC1_PFR           (SAM_EMAC1_BASE+SAM_EMAC_PFR_OFFSET)
#  define SAM_EMAC1_BFR64         (SAM_EMAC1_BASE+SAM_EMAC_BFR64_OFFSET)
#  define SAM_EMAC1_TBFR127       (SAM_EMAC1_BASE+SAM_EMAC_TBFR127_OFFSET)
#  define SAM_EMAC1_TBFR255       (SAM_EMAC1_BASE+SAM_EMAC_TBFR255_OFFSET)
#  define SAM_EMAC1_TBFR511       (SAM_EMAC1_BASE+SAM_EMAC_TBFR511_OFFSET)
#  define SAM_EMAC1_TBFR1023      (SAM_EMAC1_BASE+SAM_EMAC_TBFR1023_OFFSET)
#  define SAM_EMAC1_TBFR1518      (SAM_EMAC1_BASE+SAM_EMAC_TBFR1518_OFFSET)
#  define SAM_EMAC1_TMXBFR        (SAM_EMAC1_BASE+SAM_EMAC_TMXBFR_OFFSET)
#  define SAM_EMAC1_UFR           (SAM_EMAC1_BASE+SAM_EMAC_UFR_OFFSET)
#  define SAM_EMAC1_OFR           (SAM_EMAC1_BASE+SAM_EMAC_OFR_OFFSET)
#  define SAM_EMAC1_JR            (SAM_EMAC1_BASE+SAM_EMAC_JR_OFFSET)
#  define SAM_EMAC1_FCSE          (SAM_EMAC1_BASE+SAM_EMAC_FCSE_OFFSET)
#  define SAM_EMAC1_LFFE          (SAM_EMAC1_BASE+SAM_EMAC_LFFE_OFFSET)
#  define SAM_EMAC1_RSE           (SAM_EMAC1_BASE+SAM_EMAC_RSE_OFFSET)
#  define SAM_EMAC1_AE            (SAM_EMAC1_BASE+SAM_EMAC_AE_OFFSET)
#  define SAM_EMAC1_RRE           (SAM_EMAC1_BASE+SAM_EMAC_RRE_OFFSET)
#  define SAM_EMAC1_ROE           (SAM_EMAC1_BASE+SAM_EMAC_ROE_OFFSET)
#  define SAM_EMAC1_IHCE          (SAM_EMAC1_BASE+SAM_EMAC_IHCE_OFFSET)
#  define SAM_EMAC1_TCE           (SAM_EMAC1_BASE+SAM_EMAC_TCE_OFFSET)
#  define SAM_EMAC1_UCE           (SAM_EMAC1_BASE+SAM_EMAC_UCE_OFFSET)

/* More PTP/1588 Timer Registers */

#  define SAM_EMAC1_TISUBN        (SAM_EMAC1_BASE+SAM_EMAC_TISUBN_OFFSET)
#  define SAM_EMAC1_TSH           (SAM_EMAC1_BASE+SAM_EMAC_TSH_OFFSET)
#  define SAM_EMAC1_TSL           (SAM_EMAC1_BASE+SAM_EMAC_TSL_OFFSET)
#  define SAM_EMAC1_TN            (SAM_EMAC1_BASE+SAM_EMAC_TN_OFFSET)
#  define SAM_EMAC1_TA            (SAM_EMAC1_BASE+SAM_EMAC_TA_OFFSET)
#  define SAM_EMAC1_TI            (SAM_EMAC1_BASE+SAM_EMAC_TI_OFFSET)
#  define SAM_EMAC1_EFTSL         (SAM_EMAC1_BASE+SAM_EMAC_EFTSL_OFFSET)
#  define SAM_EMAC1_EFTN          (SAM_EMAC1_BASE+SAM_EMAC_EFTN_OFFSET)
#  define SAM_EMAC1_EFRSL         (SAM_EMAC1_BASE+SAM_EMAC_EFRSL_OFFSET)
#  define SAM_EMAC1_EFRN          (SAM_EMAC1_BASE+SAM_EMAC_EFRN_OFFSET)
#  define SAM_EMAC1_PEFTSL        (SAM_EMAC1_BASE+SAM_EMAC_PEFTSL_OFFSET)
#  define SAM_EMAC1_PEFTN         (SAM_EMAC1_BASE+SAM_EMAC_PEFTN_OFFSET)
#  define SAM_EMAC1_PEFRSL        (SAM_EMAC1_BASE+SAM_EMAC_PEFRSL_OFFSET)
#  define SAM_EMAC1_PEFRN         (SAM_EMAC1_BASE+SAM_EMAC_PEFRN_OFFSET)

/* Priority Queue */

#  define SAM_EMAC1_ISRPQ_ISRPQ(n)   (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_ISRPQ_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_TBQBAPQ(n) (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_TBQBAPQ_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_RBQBAPQ(n) (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_RBQBAPQ_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_RBSRPQ(n)  (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_RBSRPQ_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_CBSCR      (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_CBSCR_OFFSET)
#  define SAM_EMAC1_ISRPQ_CBSISQA    (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_CBSISQA_OFFSET)
#  define SAM_EMAC1_ISRPQ_CBSISQB    (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_CBSISQB_OFFSET)
#  define SAM_EMAC1_ISRPQ_ST1RPQ(n)  (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_ST1RPQ_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_ST2RPQ(n)  (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_ST2RPQ_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_IERPQ(n)   (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_IERPQ_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_IDRPQ(n)   (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_IDRPQ_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_IMRPQ(n)   (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_IMRPQ_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_ST2ER(n)   (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_ST2ER_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_ST2CW0(n)  (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_ST2CW0_OFFSET(n))
#  define SAM_EMAC1_ISRPQ_ST2CW1(n)  (SAM_EMAC1_BASE+SAM_EMAC_ISRPQ_ST2CW1_OFFSET(n))
#endif

/* EMAC Register Bit Definitions ********************************************/

/* Network Control Register */

#define EMAC_NCR_LBL              (1 << 1)  /* Bit 1:  Loopback local */
#define EMAC_NCR_RXEN             (1 << 2)  /* Bit 2:  Receive enable */
#define EMAC_NCR_TXEN             (1 << 3)  /* Bit 3:  Transmit enable */
#define EMAC_NCR_MPE              (1 << 4)  /* Bit 4:  Management port enable */
#define EMAC_NCR_CLRSTAT          (1 << 5)  /* Bit 5:  Clear statistics registers */
#define EMAC_NCR_INCSTAT          (1 << 6)  /* Bit 6:  Increment statistics registers */
#define EMAC_NCR_WESTAT           (1 << 7)  /* Bit 7:  Write enable for statistics registers */
#define EMAC_NCR_BP               (1 << 8)  /* Bit 8:  Back pressure */
#define EMAC_NCR_TSTART           (1 << 9)  /* Bit 9:  Start transmission */
#define EMAC_NCR_THALT            (1 << 10) /* Bit 10: Transmit halt */
#define EMAC_TXPF                 (1 << 11) /* Bit 11: Transmit Pause Frame */
#define EMAC_TXZQPF               (1 << 12) /* Bit 12: Transmit Zero Quantum Pause Frame */
#define EMAC_SRTSM                (1 << 15) /* Bit 15: Store Receive Time Stamp to Memory */
#define EMAC_ENPBPR               (1 << 16) /* Bit 16: Enable PFC Priority-based Pause Reception */
#define EMAC_TXPBPF               (1 << 17) /* Bit 17: Transmit PFC Priority-based Pause Frame */
#define EMAC_FNP                  (1 << 18) /* Bit 18: Flush Next Packet */

/* Network Configuration Register */

#define EMAC_NCFGR_SPD            (1 << 0)  /* Bit 0:  Speed */
#define EMAC_NCFGR_FD             (1 << 1)  /* Bit 1:  Full Duplex */
#define EMAC_NCFGR_DNVLAN         (1 << 2)  /* Bit 2:  Discard Non-VLAN FRAMES */
#define EMAC_NCFGR_JFRAME         (1 << 3)  /* Bit 3:  Jumbo Frames */
#define EMAC_NCFGR_CAF            (1 << 4)  /* Bit 4:  Copy All Frames */
#define EMAC_NCFGR_NBC            (1 << 5)  /* Bit 5:  No Broadcast */
#define EMAC_NCFGR_MTIHEN         (1 << 6)  /* Bit 6:  Multicast Hash Enable */
#define EMAC_NCFGR_UNIHEN         (1 << 7)  /* Bit 7:  Unicast Hash Enable */
#define EMAC_NCFGR_MAXFS          (1 << 8)  /* Bit 8:  1536 Maximum Frame Size */
#define EMAC_NCFGR_RTY            (1 << 12) /* Bit 12: Retry test */
#define EMAC_NCFGR_PEN            (1 << 13) /* Bit 13: Pause Enable */
#define EMAC_NCFGR_RXBUFO_SHIFT   (14)      /* Bits 14-15: Receive Buffer Offset */
#define EMAC_NCFGR_RXBUFO_MASK    (3 << EMAC_NCFGR_RXBUFO_SHIFT)
#  define EMAC_NCFGR_RXBUFO(n)    ((uint32_t)(n) << EMAC_NCFGR_RXBUFO_SHIFT)
#  define EMAC_NCFGR_RXBUFO_NONE  (0 << EMAC_NCFGR_RXBUFO_SHIFT) /* No offset from RX buffer start */
#  define EMAC_NCFGR_RXBUFO_1     (1 << EMAC_NCFGR_RXBUFO_SHIFT) /* One-byte offset from RX buffer start */
#  define EMAC_NCFGR_RXBUFO_2     (2 << EMAC_NCFGR_RXBUFO_SHIFT) /* Two-byte offset from RX buffer start */
#  define EMAC_NCFGR_RXBUFO_3     (3 << EMAC_NCFGR_RXBUFO_SHIFT) /* Three-byte offset fromRX buffer start */

#define EMAC_NCFGR_LFERD          (1 << 16) /* Bit 16: Length Field Error Frame Discard */
#define EMAC_NCFGR_RFCS           (1 << 17) /* Bit 17: Remove FCS */
#define EMAC_NCFGR_CLK_SHIFT      (18)      /* Bits 18-20: MDC clock divider */
#define EMAC_NCFGR_CLK_MASK       (7 << EMAC_NCFGR_CLK_SHIFT)
#  define EMAC_NCFGR_CLK_DIV8     (0 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 8 (MCK up to 20 MHz) */
#  define EMAC_NCFGR_CLK_DIV16    (1 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 16 (MCK up to 40 MHz) */
#  define EMAC_NCFGR_CLK_DIV32    (2 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 32 (MCK up to 80 MHz) */
#  define EMAC_NCFGR_CLK_DIV48    (3 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 48 (MCK up to 120 MHz) */
#  define EMAC_NCFGR_CLK_DIV64    (4 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 64 (MCK up to 160 MHz) */
#  define EMAC_NCFGR_CLK_DIV96    (5 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 96 (MCK up to 240 MHz) */

#define EMAC_NCFGR_DBW_SHIFT      (21)      /* Bit 21-22: Data Bus Width */
#define EMAC_NCFGR_DBW_MASK       (3 << EMAC_NCFGR_DBW_SHIFT)
#  define EMAC_NCFGR_DBW_ZERO     (0 << EMAC_NCFGR_DBW_SHIFT) /* Must be zero */

#define EMAC_NCFGR_DCPF           (1 << 23) /* Bit 23: Disable Copy of Pause Frames */
#define EMAC_NCFGR_RXCOEN         (1 << 24) /* Bit 24: Receive Checksum Offload Enable */
#define EMAC_NCFGR_EFRHD          (1 << 25) /* Bit 25: Enable Frames Received in Half Duplex */
#define EMAC_NCFGR_IRXFCS         (1 << 26) /* Bit 26: Ignore RX FCS */
#define EMAC_NCFGR_IPGSEN         (1 << 28) /* Bit 28: IP Stretch Enable */
#define EMAC_NCFGR_RXBP           (1 << 29) /* Bit 29: Receive Bad Preamble */
#define EMAC_NCFGR_IRXER          (1 << 30) /* Bit 30: Ignore IPG GRXER */

/* Network Status Register */

#define EMAC_NSR_MDIO             (1 << 1)  /* Bit 1:  Status of the MDIO input pin */
#define EMAC_NSR_IDLE             (1 << 2)  /* Bit 2:  PHY management logic is idle */

/* User Register */

#define EMAC_UR_RMII              (1 << 0)  /* Bit 0:  Reduced MII Mode */

/* DMA Configuration Register */

#define EMAC_DCFGR_FBLDO_SHIFT    (0)     /* Bits 0-4: Fixed Burst Length for DMA Data Operations */
#define EMAC_DCFGR_FBLDO_MASK     (31 << EMAC_DCFGR_FBLDO_SHIFT)
#  define EMAC_DCFGR_FBLDO_SINGLE (1 << EMAC_DCFGR_FBLDO_SHIFT)  /* Always use SINGLE AHB bursts */
#  define EMAC_DCFGR_FBLDO_INCR4  (4 << EMAC_DCFGR_FBLDO_SHIFT)  /* Attempt to use INCR4 AHB bursts */
#  define EMAC_DCFGR_FBLDO_INCR8  (8 << EMAC_DCFGR_FBLDO_SHIFT)  /* Attempt to use INCR8 AHB bursts */
#  define EMAC_DCFGR_FBLDO_INCR16 (16 << EMAC_DCFGR_FBLDO_SHIFT) /* Attempt to use INCR16 AHB bursts */

#define EMAC_DCFGR_ESMA           (1 << 6) /* Bit 6:  Endian Swap Mode Enable for Management Descriptor Accesses */
#define EMAC_DCFGR_ESPA           (1 << 7) /* Bit 7:  Endian Swap Mode Enable for Packet Data Accesses */
#define EMAC_DCFGR_RXBMS_SHIFT    (8)      /* Bits 8-9:  Receiver Packet Buffer Memory Size Select */
#define EMAC_DCFGR_RXBMS_MASK     (3 << EMAC_DCFGR_RXBMS_SHIFT)
#  define EMAC_DCFGR_RXBMS_8TH    (0 << EMAC_DCFGR_RXBMS_SHIFT) /* 4/8 Kbyte Memory Size */
#  define EMAC_DCFGR_RXBMS_4TH    (1 << EMAC_DCFGR_RXBMS_SHIFT) /* 4/4 Kbytes Memory Size */
#  define EMAC_DCFGR_RXBMS_HALF   (2 << EMAC_DCFGR_RXBMS_SHIFT) /* 4/2 Kbytes Memory Size */
#  define EMAC_DCFGR_RXBMS_FULL   (3 << EMAC_DCFGR_RXBMS_SHIFT) /* 4 Kbytes Memory Size */

#define EMAC_DCFGR_TXPBMS         (1 << 10) /* Bit 10: Transmitter Packet Buffer Memory Size Select */
#define EMAC_DCFGR_TXCOEN         (1 << 11) /* Bit 11: Transmitter Checksum Generation Offload Enable */
#define EMAC_DCFGR_DRBS_SHIFT     (16)      /* Bits 16-23: DMA Receive Buffer Size */
#define EMAC_DCFGR_DRBS_MASK      (0xff << EMAC_DCFGR_DRBS_SHIFT)
#  define EMAC_DCFGR_DRBS(n)      ((uint32_t)(n) << EMAC_DCFGR_DRBS_SHIFT)
#define EMAC_DCFGR_DDRP           (1 << 24) /* Bit 24: DMA Discard Receive Packets */

/* Transmit Status Register */

#define EMAC_TSR_UBR              (1 << 0)  /* Bit 0:  Used Bit Read */
#define EMAC_TSR_COL              (1 << 1)  /* Bit 1:  Collision Occurred */
#define EMAC_TSR_RLE              (1 << 2)  /* Bit 2:  Retry Limit exceeded */
#define EMAC_TSR_TXGO             (1 << 3)  /* Bit 3:  Transmit Go */
#define EMAC_TSR_TFC              (1 << 4)  /* Bit 4:  Transmit Frame Corruption due to AHB error */
#define EMAC_TSR_TXCOMP           (1 << 5)  /* Bit 5:  Transmit Complete */
#define EMAC_TSR_HRESP            (1 << 8)  /* Bit 8:  HRESP Not OK */

/* Receive Buffer Queue Pointer Register */

#define EMAC_RBQB_MASK            (0xfffffffc)  /* Bits 2-31: Receive buffer queue pointer address */

/* Transmit Buffer Queue Pointer Register */

#define EMAC_TBQB_MASK            (0xfffffffc)  /* Bits 2-31: Transmit buffer queue pointer address */

/* Receive Status Register */

#define EMAC_RSR_BNA              (1 << 0)  /* Bit 0:  Buffer Not Available */
#define EMAC_RSR_REC              (1 << 1)  /* Bit 1:  Frame Received */
#define EMAC_RSR_RXOVR            (1 << 2)  /* Bit 2:  Receive Overrun */
#define EMAC_RSR_HNO              (1 << 3)  /* Bit 3:  HRESP Not OK */

/* Interrupt Status Register (ISR), Interrupt Enable Register (IER),
 * Interrupt Disable Register (IDR) and Interrupt Mask Register (IMR)
 */

#define EMAC_INT_MFS              (1 << 0)  /* Bit 0:  Management Frame Sent */
#define EMAC_INT_RCOMP            (1 << 1)  /* Bit 1:  Receive Complete */
#define EMAC_INT_RXUBR            (1 << 2)  /* Bit 2:  Receive Used Bit Read */
#define EMAC_INT_TXUBR            (1 << 3)  /* Bit 3:  Transmit Used Bit Read */
#define EMAC_INT_TUR              (1 << 4)  /* Bit 4:  Ethernet Transmit Buffer Underrun */
#define EMAC_INT_RLEX             (1 << 5)  /* Bit 5:  Retry Limit Exceeded */
#define EMAC_INT_TFC              (1 << 6)  /* Bit 6:  Transmit Frame Corruption due to AHB error */
#define EMAC_INT_TCOMP            (1 << 7)  /* Bit 7:  Transmit Complete */
#define EMAC_INT_ROVR             (1 << 10) /* Bit 10: Receive Overrun */
#define EMAC_INT_HRESP            (1 << 11) /* Bit 11: Hresp not OK */
#define EMAC_INT_PFNZ             (1 << 12) /* Bit 12: Pause Frame with Non-zero Pause Quantum Received */
#define EMAC_INT_PTZ              (1 << 13) /* Bit 13: Pause Time Zero */
#define EMAC_INT_PTFR             (1 << 14) /* Bit 14: Pause Frame Transmitted */
#define EMAC_INT_EXINT            (1 << 15) /* Bit 15: External Interrupt (not SR) */
#define EMAC_INT_DRQFR            (1 << 18) /* Bit 18: PTP Delay Request Frame Received */
#define EMAC_INT_SFR              (1 << 19) /* Bit 19: PTP Sync Frame Received */
#define EMAC_INT_DRQFT            (1 << 20) /* Bit 20: PTP Delay Request Frame Transmitted */
#define EMAC_INT_SFT              (1 << 21) /* Bit 21: PTP Sync Frame Transmitted */
#define EMAC_INT_PDRQFR           (1 << 22) /* Bit 22: PDelay Request Frame Received */
#define EMAC_INT_PDRSFR           (1 << 23) /* Bit 23: PDelay Response Frame Received */
#define EMAC_INT_PDRQFT           (1 << 24) /* Bit 24: PDelay Request Frame Transmitted */
#define EMAC_INT_PDRSFT           (1 << 25) /* Bit 25: PDelay Response Frame Transmitted */
#define EMAC_INT_SRI              (1 << 26) /* Bit 26: TSU Seconds Register Increment (not IMR) */
#define EMAC_INT_WOL              (1 << 28) /* Bit 28: Wake On LAN (not IMR) */

#define EMAC_INT_ALL              (0x17fcfcff)
#define EMAC_INT_UNUSED           (0xe8030300)

/* PHY Maintenance Register */

#define EMAC_MAN_DATA_SHIFT       (0)       /* Bits 0-15: Read/write data */
#define EMAC_MAN_DATA_MASK        (0x0000ffff << EMAC_MAN_DATA_SHIFT)
#  define EMAC_MAN_DATA(n)        ((uint32_t)(n) << EMAC_MAN_DATA_SHIFT)
#define EMAC_MAN_WTN_SHIFT        (16)      /* Bits 16-17:  Must be written to b10 */
#define EMAC_MAN_WTN_MASK         (3 << EMAC_MAN_WTN_SHIFT)
#  define EMAC_MAN_WTN            (2 << EMAC_MAN_WTN_SHIFT)
#define EMAC_MAN_REGA_SHIFT       (18)      /* Bits 18-22: Register Address */
#define EMAC_MAN_REGA_MASK        (31 << EMAC_MAN_REGA_SHIFT)
#  define EMAC_MAN_REGA(n)        ((uint32_t)(n) << EMAC_MAN_REGA_SHIFT)
#define EMAC_MAN_PHYA_SHIFT       (23)      /* Bits 23-27: PHY Address */
#define EMAC_MAN_PHYA_MASK        (31 << EMAC_MAN_PHYA_SHIFT)
#  define EMAC_MAN_PHYA(n)        ((uint32_t)(n) << EMAC_MAN_PHYA_SHIFT)
#define EMAC_MAN_OP_SHIFT         (28)      /* Bits 28-29: Operation */
#define EMAC_MAN_OP_MASK          (3 << EMAC_MAN_OP_SHIFT)
#  define EMAC_MAN_READ           (2 << EMAC_MAN_OP_SHIFT)
#  define EMAC_MAN_WRITE          (1 << EMAC_MAN_OP_SHIFT)
#define EMAC_MAN_CLTTO            (1 << 30) /* Bit 30: Clause 22 Operation */
#define EMAC_MAN_WZO              (0)       /* Bit 31: Write ZERO */

/* Pause Time Register */

#define EMAC_RPQ_MASK             (0x0000ffff) /* Bits 0-15: Received Pause Quantum */

/* Pause Frames Received Register */

#define EMAC_TPQ_MASK             (0x0000ffff) /* Bits 0-15: Transmit Pause Quantum */

/* TX Partial Store and Forward Register */

#define EMAC_TPSF_TPB1ADR_SHIFT   (0)       /* Bits 0-11: Transmit Partial Store and Forward Address */
#define EMAC_TPSF_TPB1ADR_MASK    (0xfff << EMAC_TPSF_TPB1ADR_SHIFT)
#  define EMAC_TPSF_TPB1ADR(n)    ((uint32_t)(n) << EMAC_TPSF_TPB1ADR_SHIFT)
#define EMAC_TPSF_ENTXP           (1 << 31) /* Bit 31: Enable TX Partial Store and Forward Operation */

/* RX Partial Store and Forward Register */

#define EMAC_RPSF_RPB1ADR_SHIFT   (0)       /* Bits 0-11: Receive Partial Store and Forward Address */
#define EMAC_RPSF_RPB1ADR_MASK    (0xfff << EMAC_RPSF_RPB1ADR_SHIFT)
#  define EMAC_RPSF_RPB1ADR(n)    ((uint32_t)(n) << EMAC_RPSF_RPB1ADR_SHIFT)
#define EMAC_RPSF_ENRXP           (1 << 31) /* Bit 31: Enable RX Partial Store and Forward Operation */

/* RX Jumbo Frame Max Length Register */

#define EMAC_RJFML_MASK           0x00003fff /* Bits 0-13: Frame Max Length */

/* Hash Register Bottom [31:0] Register (LS 32-bit hash address) */

/* Hash Register Top [63:32] Register (MS 32-bit hash address) */

/* Specific Address 1 Bottom [31:0] Register (LS 32-bit address) */

/* Specific Address 1 Top [47:32] Register */

#define EMAC_SAT1_MASK            (0x0000ffff) /* Bits 0-15: Bits 32-47 of the destination address */

/* Specific Address 2 Bottom [31:0] Register (LS 32-bit address) */

/* Specific Address 2 Top [47:32] Register */

#define EMAC_SAT2_MASK            (0x0000ffff) /* Bits 0-15: Bits 32-47 of the destination address */

/* Specific Address 3 Bottom [31:0] Register (LS 32-bit address) */

/* Specific Address 3 Top [47:32] Register */

#define EMAC_SAT3_MASK            (0x0000ffff) /* Bits 0-15: Bits 32-47 of the destination address */

/* Specific Address 4 Bottom [31:0] Register (LS 32-bit address) */

/* Specific Address 4 Top [47:32] Register */

#define EMAC_SAT4_MASK            (0x0000ffff) /* Bits 0-15: Bits 32-47 of the destination address */

/* Type ID Match Registers */

#define EMAC_TIDM_MASK            (0x0000ffff) /* Bits 0-15: Type ID Match n */

/* Type ID Checking Register */

#define EMAC_TID_SHIFT            (0)       /* Bits 0-15: Type ID Match */
#define EMAC_TID_MASK             (0xffff << EMAC_TID_SHIFT)
#  define EMAC_TID(n)             ((uint32_t)(n) << EMAC_TID_SHIFT)
#define EMAC_TID_ENID             (1 << 31) /* Bit 31: Enable Copying of TID Matched Frames */

/* Wake-up on LAN Register */

#define EMAC_WOL_IP_SHIFT         (0)       /* Bits 0-15: ARP Request IP Address */
#define EMAC_WOL_IP_MASK          (0x0000ffff << EMAC_WOL_IP_SHIFT)
#  define EMAC_WOL_IP(n)          ((uin32_t)(n) << EMAC_WOL_IP_SHIFT)
#define EMAC_WOL_MAG              (1 << 16) /* Bit 16:  Magic Packet Event Enable */
#define EMAC_WOL_ARP              (1 << 17) /* Bit 17: ARP Request Event Enable */
#define EMAC_WOL_SA1              (1 << 18) /* Bit 18: Specific Address Register 1 Event Enable */
#define EMAC_WOL_MTI              (1 << 19) /* Bit 19: Multicast Hash Event Enable */

/* IPG Stretch Register */

#define EMAC_IPGS_FL_MASK         (0x0000ffff) /* Bit 0-15: Frame Length */

/* Stacked VLAN Register */

#define EMAC_SVLAN_VLANTYPE_SHIFT (0)       /* Bits 0-15: User Defined VLAN_TYPE Field */
#define EMAC_SVLAN_VLANTYPE_MASK  (0x0000ffff << EMAC_SVLAN_VLANTYPE_SHIFT)
#  define EMAC_SVLAN_VLANTYPE(n)  ((uint32_t)(n) << EMAC_SVLAN_VLANTYPE_SHIFT)
#define EMAC_SVLAN_ESVLAN         (1 << 31) /* Bit 31: Enable Stacked VLAN Processing Mode */

/* Transmit PFC Pause Register */

#define EMAC_TPFCP_PEV_SHIFT      (0)       /* Bits 0-7: Priority Enable Vector */
#define EMAC_TPFCP_PEV_MASK       (0xff << EMAC_TPFCP_PEV_SHIFT)
#  define EMAC_TPFCP_PEV(n)       ((uint32_t)(n) << EMAC_TPFCP_PEV_SHIFT)
#define EMAC_TPFCP_PQ_SHIFT       (8)       /* Bits 8-15: Pause Quantum */
#define EMAC_TPFCP_PQ_MASK        (0xff << EMAC_TPFCP_PQ_SHIFT)
#  define EMAC_TPFCP_PQ(n)        ((uint32_t)(n) << EMAC_TPFCP_PQ_SHIFT)

/* Specific Address 1 Mask Bottom [31:0] Register (LS 32-bit address) */

/* Specific Address 1 Mask Top [47:32] Register (MS 16-bit address) */

#define EMAC_SAMT1_MASK           (0x0000ffff) /* Bits 0-15: Bits 32-47 of Specific Address 1 Mask */

/* 1588 Timer Nanosecond Comparison Register */

#define EMAC_NSC_MASK             (0x00cfffff) /* Bits 0-21: 1588 Timer Nanosecond Comparison Value */

/* 1588 Timer Second Comparison Low Register (32-bit value) */

/* 1588 Timer Second Comparison High Register */

#define EMAC_SCH_MASK             (0x0000ffff) /* Bits 0-15: 1588 Timer Second Comparison Value */

/* PTP Event Frame Transmitted Seconds High Register */

#define EMAC_EFTSH_MASK           (0x0000ffff) /* Bits 0-15: Register Update */

/* PTP Event Frame Received Seconds High Register */

#define EMAC_EFRSH_MASK           (0x0000ffff) /* Bits 0-15: Register Update */

/* PTP Peer Event Frame Transmitted Seconds High Register */

#define EMAC_PEFTSH_MASK          (0x0000ffff) /* Bits 0-15: Register Update */

/* PTP Peer Event Frame Received Seconds High Register */

#define EMAC_PEFRSH_MASK          (0x0000ffff) /* Bits 0-15: Register Update */

/* Statistics registers.  Only masking is needed.
 *
 *   Octets Transmitted [31:0] Register (OTLO)                      32-bit
 *   Octets Transmitted [47:32] Register (OTHI)                     16-bit
 *   Frames Transmitted Register (FT)                               32-bit
 *   Broadcast Frames Transmitted Register (BCFT)                   32-bit
 *   Multicast Frames Transmitted Register (MFT)                    32-bit
 *   Pause Frames Transmitted Register (PFT)                        16-bit
 *   64 Byte Frames Transmitted Register (BFT64)                    32-bit
 *   65 to 127 Byte Frames Transmitted Register (TBFT127)           32-bit
 *   128 to 255 Byte Frames Transmitted Register (TBFT255)          32-bit
 *   256 to 511 Byte Frames Transmitted Register (TBFT511)          32-bit
 *   512 to 1023 Byte Frames Transmitted Register (TBFT1023)        32-bit
 *   1024 to 1518 Byte Frames Transmitted Register (TBFT1518)       32-bit
 *   Greater Than 1518 Byte Frames Transmitted Register (GTBFT1518) 32-bit
 *   Transmit Under Runs Register (TUR)                             10-bit
 *   Single Collision Frames Register (SCF)                         18-bit
 *   Multiple Collision Frames Register (MCF)                       18-bit
 *   Excessive Collisions Register (EC)                             10-bit
 *   Late Collisions Register (LC)                                  10-bit
 *   Deferred Transmission Frames Register (DTF)                    18-bit
 *   Carrier Sense Errors Register (CSE)                            10-bit
 *   Octets Received [31:0] Received (ORLO)                         32-bit
 *   Octets Received [47:32] Received (ORHI)                        16-bit
 *   Frames Received Register (FR)                                  32-bit
 *   Broadcast Frames Received Register (BCFR)                      32-bit
 *   Multicast Frames Received Register (MFR)                       32-bit
 *   Pause Frames Received Register (PFR)                           32-bit
 *   64 Byte Frames Received Register (BFR64)                       32-bit
 *   65 to 127 Byte Frames Received Register (TBFR127)              32-bit
 *   128 to 255 Byte Frames Received Register (TBFR255)             32-bit
 *   256 to 511Byte Frames Received Register (TBFR511)              32-bit
 *   512 to 1023 Byte Frames Received Register (TBFR1023)           32-bit
 *   1024 to 1518 Byte Frames Received Register (TBFR1518)          32-bit
 *   1519 to Maximum Byte Frames Received Register (TMXBFR)         32-bit
 *   Undersize Frames Received Register (UFR)                       10-bit
 *   Oversize Frames Received Register (OFR)                        10-bit
 *   Jabbers Received Register (JR)                                 10-bit
 *   Frame Check Sequence Errors Register (FCSE)                    10-bit
 *   Length Field Frame Errors Register (LFFE)                      10-bit
 *   Receive Symbol Errors Register (RSE)                           10-bit
 *   Alignment Errors Register (AE)                                 10-bit
 *   Receive Resource Errors Register (RRE)                         18-bit
 *   Receive Overrun Register (ROE)                                 10-bit
 *   IP Header Checksum Errors Register (IHCE)                       8-bit
 *   TCP Checksum Errors Register (TCE)                              8-bit
 *   UDP Checksum Errors Register (UCE)                              8-bit
 */

/* PTP/1588 Timer Registers */

/* 1588 Timer Increment Sub-nanoseconds Register */

#define EMAC_TISUBN_MASK          (0x0000ffff) /* Bits 0-15: LS Bits of Timer Increment Register */

/* 1588 Timer Seconds High Register (32-bit timer value) */

/* 1588 Timer Seconds Low Register (32-bit timer value) */

/* 1588 Timer Nanoseconds Register (30-bit timer value) */

#define EMAC_TN_MASK              (0x3fffffff) /* Bit 0-29: Timer Count in Nanoseconds */

/* 1588 Timer Adjust Register */

#define EMAC_TA_ITDT_SHIFT        (0)       /* Bits 0-29: Increment/Decrement */
#define EMAC_TA_ITDT_MASK         (0x3fffffff << EMAC_TA_ITDT_SHIFT)
#  define EMAC_TA_ITDT(n)         ((uint32_t)(n) << EMAC_TA_ITDT_SHIFT)
#define EMAC_TA_ADJ               (1 << 31) /* Bit 31: Adjust 1588 Timer */

/* 1588 Timer Increment Register */

#define EMAC_TI_CNS_SHIFT         (0)       /* Bits 0-7: Count Nanoseconds */
#define EMAC_TI_CNS_MASK          (0xff << EMAC_TI_CNS_SHIFT)
#  define EMAC_TI_CNS(n)          ((uint32_t)(n) << EMAC_TI_CNS_SHIFT)
#define EMAC_TI_ACNS_SHIFT        (8)       /* Bits 8-15: Alternative Count Nanoseconds */
#define EMAC_TI_ACNS_MASK         (0xff << EMAC_TI_ACNS_SHIFT)
#  define EMAC_TI_ACNS(n)         ((uint32_t)(n) << EMAC_TI_ACNS_SHIFT)
#define EMAC_TI_NIT_SHIFT         (16)       /* Bits 16-23: Number of Increments */
#define EMAC_TI_NIT_MASK          (0xff << EMAC_TI_NIT_SHIFT)
#  define EMAC_TI_NIT(n)          ((uint32_t)(n) << EMAC_TI_NIT_SHIFT)

/* PTP Event Frame Transmitted Seconds Low (32-bit timer value) */

/* PTP Event Frame Transmitted Nanoseconds (30-bit timer value) */

#define EMAC_EFTN_MASK            (0x3fffffff) /* Bit 0-29: Register Update */

/* PTP Event Frame Received Seconds Low (32-bit timer value) */

/* PTP Event Frame Received Nanoseconds (30-bit timer value) */

#define EMAC_EFRN_MASK            (0x3fffffff) /* Bit 0-29: Register Update */

/* PTP Peer Event Frame Transmitted Seconds Low (32-bit timer value) */

/* PTP Peer Event Frame Transmitted Nanoseconds (30-bit timer value) */

#define EMAC_PEFTN_MASK           (0x3fffffff) /* Bit 0-29: Register Update */

/* PTP Peer Event Frame Received Seconds Low (32-bit timer value) */

/* PTP Peer Event Frame Received Nanoseconds (30-bit timer value) */

#define EMAC_PEFRN_MASK           (0x3fffffff) /* Bit 0-29: Register Update */

/* Interrupt Status Register Priority Queue,
 * Interrupt Enable Register Priority Queue,
 * Interrupt Disable Register Priority Queue,
 * and Interrupt Mask Register Priority Queue,
 * n=1-3
 *
 * This register uses a subset of the standard interrupt definitions:
 *
 *   EMAC_INT_RCOMP            Bit 1:  Receive Complete
 *   EMAC_INT_RXUBR            Bit 2:  Receive Used Bit Read
 *   EMAC_INT_RLEX             Bit 5:  Retry Limit Exceeded
 *   EMAC_INT_TFC              Bit 6:  Transmit Frame Corruption due
 *                                     to AHB error
 *   EMAC_INT_TCOMP            Bit 7:  Transmit Complete
 *   EMAC_INT_ROVR             Bit 10: Receive Overrun
 *   EMAC_INT_HRESP            Bit 11: Hresp not OK
 */

#define EMAC_INTPQ_ALL            (0x00000ce6)
#define EMAC_INTPG_UNUSED         (0xfffffe19)

/* Transmit Buffer Queue Base Address Register Priority Queue, n=1-3 */

#define EMAC_ISRPQ_TBQBAPQ_MASK   (0xfffffffc) /* Bits 2-31: Transmit Buffer Queue Base Address */

/* Receive Buffer Queue Base Address Register Priority Queue, n=1-3 */

#define EMAC_ISRPQ_RBQBAPQ_MASK   (0xfffffffc) /* Bits 2-31: Receive Buffer Queue Base Address */

/* Receive Buffer Size Register Priority Queue, n=1-3 */

#define EMAC_ISRPQ_RBSRPQ_MASK   (0x0000ffff) /* Bits 0-15: Receive Buffer Size */

/* Credit-Based Shaping Control Register */

#define EMAC_ISRPQ_CBSCR_QAE     (1 << 0)  /* Bit 0:  Queue A CBS Enable */
#define EMAC_ISRPQ_CBSCR_QBE     (1 << 1)  /* Bit 1:  Queue B CBS Enable */

/* Credit-Based Shaping IdleSlope Register for Queue A (32-bit value) */

/* Credit-Based Shaping IdleSlope Register for Queue B (32-bit value) */

/* Screening Type 1 Register Priority Queue, 0=1-3 */

#define EMAC_ISRPQ_ST1RPQ_QNB_SHIFT    (0)       /* Bits 0-2: Queue Number (0–2) */
#define EMAC_ISRPQ_ST1RPQ_QNB_MASK     (7 << EMAC_ISRPQ_ST1RPQ_QNB_SHIFT)
#  define EMAC_ISRPQ_ST1RPQ_QNB(n)     ((uint32_t)(n) << EMAC_ISRPQ_ST1RPQ_QNB_SHIFT)
#define EMAC_ISRPQ_ST1RPQ_DSTCM_SHIFT  (4)       /* Bits 4-11: Differentiated Services or Traffic Class Match */
#define EMAC_ISRPQ_ST1RPQ_DSTCM_MASK   (0xff << EMAC_ISRPQ_ST1RPQ_DSTCM_SHIFT)
#  define EMAC_ISRPQ_ST1RPQ_DSTCM(n)   ((uint32_t)(n) << EMAC_ISRPQ_ST1RPQ_DSTCM_SHIFT)
#define EMAC_ISRPQ_ST1RPQ_UDPM_SHIFT   (12)      /* Bits 12-27: UDP Port Match */
#define EMAC_ISRPQ_ST1RPQ_UDPM_MASK    (0xffff << EMAC_ISRPQ_ST1RPQ_UDPM_SHIFT)
#  define EMAC_ISRPQ_ST1RPQ_UDPM(n)    ((uint32_t)(n) << EMAC_ISRPQ_ST1RPQ_UDPM_SHIFT)
#define EMAC_ISRPQ_ST1RPQ_DSTCE        (1 << 28) /* Bit 28: Differentiated Services or Traffic Class Match Enable */
#define EMAC_ISRPQ_ST1RPQ_UDPE         (1 << 29) /* Bit 29: UDP Port Match Enable */

/* Screening Type 2 Register Priority Queue, 0=1-7 */

#define EMAC_ISRPQ_ST2RPQ_QNB_SHIFT    (0)       /* Bits 0-3: Queue Number (0–2) */
#define EMAC_ISRPQ_ST2RPQ_QNB_MASK     (7 << EMAC_ISRPQ_ST2RPQ_QNB_SHIFT)
#  define EMAC_ISRPQ_ST2RPQ_QNB(n)     ((uint32_t)(n) << EMAC_ISRPQ_ST2RPQ_QNB_SHIFT)
#define EMAC_ISRPQ_ST2RPQ_VLANP_SHIFT  (4)       /* Bits 4-6: VLAN Priority */
#define EMAC_ISRPQ_ST2RPQ_VLANP_MASK   (7 << EMAC_ISRPQ_ST2RPQ_VLANP_SHIFT)
#  define EMAC_ISRPQ_ST2RPQ_VLANP(n)   ((uint32_t)(n) << EMAC_ISRPQ_ST2RPQ_VLANP_SHIFT)
#define EMAC_ISRPQ_ST2RPQ_VLANE        (1 << 8)  /* Bit 8:  VLAN Enable */
#define EMAC_ISRPQ_ST2RPQ_I2ETH_SHIFT  (9)       /* Bits 9-11: Index of Screening Type 2 EtherType register x */
#define EMAC_ISRPQ_ST2RPQ_I2ETH_MASK   (7 << EMAC_ISRPQ_ST2RPQ_I2ETH_SHIFT)
#  define EMAC_ISRPQ_ST2RPQ_I2ETH(n)   ((uint32_t)(n) << EMAC_ISRPQ_ST2RPQ_I2ETH_SHIFT)
#define EMAC_ISRPQ_ST2RPQ_ETHE         (1 << 12) /* Bit 12: EtherType Enable */
#define EMAC_ISRPQ_ST2RPQ_COMPA_SHIFT  (13)      /* Bits 13-17: Index of Screening Type 2 Compare Word 0/Word 1 register x */
#define EMAC_ISRPQ_ST2RPQ_COMPA_MASK   (31 << EMAC_ISRPQ_ST2RPQ_COMPA_SHIFT)
#  define EMAC_ISRPQ_ST2RPQ_COMPA(n)   ((uint32_t)(n) << EMAC_ISRPQ_ST2RPQ_COMPA_SHIFT)
#define EMAC_ISRPQ_ST2RPQ_COMPAE       (1 << 18) /* Bit 18: Compare A Enable */
#define EMAC_ISRPQ_ST2RPQ_COMPB_SHIFT  (19)      /* Bits 19-23: Index of Screening Type 2 Compare Word 0/Word 1 register x */
#define EMAC_ISRPQ_ST2RPQ_COMPB_MASK   (31 << EMAC_ISRPQ_ST2RPQ_COMPB_SHIFT)
#  define EMAC_ISRPQ_ST2RPQ_COMPB(n)   ((uint32_t)(n) << EMAC_ISRPQ_ST2RPQ_COMPB_SHIFT)
#define EMAC_ISRPQ_ST2RPQ_COMPBE       (1 << 24) /* Bit 24: Compare B Enable */
#define EMAC_ISRPQ_ST2RPQ_COMPC_SHIFT  (25)      /* Bits 25-29: Index of Screening Type 2 Compare Word 0/Word 1 register x */
#define EMAC_ISRPQ_ST2RPQ_COMPC_MASK   (31 << EMAC_ISRPQ_ST2RPQ_COMPC_SHIFT)
#  define EMAC_ISRPQ_ST2RPQ_COMPC(n)   ((uint32_t)(n) << EMAC_ISRPQ_ST2RPQ_COMPC_SHIFT)
#define EMAC_ISRPQ_ST2RPQ_COMPCE       (1 << 30) /* Bit 30: Compare C Enable */

/* Screening Type 2 Ethertype Register, n=0-3 */

#define EMAC_ISRPQ_ST2ER_MASK          (0x0000ffff) /* Bits 0-15: Ethertype Compare Value */

/* Screening Type 2 Compare Word 0 Registerm, n=0-23 */

#define EMAC_ISRPQ_ST2CW0_MASK_SHIFT   (0)       /* Bits 0-15:  Mask Value */
#define EMAC_ISRPQ_ST2CW0_MASK_MASK    (0xffff << EMAC_ISRPQ_ST2CW0_MASK_SHIFT)
#  define EMAC_ISRPQ_ST2CW0_MASK(n)    ((uint32_t)(n) << EMAC_ISRPQ_ST2CW0_MASK_SHIFT)
#define EMAC_ISRPQ_ST2CW0_COMP_SHIFT   (16)      /* Bits 16-31: Compare Value */
#define EMAC_ISRPQ_ST2CW0_COMP_MASK    (0xffff << EMAC_ISRPQ_ST2CW0_COMP_SHIFT)
#  define EMAC_ISRPQ_ST2CW0_COMP(n)    ((uint32_t)(n) << EMAC_ISRPQ_ST2CW0_COMP_SHIFT)

/* Screening Type 2 Compare Word 1 Register, n=0-23 */

#define EMAC_ISRPQ_ST2CW1_OFFS_SHIFT   (0)       /* Bits 0-6: Offset Value in Bytes */
#define EMAC_ISRPQ_ST2CW1_OFFS_MASK    (0x7f << EMAC_ISRPQ_ST2CW1_OFFS_SHIFT)
#  define EMAC_ISRPQ_ST2CW1_OFFS(n)    ((uint32_t)(n) << EMAC_ISRPQ_ST2CW1_OFFS_SHIFT)
#define EMAC_ISRPQ_ST2CW1_STRT_SHIFT   (7)       /* Bits 7-8: Ethernet Frame Offset Start */
#define EMAC_ISRPQ_ST2CW1_STRT_MASK    (3 << EMAC_ISRPQ_ST2CW1_STRT_SHIFT)
#  define EMAC_ISRPQ_ST2CW1_STRT_FRMSTRT (0 << EMAC_ISRPQ_ST2CW1_STRT_SHIFT) /* Offset from the start of the frame */
#  define EMAC_ISRPQ_ST2CW1_STRT_ETHTYP  (1 << EMAC_ISRPQ_ST2CW1_STRT_SHIFT) /* Offset from the byte after the EtherType field */
#  define EMAC_ISRPQ_ST2CW1_STRT_IP      (2 << EMAC_ISRPQ_ST2CW1_STRT_SHIFT) /* Offset from the byte after the IP header field */
#  define EMAC_ISRPQ_ST2CW1_STRT_TCPUDP  (3 << EMAC_ISRPQ_ST2CW1_STRT_SHIFT) /* Offset from the byte after the TCP/UDP header field */

/* Descriptors **************************************************************/

/* Receive buffer descriptor:  Address word */

#define EMACRXD_ADDR_OWNER        (1 << 0)     /* Bit 0:  1=Software owns; 0=EMAC owns */
#define EMACRXD_ADDR_WRAP         (1 << 1)     /* Bit 1:  Last descriptor in list */
#define EMACRXD_ADDR_MASK         (0xfffffffc) /* Bits 2-31: Aligned buffer address */

/* Receive buffer descriptor:  Control word */

#define EMACRXD_STA_FRLEN_SHIFT   (0)       /* Bits 0-12: Length of frame (not jumbo)*/
#define EMACRXD_STA_FRLEN_MASK    (0x00001fff << EMACRXD_STA_FRLEN_SHIFT)
#define EMACRXD_STA_JFRLEN_SHIFT  (0)       /* Bits 0-13: Length of frame (jumbo)*/
#define EMACRXD_STA_JFRLEN_MASK   (0x00003fff << EMACRXD_STA_JFRLEN_SHIFT)
#define EMACRXD_STA_BADFCS        (1 << 13) /* Bit 13: Frame had bad FCS (not jumbo) */
#define EMACRXD_STA_SOF           (1 << 14) /* Bit 14: Start of frame */
#define EMACRXD_STA_EOF           (1 << 15) /* Bit 15: End of frame */
#define EMACRXD_STA_CFI           (1 << 16) /* Bit 16: Concatenation format indicator (CFI) bit */
#define EMACRXD_STA_VLPRIO_SHIFT  (17)      /* Bits 17-19: VLAN priority */
#define EMACRXD_STA_VLPRIO_MASK   (7 << EMACRXD_STA_VLANPRIO_SHIFT)
#define EMACRXD_STA_PRIODET       (1 << 20) /* Bit 20: Priority tag detected */
#define EMACRXD_STA_VLANTAG       (1 << 21) /* Bit 21: VLAN tag detected */
#define EMACRXD_STA_TYPEID_SHIFT  (22)      /* Bit 22-23: Specific address register */
#define EMACRXD_STA_TYPEID_MASK   (3 << EMACRXD_STA_TYPEID_SHIFT)
#  define EMACRXD_STA_TYPEID1     (0 << EMACRXD_STA_TYPEID_SHIFT) /* Type ID register 1 match */
#  define EMACRXD_STA_TYPEID2     (1 << EMACRXD_STA_TYPEID_SHIFT) /* Type ID register 2 match */
#  define EMACRXD_STA_TYPEID3     (2 << EMACRXD_STA_TYPEID_SHIFT) /* Type ID register 3 match */
#  define EMACRXD_STA_TYPEID4     (3 << EMACRXD_STA_TYPEID_SHIFT) /* Type ID register 4 match */

#define EMACRXD_STA_TYPEIDMATCH   (1 << 24) /* Bit 24: Type ID register match found */
#define EMACRXD_STA_SNAP          (1 << 24) /* Bit 24: Frame was SNAP encoded */
#define EMACRXD_STA_ADDR_SHIFT    (25)      /* Bit 25-26: Specific address register */
#define EMACRXD_STA_ADDR_MASK     (3 << EMACRXD_STA_ADDR_SHIFT)
#  define EMACRXD_STA_ADDR1       (0 << EMACRXD_STA_ADDR_SHIFT) /* Specific address register 1 match */
#  define EMACRXD_STA_ADDR2       (1 << EMACRXD_STA_ADDR_SHIFT) /* Specific address register 2 match */
#  define EMACRXD_STA_ADDR3       (2 << EMACRXD_STA_ADDR_SHIFT) /* Specific address register 3 match */
#  define EMACRXD_STA_ADDR4       (3 << EMACRXD_STA_ADDR_SHIFT) /* Specific address register 4 match */

#define EMACRXD_STA_ADDRMATCH     (1 << 27) /* Bit 27: Specific address match found */
                                            /* Bit 28: Reserved */
#define EMACRXD_STA_UCAST         (1 << 29) /* Bit 29: Unicast hash match */
#define EMACRXD_STA_MCAST         (1 << 30) /* Bit 30: Multicast hash match */
#define EMACRXD_STA_BCAST         (1 << 31) /* Bit 31: Global all ones broadcast address detected */

/* Transmit buffer descriptor:  Address word (un-aligned, 32-bit address */

/* Transmit buffer descriptor:  Control word */

#define EMACTXD_STA_BUFLEN_SHIFT  (0)       /* Bits 0-13: Length of buffer */
#define EMACTXD_STA_BUFLEN_MASK   (0x00003fff << EMACTXD_STA_BUFLEN_SHIFT)
                                            /* Bit 14: Reserved */
#define EMACTXD_STA_LAST          (1 << 15) /* Bit 15: Last buffer in the current frame */
#define EMACTXD_STA_NOCRC         (1 << 16) /* Bit 16: No CRC */
                                            /* Bits 17-19: Reserved */
#define EMACTXD_STA_CHKERR_SHIFT  (20)      /* Bits 20-22: Transmit IP/TCP/UDP checksum generation offload errors */
#define EMACTXD_STA_CHKERR_MASK   (7 << EMACTXD_STA_CHKERR_SHIFT)
#  define EMACTXD_STA_CHKERR_NONE    (0 << EMACTXD_STA_CHKERR_SHIFT) /* No Error */
#  define EMACTXD_STA_CHKERR_BADVLAN (1 << EMACTXD_STA_CHKERR_SHIFT) /* Incomplete/erroneous VLAN type */
#  define EMACTXD_STA_CHKERR_BADSNAP (2 << EMACTXD_STA_CHKERR_SHIFT) /* Incomplete/erroneous SNAP type */
#  define EMACTXD_STA_CHKERR_NOTIP   (3 << EMACTXD_STA_CHKERR_SHIFT) /* Not IP or invalid IP type */
#  define EMACTXD_STA_CHKERR_UNREC   (4 << EMACTXD_STA_CHKERR_SHIFT) /* Not VLAN, SNAP, or IP */
#  define EMACTXD_STA_CHKERR_BADFRAG (5 << EMACTXD_STA_CHKERR_SHIFT) /* Unsupported fragmentation */
#  define EMACTXD_STA_CHKERR_PKTTYPE (6 << EMACTXD_STA_CHKERR_SHIFT) /* Not TCP or UDP */
#  define EMACTXD_STA_CHKERR_EPKT    (7 << EMACTXD_STA_CHKERR_SHIFT) /* Premature end of packet */

                                            /* Bits 23-25: Reserved */
#define EMACTXD_STA_LCOL          (1 << 26) /* Bit 26: Late collision, transmit error detected */
#define EMACTXD_STA_TFC           (1 << 27) /* Bit 27: Transmit frame corruption due to AHB error */
                                            /* Bit 28: Reserved */
#define EMACTXD_STA_TXERR         (1 << 29) /* Bit 29: Retry limit exceeded, transmit error detected */
#define EMACTXD_STA_WRAP          (1 << 30) /* Bit 30: Last descriptor in descriptor list */
#define EMACTXD_STA_USED          (1 << 31) /* Bit 31: Zero for the EMAC to read from buffer */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Receive buffer descriptor */

struct emac_rxdesc_s
{
  uint32_t addr;     /* Buffer address */
  uint32_t status;   /* RX status and controls */
};

/* Transmit buffer descriptor */

struct emac_txdesc_s
{
  uint32_t addr;     /* Buffer address */
  uint32_t status;   /* TX status and controls */
};

#endif /* SAMV7_NEMAC > 0 */
#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_EMAC_H */
