/****************************************************************************
 * arch/arm64/src/zynq-mpsoc/hardware/zynq_gmac.h
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

#ifndef __ARCH_ARM64_SRC_ZYNQ_MPSOC_HARDWARE_ZYNQ_GMAC_H
#define __ARCH_ARM64_SRC_ZYNQ_MPSOC_HARDWARE_ZYNQ_GMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/zynq_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GMAC Register Offsets ****************************************************/

#define ZYNQ_GMAC_NCR        0x0000 /* Network Control Register */
#define ZYNQ_GMAC_NCFGR      0x0004 /* Network Configuration Register */
#define ZYNQ_GMAC_NSR        0x0008 /* Network Status Register */
#define ZYNQ_GMAC_UR         0x000c /* User Register */
#define ZYNQ_GMAC_DCFGR      0x0010 /* DMA Configuration Register */
#define ZYNQ_GMAC_TSR        0x0014 /* Transmit Status Register */
#define ZYNQ_GMAC_RBQB       0x0018 /* Receive Buffer Queue Base Address */
#define ZYNQ_GMAC_TBQB       0x001c /* Transmit Buffer Queue Base Address */
#define ZYNQ_GMAC_RSR        0x0020 /* Receive Status Register */
#define ZYNQ_GMAC_ISR        0x0024 /* Interrupt Status Register */
#define ZYNQ_GMAC_IER        0x0028 /* Interrupt Enable Register */
#define ZYNQ_GMAC_IDR        0x002c /* Interrupt Disable Register */
#define ZYNQ_GMAC_IMR        0x0030 /* Interrupt Mask Register */
#define ZYNQ_GMAC_MAN        0x0034 /* PHY Maintenance Register */
#define ZYNQ_GMAC_RPQ        0x0038 /* Received Pause Quantum Register */
#define ZYNQ_GMAC_TPQ        0x003c /* Transmit Pause Quantum Register */
#define ZYNQ_GMAC_TPSF       0x0040 /* TX Partial Store and Forward Register */
#define ZYNQ_GMAC_RPSF       0x0044 /* RX Partial Store and Forward Register */

/* 0x0048-0x007c Reserved */

#define ZYNQ_GMAC_HRB        0x0080 /* Hash Register Bottom [31:0] */
#define ZYNQ_GMAC_HRT        0x0084 /* Hash Register Top [63:32] */

#define ZYNQ_GMAC_SAB(n)     (0x0088 + (((n)-1) << 3))  /* n=1..4 */
#define ZYNQ_GMAC_SAT(n)     (0x008c + (((n)-1) << 3))  /* n=1..4 */

#define ZYNQ_GMAC_SAB1       0x0088 /* Specific Address 1 Bottom [31:0] Register */
#define ZYNQ_GMAC_SAT1       0x008c /* Specific Address 1 Top [47:32] Register */
#define ZYNQ_GMAC_SAB2       0x0090 /* Specific Address 2 Bottom [31:0] Register */
#define ZYNQ_GMAC_SAT2       0x0094 /* Specific Address 2 Top [47:32] Register */
#define ZYNQ_GMAC_SAB3       0x0098 /* Specific Address 3 Bottom [31:0] Register */
#define ZYNQ_GMAC_SAT3       0x009c /* Specific Address 3 Top [47:32] Register */
#define ZYNQ_GMAC_SAB4       0x00a0 /* Specific Address 4 Bottom [31:0] Register */
#define ZYNQ_GMAC_SAT4       0x00a4 /* Specific Address 4 Top [47:32] Register */

#define ZYNQ_GMAC_TIDM(n)    (0x00a8 + (((n)-1) << 2))  /* n=1..4 */

#define ZYNQ_GMAC_TIDM1      0x00a8 /* Type ID Match 1 Register */
#define ZYNQ_GMAC_TIDM2      0x00ac /* Type ID Match 2 Register */
#define ZYNQ_GMAC_TIDM3      0x00b0 /* Type ID Match 3 Register */
#define ZYNQ_GMAC_TIDM4      0x00b4 /* Type ID Match 4 Register */
#define ZYNQ_GMAC_WOL        0x00b8 /* Wake on LAN Register */
#define ZYNQ_GMAC_IPGS       0x00bc /* IPG Stretch Register */
#define ZYNQ_GMAC_SVLAN      0x00c0 /* Stacked VLAN Register */
#define ZYNQ_GMAC_TPFCP      0x00c4 /* Transmit PFC Pause Register */
#define ZYNQ_GMAC_ZYNQB1     0x00c8 /* Specific Address 1 Mask Bottom [31:0] Register */
#define ZYNQ_GMAC_ZYNQT1     0x00cc /* Specific Address 1 Mask Top [47:32] Register */

/* 0x00fc Reserved */

#define ZYNQ_GMAC_OTLO       0x0100 /* Octets Transmitted [31:0] Register */
#define ZYNQ_GMAC_OTHI       0x0104 /* Octets Transmitted [47:32] Register */
#define ZYNQ_GMAC_FT         0x0108 /* Frames Transmitted Register */
#define ZYNQ_GMAC_BCFT       0x010c /* Broadcast Frames Transmitted Register */
#define ZYNQ_GMAC_MFT        0x0110 /* Multicast Frames Transmitted Register */
#define ZYNQ_GMAC_PFT        0x0114 /* Pause Frames Transmitted Register */
#define ZYNQ_GMAC_BFT64      0x0118 /* 64 Byte Frames Transmitted Register */
#define ZYNQ_GMAC_TBFT127    0x011c /* 65 to 127 Byte Frames Transmitted Register */
#define ZYNQ_GMAC_TBFT255    0x0120 /* 128 to 255 Byte Frames Transmitted Register */
#define ZYNQ_GMAC_TBFT511    0x0124 /* 256 to 511 Byte Frames Transmitted Register */
#define ZYNQ_GMAC_TBFT1023   0x0128 /* 512 to 1023 Byte Frames Transmitted Register */
#define ZYNQ_GMAC_TBFT1518   0x012c /* 1024 to 1518 Byte Frames Transmitted Register */
#define ZYNQ_GMAC_GTBFT1518  0x0130 /* Greater Than 1518 Byte Frames Transmitted Register */
#define ZYNQ_GMAC_TUR        0x0134 /* Transmit Under Runs Register */
#define ZYNQ_GMAC_SCF        0x0138 /* Single Collision Frames Register */
#define ZYNQ_GMAC_MCF        0x013c /* Multiple Collision Frames Register */
#define ZYNQ_GMAC_EC         0x0140 /* Excessive Collisions Register */
#define ZYNQ_GMAC_LC         0x0144 /* Late Collisions Register */
#define ZYNQ_GMAC_DTF        0x0148 /* Deferred Transmission Frames Register */
#define ZYNQ_GMAC_CSE        0x014c /* Carrier Sense Errors Register */
#define ZYNQ_GMAC_ORLO       0x0150 /* Octets Received [31:0] Received */
#define ZYNQ_GMAC_ORHI       0x0154 /* Octets Received [47:32] Received */
#define ZYNQ_GMAC_FR         0x0158 /* Frames Received Register */
#define ZYNQ_GMAC_BCFR       0x015c /* Broadcast Frames Received Register */
#define ZYNQ_GMAC_MFR        0x0160 /* Multicast Frames Received Register */
#define ZYNQ_GMAC_PFR        0x0164 /* Pause Frames Received Register */
#define ZYNQ_GMAC_BFR64      0x0168 /* 64 Byte Frames Received Register */
#define ZYNQ_GMAC_TBFR127    0x016c /* 65 to 127 Byte Frames Received Register */
#define ZYNQ_GMAC_TBFR255    0x0170 /* 128 to 255 Byte Frames Received Register */
#define ZYNQ_GMAC_TBFR511    0x0174 /* 256 to 511Byte Frames Received Register */
#define ZYNQ_GMAC_TBFR1023   0x0178 /* 512 to 1023 Byte Frames Received Register */
#define ZYNQ_GMAC_TBFR1518   0x017c /* 1024 to 1518 Byte Frames Received Register */
#define ZYNQ_GMAC_TMXBFR     0x0180 /* 1519 to Maximum Byte Frames Received Register */
#define ZYNQ_GMAC_UFR        0x0184 /* Undersize Frames Received Register */
#define ZYNQ_GMAC_OFR        0x0188 /* Oversize Frames Received Register */
#define ZYNQ_GMAC_JR         0x018c /* Jabbers Received Register */
#define ZYNQ_GMAC_FCSE       0x0190 /* Frame Check Sequence Errors Register */
#define ZYNQ_GMAC_LFFE       0x0194 /* Length Field Frame Errors Register */
#define ZYNQ_GMAC_RSE        0x0198 /* Receive Symbol Errors Register */
#define ZYNQ_GMAC_AE         0x019c /* Alignment Errors Register */
#define ZYNQ_GMAC_RRE        0x01a0 /* Receive Resource Errors Register */
#define ZYNQ_GMAC_ROE        0x01a4 /* Receive Overrun Register */
#define ZYNQ_GMAC_IHCE       0x01a8 /* IP Header Checksum Errors Register */
#define ZYNQ_GMAC_TCE        0x01ac /* TCP Checksum Errors Register */
#define ZYNQ_GMAC_UCE        0x01b0 /* UDP Checksum Errors Register */
#define ZYNQ_GMAC_TSSS       0x01c8 /* 1588 Timer Sync Strobe Seconds Register */
#define ZYNQ_GMAC_TSSN       0x01cc /* 1588 Timer Sync Strobe Nanoseconds Register */
#define ZYNQ_GMAC_TS         0x01d0 /* 1588 Timer Seconds Register */
#define ZYNQ_GMAC_TN         0x01d4 /* 1588 Timer Nanoseconds Register */
#define ZYNQ_GMAC_TA         0x01d8 /* 1588 Timer Adjust Register */
#define ZYNQ_GMAC_TI         0x01dc /* 1588 Timer Increment Register */
#define ZYNQ_GMAC_EFTS       0x01e0 /* PTP Event Frame Transmitted Seconds */
#define ZYNQ_GMAC_EFTN       0x01e4 /* PTP Event Frame Transmitted Nanoseconds */
#define ZYNQ_GMAC_EFRS       0x01e8 /* PTP Event Frame Received Seconds */
#define ZYNQ_GMAC_EFRN       0x01ec /* PTP Event Frame Received Nanoseconds */
#define ZYNQ_GMAC_PEFTS      0x01f0 /* PTP Peer Event Frame Transmitted Seconds */
#define ZYNQ_GMAC_PEFTN      0x01f4 /* PTP Peer Event Frame Transmitted Nanoseconds */
#define ZYNQ_GMAC_PEFRS      0x01f8 /* PTP Peer Event Frame Received Seconds */
#define ZYNQ_GMAC_PEFRN      0x01fc /* PTP Peer Event Frame Received Nanoseconds */

/* 0x0200-0x023c Reserved 0x0280-0x0298 Reserved */

#define ZYNQ_GMAC_ISRPQ(n)   (0x400 + ((n) << 2))  /* n=0..6 */

#define ZYNQ_GMAC_ISRPQ0     0x400 /* Interrupt Status Register Priority Queue 0 */
#define ZYNQ_GMAC_ISRPQ1     0x404 /* Interrupt Status Register Priority Queue 1 */
#define ZYNQ_GMAC_ISRPQ2     0x408 /* Interrupt Status Register Priority Queue 2 */
#define ZYNQ_GMAC_ISRPQ3     0x40c /* Interrupt Status Register Priority Queue 3 */
#define ZYNQ_GMAC_ISRPQ4     0x410 /* Interrupt Status Register Priority Queue 4 */
#define ZYNQ_GMAC_ISRPQ5     0x414 /* Interrupt Status Register Priority Queue 5 */
#define ZYNQ_GMAC_ISRPQ6     0x418 /* Interrupt Status Register Priority Queue 6 */

#define ZYNQ_GMAC_TBQBAPQ(n) (0x440 + ((n) << 2))  /* n=0..6 */

#define ZYNQ_GMAC_TBQBAPQ0   0x440 /* Transmit Buffer Queue Base Address Priority Queue 0 */
#define ZYNQ_GMAC_TBQBAPQ1   0x444 /* Transmit Buffer Queue Base Address Priority Queue 1 */
#define ZYNQ_GMAC_TBQBAPQ2   0x448 /* Transmit Buffer Queue Base Address Priority Queue 2 */
#define ZYNQ_GMAC_TBQBAPQ3   0x44c /* Transmit Buffer Queue Base Address Priority Queue 3 */
#define ZYNQ_GMAC_TBQBAPQ4   0x450 /* Transmit Buffer Queue Base Address Priority Queue 4 */
#define ZYNQ_GMAC_TBQBAPQ5   0x454 /* Transmit Buffer Queue Base Address Priority Queue 5 */
#define ZYNQ_GMAC_TBQBAPQ6   0x458 /* Transmit Buffer Queue Base Address Priority Queue 6 */

#define ZYNQ_GMAC_RBQBAPQ(n) (0x480 + ((n) << 2))  /* n=0..6 */

#define ZYNQ_GMAC_RBQBAPQ0   0x480 /* Receive Buffer Queue Base Address Priority Queue 0 */
#define ZYNQ_GMAC_RBQBAPQ1   0x484 /* Receive Buffer Queue Base Address Priority Queue 1 */
#define ZYNQ_GMAC_RBQBAPQ2   0x488 /* Receive Buffer Queue Base Address Priority Queue 2 */
#define ZYNQ_GMAC_RBQBAPQ3   0x48c /* Receive Buffer Queue Base Address Priority Queue 3 */
#define ZYNQ_GMAC_RBQBAPQ4   0x490 /* Receive Buffer Queue Base Address Priority Queue 4 */
#define ZYNQ_GMAC_RBQBAPQ5   0x494 /* Receive Buffer Queue Base Address Priority Queue 5 */
#define ZYNQ_GMAC_RBQBAPQ6   0x498 /* Receive Buffer Queue Base Address Priority Queue 6 */

#define ZYNQ_GMAC_RBSRPQ(n)  (0x4a0 + ((n) << 2))  /* n=0..6 */

#define ZYNQ_GMAC_RBSRPQ0    0x4a0 /* Receive Buffer Size Register Priority Queue 0 */
#define ZYNQ_GMAC_RBSRPQ1    0x4a4 /* Receive Buffer Size Register Priority Queue 1 */
#define ZYNQ_GMAC_RBSRPQ2    0x4a8 /* Receive Buffer Size Register Priority Queue 2 */
#define ZYNQ_GMAC_RBSRPQ3    0x4ac /* Receive Buffer Size Register Priority Queue 3 */
#define ZYNQ_GMAC_RBSRPQ4    0x4b0 /* Receive Buffer Size Register Priority Queue 4 */
#define ZYNQ_GMAC_RBSRPQ5    0x4b4 /* Receive Buffer Size Register Priority Queue 5 */
#define ZYNQ_GMAC_RBSRPQ6    0x4b8 /* Receive Buffer Size Register Priority Queue 6 */

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
#define GMAC_NCFGR_GBE            (1 << 10) /* Bit 10: Gigabit Mode Enable */
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

#define GMAC_UR_RGMII             (1 << 0)  /* Bit 0:  Reduced GMII Mode */

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

#define GMAC_INT_ALL              (0x3ffffeff)
#define GMAC_INT_UNUSED           (0xc0000100)

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

#define GMAC_ZYNQT1_MASK           (0x0000ffff) /* Bits 0-15: Specific Address 1 Mask [47:32] */

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
 *      GMAC_INT_RCOMP    Bit 1:  Receive Complete
 *      GMAC_INT_RXUBR    Bit 2:  Receive Used Bit Read
 *      GMAC_INT_RLEX     Bit 5:  Retry Limit Exceeded or Late Collision
 *      GMAC_INT_TFC      Bit 6:  Transmit Frame Corruption due to AHB error
 *      GMAC_INT_TCOMP    Bit 7:  Transmit Complete
 *      GMAC_INT_ROVR     Bit 10: Receive Overrun
 *      GMAC_INT_HRESP    Bit 11: HRESP not OK
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

#define GMACRXD_STA_TYPID_SHIFT   (22) /* Bits 22-23: Type ID register match */
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

#define GMAC_STATUS_LENGTH_MASK   0x3fffu

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

#endif /* __ARCH_ARM54_SRC_ZYNQ_MPSOC_HARDWARE_ZYNQ_GMAC_H */
