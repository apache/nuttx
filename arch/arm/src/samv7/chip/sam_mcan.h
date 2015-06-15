/****************************************************************************************
 * arch/arm/src/samv7/chip/sam_mcan.h
 * Controller Area Network (MCAN) definitions for the SAMV71
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMV7_CHIP_SAM_MCAN_H
#define __ARCH_ARM_SRC_SAMV7_CHIP_SAM_MCAN_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* MCAN register offsets ****************************************************************/

                                          /* 0x0000-0x0004 Reserved */
#define SAM_MCAN_CUST_OFFSET       0x0008 /* Customer Register */
#define SAM_MCAN_FBTP_OFFSET       0x000c /* Fast Bit Timing and Prescaler Register */
#define SAM_MCAN_TEST_OFFSET       0x0010 /* Test Register */
#define SAM_MCAN_RWD_OFFSET        0x0014 /* RAM Watchdog Register */
#define SAM_MCAN_CCCR_OFFSET       0x0018 /* CC Control Register */
#define SAM_MCAN_BTP_OFFSET        0x001c /* Bit Timing and Prescaler Register */
#define SAM_MCAN_TSCC_OFFSET       0x0020 /* Timestamp Counter Configuration Register */
#define SAM_MCAN_TSCV_OFFSET       0x0024 /* Timestamp Counter Value Register */
#define SAM_MCAN_TOCC_OFFSET       0x0028 /* Timeout Counter Configuration Register */
#define SAM_MCAN_TOCV_OFFSET       0x002c /* Timeout Counter Value Register */
                                          /* 0x0030-0x003c Reserved */
#define SAM_MCAN_ECR_OFFSET        0x0040 /* Error Counter Register */
#define SAM_MCAN_PSR_OFFSET        0x0044 /* Protocol Status Register */
                                          /* 0x0048-0x004c Reserved */
#define SAM_MCAN_IR_OFFSET         0x0050 /* Interrupt Register*/ 
#define SAM_MCAN_IE_OFFSET         0x0054 /* Interrupt Enable Register */
#define SAM_MCAN_ILS_OFFSET        0x0058 /* Interrupt Line Select Register */
#define SAM_MCAN_ILE_OFFSET        0x005c /* Interrupt Line Enable Register */
                                          /* 0x0060-0x007c Reserved */
#define SAM_MCAN_GFC_OFFSET        0x0080 /* Global Filter Configuration Register */
#define SAM_MCAN_SIDFC_OFFSET      0x0084 /* Standard ID Filter Configuration Register */
#define SAM_MCAN_XIDFC_OFFSET      0x0088 /* Extended ID Filter Configuration Register */
                                          /* 0x008c Reserved */
#define SAM_MCAN_XIDAM_OFFSET      0x0090 /* Extended ID AND Mask Register */
#define SAM_MCAN_HPMS_OFFSET       0x0094 /* High Priority Message Status Register */
#define SAM_MCAN_NDAT1_OFFSET      0x0098 /* New Data 1 Register */
#define SAM_MCAN_NDAT2_OFFSET      0x009c /* New Data 2 Register */
#define SAM_MCAN_RXF0C_OFFSET      0x00a0 /* Receive FIFO 0 Configuration Register */
#define SAM_MCAN_RXF0S_OFFSET      0x00a4 /* Receive FIFO 0 Status Register */
#define SAM_MCAN_RXF0A_OFFSET      0x00a8 /* Receive FIFO 0 Acknowledge Register */
#define SAM_MCAN_RXBC_OFFSET       0x00ac /* Receive Rx Buffer Configuration Register */
#define SAM_MCAN_RXF1C_OFFSET      0x00b0 /* Receive FIFO 1 Configuration Register */
#define SAM_MCAN_RXF1S_OFFSET      0x00b4 /* Receive FIFO 1 Status Register */
#define SAM_MCAN_RXF1A_OFFSET      0x00b8 /* Receive FIFO 1 Acknowledge Register */
#define SAM_MCAN_RXESC_OFFSET      0x00bc /* Receive Buffer / FIFO Element Size Configuration Register */
#define SAM_MCAN_TXBC_OFFSET       0x00c0 /* Transmit Buffer Configuration Register */
#define SAM_MCAN_TXFQS_OFFSET      0x00c4 /* Transmit FIFO/Queue Status Register */
#define SAM_MCAN_TXESC_OFFSET      0x00c8 /* Transmit Buffer Element Size Configuration Register */
#define SAM_MCAN_TXBRP_OFFSET      0x00cc /* Transmit Buffer Request Pending Register */
#define SAM_MCAN_TXBAR_OFFSET      0x00d0 /* Transmit Buffer Add Request Register */
#define SAM_MCAN_TXBCR_OFFSET      0x00d4 /* Transmit Buffer Cancellation Request Register */
#define SAM_MCAN_TXBTO_OFFSET      0x00d8 /* Transmit Buffer Transmission Occurred Register */
#define SAM_MCAN_TXBCF_OFFSET      0x00dc /* Transmit Buffer Cancellation Finished Register */
#define SAM_MCAN_TXBTIE_OFFSET     0x00e0 /* Transmit Buffer Transmission Interrupt Enable Register */
#define SAM_MCAN_TXBCIE_OFFSET     0x00e4 /* Transmit Buffer Cancellation Finished Interrupt Enable Register */
                                          /* 0x00e8-0x00ec Reserved */
#define SAM_MCAN_TXEFC_OFFSET      0x00f0 /* Transmit Event FIFO Configuration Register */
#define SAM_MCAN_TXEFS_OFFSET      0x00f4 /* Transmit Event FIFO Status Register */
#define SAM_MCAN_TXEFA_OFFSET      0x00f8 /* Transmit Event FIFO Acknowledge Register */
                                          /* 0x00fc Reserved */

/* MCAN register addresses **************************************************************/

#define SAM_MCAN0_CUST             (SAM_MCAN0_BASE+SAM_MCAN_CUST_OFFSET)
#define SAM_MCAN0_FBTP             (SAM_MCAN0_BASE+SAM_MCAN_FBTP_OFFSET)
#define SAM_MCAN0_TEST             (SAM_MCAN0_BASE+SAM_MCAN_TEST_OFFSET)
#define SAM_MCAN0_RWD              (SAM_MCAN0_BASE+SAM_MCAN_RWD_OFFSET)
#define SAM_MCAN0_CCCR             (SAM_MCAN0_BASE+SAM_MCAN_CCCR_OFFSET)
#define SAM_MCAN0_BTP              (SAM_MCAN0_BASE+SAM_MCAN_BTP_OFFSET)
#define SAM_MCAN0_TSCC             (SAM_MCAN0_BASE+SAM_MCAN_TSCC_OFFSET)
#define SAM_MCAN0_TSCV             (SAM_MCAN0_BASE+SAM_MCAN_TSCV_OFFSET)
#define SAM_MCAN0_TOCC             (SAM_MCAN0_BASE+SAM_MCAN_TOCC_OFFSET)
#define SAM_MCAN0_TOCV             (SAM_MCAN0_BASE+SAM_MCAN_TOCV_OFFSET)
#define SAM_MCAN0_ECR              (SAM_MCAN0_BASE+SAM_MCAN_ECR_OFFSET)
#define SAM_MCAN0_PSR              (SAM_MCAN0_BASE+SAM_MCAN_PSR_OFFSET)
#define SAM_MCAN0_IR               (SAM_MCAN0_BASE+SAM_MCAN_IR_OFFSET)
#define SAM_MCAN0_IE               (SAM_MCAN0_BASE+SAM_MCAN_IE_OFFSET)
#define SAM_MCAN0_ILS              (SAM_MCAN0_BASE+SAM_MCAN_ILS_OFFSET)
#define SAM_MCAN0_ILE              (SAM_MCAN0_BASE+SAM_MCAN_ILE_OFFSET)
#define SAM_MCAN0_GFC              (SAM_MCAN0_BASE+SAM_MCAN_GFC_OFFSET)
#define SAM_MCAN0_SIDFC            (SAM_MCAN0_BASE+SAM_MCAN_SIDFC_OFFSET)
#define SAM_MCAN0_XIDFC            (SAM_MCAN0_BASE+SAM_MCAN_XIDFC_OFFSET)
#define SAM_MCAN0_XIDAM            (SAM_MCAN0_BASE+SAM_MCAN_XIDAM_OFFSET)
#define SAM_MCAN0_HPMS             (SAM_MCAN0_BASE+SAM_MCAN_HPMS_OFFSET)
#define SAM_MCAN0_NDAT1            (SAM_MCAN0_BASE+SAM_MCAN_NDAT1_OFFSET)
#define SAM_MCAN0_NDAT2            (SAM_MCAN0_BASE+SAM_MCAN_NDAT2_OFFSET)
#define SAM_MCAN0_RXF0C            (SAM_MCAN0_BASE+SAM_MCAN_RXF0C_OFFSET)
#define SAM_MCAN0_RXF0S            (SAM_MCAN0_BASE+SAM_MCAN_RXF0S_OFFSET)
#define SAM_MCAN0_RXF0A            (SAM_MCAN0_BASE+SAM_MCAN_RXF0A_OFFSET)
#define SAM_MCAN0_RXBC             (SAM_MCAN0_BASE+SAM_MCAN_RXBC_OFFSET)
#define SAM_MCAN0_RXF1C            (SAM_MCAN0_BASE+SAM_MCAN_RXF1C_OFFSET)
#define SAM_MCAN0_RXF1S            (SAM_MCAN0_BASE+SAM_MCAN_RXF1S_OFFSET)
#define SAM_MCAN0_RXF1A            (SAM_MCAN0_BASE+SAM_MCAN_RXF1A_OFFSET)
#define SAM_MCAN0_RXESC            (SAM_MCAN0_BASE+SAM_MCAN_RXESC_OFFSET)
#define SAM_MCAN0_TXBC             (SAM_MCAN0_BASE+SAM_MCAN_TXBC_OFFSET)
#define SAM_MCAN0_TXFQS            (SAM_MCAN0_BASE+SAM_MCAN_TXFQS_OFFSET)
#define SAM_MCAN0_TXESC            (SAM_MCAN0_BASE+SAM_MCAN_TXESC_OFFSET)
#define SAM_MCAN0_TXBRP            (SAM_MCAN0_BASE+SAM_MCAN_TXBRP_OFFSET)
#define SAM_MCAN0_TXBAR            (SAM_MCAN0_BASE+SAM_MCAN_TXBAR_OFFSET)
#define SAM_MCAN0_TXBCR            (SAM_MCAN0_BASE+SAM_MCAN_TXBCR_OFFSET)
#define SAM_MCAN0_TXBTO            (SAM_MCAN0_BASE+SAM_MCAN_TXBTO_OFFSET)
#define SAM_MCAN0_TXBCF            (SAM_MCAN0_BASE+SAM_MCAN_TXBCF_OFFSET)
#define SAM_MCAN0_TXBTIE           (SAM_MCAN0_BASE+SAM_MCAN_TXBTIE_OFFSET)
#define SAM_MCAN0_TXBCIE           (SAM_MCAN0_BASE+SAM_MCAN_TXBCIE_OFFSET)
#define SAM_MCAN0_TXEFC            (SAM_MCAN0_BASE+SAM_MCAN_TXEFC_OFFSET)
#define SAM_MCAN0_TXEFS            (SAM_MCAN0_BASE+SAM_MCAN_TXEFS_OFFSET)
#define SAM_MCAN0_TXEFA            (SAM_MCAN0_BASE+SAM_MCAN_TXEFA_OFFSET)

#define SAM_MCAN1_CUST             (SAM_MCAN1_BASE+SAM_MCAN_CUST_OFFSET)
#define SAM_MCAN1_FBTP             (SAM_MCAN1_BASE+SAM_MCAN_FBTP_OFFSET)
#define SAM_MCAN1_TEST             (SAM_MCAN1_BASE+SAM_MCAN_TEST_OFFSET)
#define SAM_MCAN1_RWD              (SAM_MCAN1_BASE+SAM_MCAN_RWD_OFFSET)
#define SAM_MCAN1_CCCR             (SAM_MCAN1_BASE+SAM_MCAN_CCCR_OFFSET)
#define SAM_MCAN1_BTP              (SAM_MCAN1_BASE+SAM_MCAN_BTP_OFFSET)
#define SAM_MCAN1_TSCC             (SAM_MCAN1_BASE+SAM_MCAN_TSCC_OFFSET)
#define SAM_MCAN1_TSCV             (SAM_MCAN1_BASE+SAM_MCAN_TSCV_OFFSET)
#define SAM_MCAN1_TOCC             (SAM_MCAN1_BASE+SAM_MCAN_TOCC_OFFSET)
#define SAM_MCAN1_TOCV             (SAM_MCAN1_BASE+SAM_MCAN_TOCV_OFFSET)
#define SAM_MCAN1_ECR              (SAM_MCAN1_BASE+SAM_MCAN_ECR_OFFSET)
#define SAM_MCAN1_PSR              (SAM_MCAN1_BASE+SAM_MCAN_PSR_OFFSET)
#define SAM_MCAN1_IR               (SAM_MCAN1_BASE+SAM_MCAN_IR_OFFSET)
#define SAM_MCAN1_IE               (SAM_MCAN1_BASE+SAM_MCAN_IE_OFFSET)
#define SAM_MCAN1_ILS              (SAM_MCAN1_BASE+SAM_MCAN_ILS_OFFSET)
#define SAM_MCAN1_ILE              (SAM_MCAN1_BASE+SAM_MCAN_ILE_OFFSET)
#define SAM_MCAN1_GFC              (SAM_MCAN1_BASE+SAM_MCAN_GFC_OFFSET)
#define SAM_MCAN1_SIDFC            (SAM_MCAN1_BASE+SAM_MCAN_SIDFC_OFFSET)
#define SAM_MCAN1_XIDFC            (SAM_MCAN1_BASE+SAM_MCAN_XIDFC_OFFSET)
#define SAM_MCAN1_XIDAM            (SAM_MCAN1_BASE+SAM_MCAN_XIDAM_OFFSET)
#define SAM_MCAN1_HPMS             (SAM_MCAN1_BASE+SAM_MCAN_HPMS_OFFSET)
#define SAM_MCAN1_NDAT1            (SAM_MCAN1_BASE+SAM_MCAN_NDAT1_OFFSET)
#define SAM_MCAN1_NDAT2            (SAM_MCAN1_BASE+SAM_MCAN_NDAT2_OFFSET)
#define SAM_MCAN1_RXF0C            (SAM_MCAN1_BASE+SAM_MCAN_RXF0C_OFFSET)
#define SAM_MCAN1_RXF0S            (SAM_MCAN1_BASE+SAM_MCAN_RXF0S_OFFSET)
#define SAM_MCAN1_RXF0A            (SAM_MCAN1_BASE+SAM_MCAN_RXF0A_OFFSET)
#define SAM_MCAN1_RXBC             (SAM_MCAN1_BASE+SAM_MCAN_RXBC_OFFSET)
#define SAM_MCAN1_RXF1C            (SAM_MCAN1_BASE+SAM_MCAN_RXF1C_OFFSET)
#define SAM_MCAN1_RXF1S            (SAM_MCAN1_BASE+SAM_MCAN_RXF1S_OFFSET)
#define SAM_MCAN1_RXF1A            (SAM_MCAN1_BASE+SAM_MCAN_RXF1A_OFFSET)
#define SAM_MCAN1_RXESC            (SAM_MCAN1_BASE+SAM_MCAN_RXESC_OFFSET)
#define SAM_MCAN1_TXBC             (SAM_MCAN1_BASE+SAM_MCAN_TXBC_OFFSET)
#define SAM_MCAN1_TXFQS            (SAM_MCAN1_BASE+SAM_MCAN_TXFQS_OFFSET)
#define SAM_MCAN1_TXESC            (SAM_MCAN1_BASE+SAM_MCAN_TXESC_OFFSET)
#define SAM_MCAN1_TXBRP            (SAM_MCAN1_BASE+SAM_MCAN_TXBRP_OFFSET)
#define SAM_MCAN1_TXBAR            (SAM_MCAN1_BASE+SAM_MCAN_TXBAR_OFFSET)
#define SAM_MCAN1_TXBCR            (SAM_MCAN1_BASE+SAM_MCAN_TXBCR_OFFSET)
#define SAM_MCAN1_TXBTO            (SAM_MCAN1_BASE+SAM_MCAN_TXBTO_OFFSET)
#define SAM_MCAN1_TXBCF            (SAM_MCAN1_BASE+SAM_MCAN_TXBCF_OFFSET)
#define SAM_MCAN1_TXBTIE           (SAM_MCAN1_BASE+SAM_MCAN_TXBTIE_OFFSET)
#define SAM_MCAN1_TXBCIE           (SAM_MCAN1_BASE+SAM_MCAN_TXBCIE_OFFSET)
#define SAM_MCAN1_TXEFC            (SAM_MCAN1_BASE+SAM_MCAN_TXEFC_OFFSET)
#define SAM_MCAN1_TXEFS            (SAM_MCAN1_BASE+SAM_MCAN_TXEFS_OFFSET)
#define SAM_MCAN1_TXEFA            (SAM_MCAN1_BASE+SAM_MCAN_TXEFA_OFFSET)

/* MCAN register bit definitions ********************************************************/

/* Customer Register (32-bit value)*/

/* Fast Bit Timing and Prescaler Register */

#define MCAN_FBTP_FSJW_SHIFT       (0)       /* Bits 0-1: Fast (Re) Synchronization Jump Width */
#define MCAN_FBTP_FSJW_MASK        (3 << MCAN_FBTP_FSJW_SHIFT)
#  define MCAN_FBTP_FSJW(n)        ((uint32_t)(n) << MCAN_FBTP_FSJW_SHIFT)
#define MCAN_FBTP_FTSEG2_SHIFT     (4)       /* Bits 4-6: Fast Time Segment After Sample Point */
#define MCAN_FBTP_FTSEG2_MASK      (7 << MCAN_FBTP_FTSEG2_SHIFT)
#  define MCAN_FBTP_FTSEG2(n)      ((uint32_t)(n) << MCAN_FBTP_FTSEG2_SHIFT)
#define MCAN_FBTP_FTSEG1_SHIFT     (8)       /* Bits 8-11: Fast Time Segment Before Sample Point */
#define MCAN_FBTP_FTSEG1_MASK      (15 << MCAN_FBTP_FTSEG1_SHIFT)
#  define MCAN_FBTP_FTSEG1(n)      ((uint32_t)(n) << MCAN_FBTP_FTSEG1_SHIFT)
#define MCAN_FBTP_FBRP_SHIFT       (16)      /* Bits 16-20: Fast Baud Rate Prescaler */
#define MCAN_FBTP_FBRP_MASK        (31 << MCAN_FBTP_FBRP_SHIFT)
#  define MCAN_FBTP_FBRP(n)        ((uint32_t)(n) << MCAN_FBTP_FBRP_SHIFT)
#define MCAN_FBTP_TDCO             (1 << 23) /* Bit: 23: Transceiver Delay Compensation */
#define MCAN_FBTP_TDCO_SHIFT       (24)      /* Bits 24-28: Transceiver Delay Compensation Offset */
#define MCAN_FBTP_TDCO_MASK        (31 << MCAN_FBTP_TDC_SHIFT)
#  define MCAN_FBTP_TDCO(n)        ((uint32_t)(n) << MCAN_FBTP_TDC_SHIFT)

/* Test Register */

#define MCAN_TEST_LBCK             (1 << 4)  /* Bit 4:  Loop Back Mode */
#define MCAN_TEST_TX_SHIFT         (5)       /* Bits 5-6: Control of Transmit Pin */
#define MCAN_TEST_TX_MASK          (3 << MCAN_TEST_TX_SHIFT)
#  define MCAN_TEST_TX_RESET       (0 << MCAN_TEST_TX_SHIFT) /* Reset value */
#  define MCAN_TEST_TX_SPMON       (1 << MCAN_TEST_TX_SHIFT) /* Sample Point can be monitored at CANTX */
#  define MCAN_TEST_TX_DOMINANT    (2 << MCAN_TEST_TX_SHIFT) /* Dominant (0) level at CANTX. */
#  define MCAN_TEST_TX_RECESSIVE   (3 << MCAN_TEST_TX_SHIFT) /* Recessive (1) at CANTX. */
#define MCAN_TEST_RX               (1 << 7)  /* Bit 7:  Receive Pin */
#define MCAN_TEST_TDCV_SHIFT       (8)       /* Bits 8-13: Transceiver Delay Compensation Value */
#define MCAN_TEST_TDCV_MASK        (0x3f << MCAN_TEST_TDCV_SHIFT)
#  define MCAN_TEST_TDCV(n)        ((uint32_t)(n) << MCAN_TEST_TDCV_SHIFT)

/* RAM Watchdog Register */

#define MCAN_RWD_WDC_SHIFT         (0)       /* Bits 0-7: Watchdog Configuration */
#define MCAN_RWD_WDC_MASK          (0xff << MCAN_RWD_WDC_SHIFT)
#  define MCAN_RWD_WDC(n)          ((uint32_t)(n) << MCAN_RWD_WDC_SHIFT)
#define MCAN_RWD_WDV_SHIFT         (8)       /* Bits 8-15: Watchdog Value */
#define MCAN_RWD_WDV_MASK          (0xff << MCAN_RWD_WDV_SHIFT)
#  define MCAN_RWD_WDV(n)          ((uint32_t)(n) << MCAN_RWD_WDV_SHIFT)

/* CC Control Register */

#define MCAN_CCCR_INIT             (1 << 0)  /* Bit 0:  Initialization */
#define MCAN_CCCR_CCE              (1 << 1)  /* Bit 1:  Configuration Change Enable */
#define MCAN_CCCR_ASM              (1 << 2)  /* Bit 2:  Restricted Operation Mode */
#define MCAN_CCCR_CSA              (1 << 3)  /* Bit 3:  Clock Stop Acknowledge */
#define MCAN_CCCR_CSR              (1 << 4)  /* Bit 4:  Clock Stop Request */
#define MCAN_CCCR_MON              (1 << 5)  /* Bit 5:  Bus Monitoring Mode */
#define MCAN_CCCR_DAR              (1 << 6)  /* Bit 6:  Disable Automatic Retransmission */
#define MCAN_CCCR_TEST             (1 << 7)  /* Bit 7:  Test Mode Enable */
#define MCAN_CCCR_CME_SHIFT        (8)       /* Bits 8-9: CAN Mode Enable */
#define MCAN_CCCR_CME_MASK         (3 << MCAN_CCCR_CME_SHIFT)
#  define MCAN_CCCR_CME_ISO11898_1 (0 << MCAN_CCCR_CME_SHIFT) /* CAN operation according to ISO11898-1 enabled */
#  define MCAN_CCCR_CME_FD         (1 << MCAN_CCCR_CME_SHIFT) /* CAN FD operation enabled */
#  define MCAN_CCCR_CME_FD_BSW     (2 << MCAN_CCCR_CME_SHIFT) /* CAN FD operation with bit rate switching enabled */
#define MCAN_CCCR_CMR_SHIFT        (10)      /* Bits 10-11: CAN Mode Request */
#define MCAN_CCCR_CMR_MASK         (3 << MCAN_CCCR_CMR_SHIFT)
#  define MCAN_CCCR_CMR_NOCHG      (0 << MCAN_CCCR_CMR_SHIFT) /* No mode change */
#  define MCAN_CCCR_CMR_FD         (1 << MCAN_CCCR_CMR_SHIFT) /* Request CAN FD operation */
#  define MCAN_CCCR_CMR_FD_BSW     (2 << MCAN_CCCR_CMR_SHIFT) /* Request CAN FD operation with bit rate switching */
#  define MCAN_CCCR_CMR_ISO11898_1 (3 << MCAN_CCCR_CMR_SHIFT) /* Request CAN operation according ISO11898-1 */
#define MCAN_CCCR_FDO              (1 << 12) /* Bit 12: CAN FD Operation */
#define MCAN_CCCR_FDBS             (1 << 13) /* Bit 13: CAN FD Bit Rate Switching */
#define MCAN_CCCR_TXP              (1 << 14) /* Bit 14: Transmit Pause */

/* Bit Timing and Prescaler Register */

#define MCAN_BTP_SJW_SHIFT         (0)       /* Bits 0-3: (Re) Synchronization Jump Width */
#define MCAN_BTP_SJW_MASK          (15 << MCAN_BTP_SJW_SHIFT)
#  define MCAN_BTP_SJW(n)          ((uint32_t)(n) << MCAN_BTP_SJW_SHIFT)
#define MCAN_BTP_TSEG2_SHIFT       (4)       /* Bits 4-7: Time Segment After Sample Point */
#define MCAN_BTP_TSEG2_MASK        (15 << MCAN_BTP_TSEG2_SHIFT)
#  define MCAN_BTP_TSEG2(n)        ((uint32_t)(n) << MCAN_BTP_TSEG2_SHIFT)
#define MCAN_BTP_TSEG1_SHIFT       (8)       /* Bits 8-13: Time Segment Before Sample Point */
#define MCAN_BTP_TSEG1_MASK        (0x3f << MCAN_BTP_TSEG1_SHIFT)
#  define MCAN_BTP_TSEG1(n)        ((uint32_t)(n) << MCAN_BTP_TSEG1_SHIFT)
#define MCAN_BTP_BRP_SHIFT         (16)      /* Bits 16-25: Baud Rate Prescaler */
#define MCAN_BTP_BRP_MASK          (0x3ff << MCAN_BTP_BRP_SHIFT)
#  define MCAN_BTP_BRP(n)          ((uint32_t)(n) << MCAN_BTP_BRP_SHIFT)

/* Timestamp Counter Configuration Register */

#define MCAN_TSCC_TSS_SHIFT        (0)       /* Bits 0-1: Timestamp Select */
#define MCAN_TSCC_TSS_MASK         (3 << MCAN_TSCC_TSS_SHIFT)
#  define MCAN_TSCC_TSS_ ZERO      (0 << MCAN_TSCC_TSS_SHIFT) /* Timestamp counter value always 0x0000 */
#  define MCAN_TSCC_TSS_TCP_INC    (1 << MCAN_TSCC_TSS_SHIFT) /* Timestamp counter value incremented according to TCP */
#  define MCAN_TSCC_TSS_EXT_TS     (2 << MCAN_TSCC_TSS_SHIFT) /* External timestamp counter value used */
#define MCAN_TSCC_TCP_SHIFT        (16)      /* Bits 16-19: Timestamp Counter Prescaler */
#define MCAN_TSCC_TCP_MASK         (15 << MCAN_TSCC_TCP_SHIFT)
#  define MCAN_TSCC_TCP(n)         ((uint32_t)(n) << MCAN_TSCC_TCP_SHIFT)

/* Timestamp Counter Value Register */

#define MCAN_TSCV_MASK             0x0000ffff /* Timestamp counter mask */

/* Timeout Counter Configuration Register */

#define MCAN_TOCC_ETOC             (1 << 0)  /* Bit 0:  Enable Timeout Counter */
#define MCAN_TOCC_TOS_SHIFT        (1)       /* Bits 1-2: Timeout Select */
#define MCAN_TOCC_TOS_MASK         (3 << MCAN_TOCC_TOS_SHIFT)
#  define MCAN_TOCC_TOS_CONTINUOUS   (0 << MCAN_TOCC_TOS_SHIFT) /* Continuous operation */
#  define MCAN_TOCC_TOS_TX_TIMEOUT   (1 << MCAN_TOCC_TOS_SHIFT) /* Timeout controlled by Tx Event FIFO */
#  define MCAN_TOCC_TOS_RX0_TIMEOUT  (2 << MCAN_TOCC_TOS_SHIFT) /* Timeout controlled by Receive FIFO 0 */
#  define MCAN_TOCC_TOS_RX1_TIMEOUT  (3 << MCAN_TOCC_TOS_SHIFT) /* Timeout controlled by Receive FIFO 1 */
#define MCAN_TOCC_TOP_SHIFT        (16)      /* Bits 16-31: Timeout Period */
#define MCAN_TOCC_TOP_MASK         (0xffff << MCAN_TOCC_TOP_SHIFT)
#  define MCAN_TOCC_TOP(n)         ((uint32_t)(n) << MCAN_TOCC_TOP_SHIFT)

/* Timeout Counter Value Register */

#define MCAN_TOCV_MASK             0x0000ffff /* Timeout counter mask */

/* Error Counter Register */

#define MCAN_ECR_TEC_SHIFT         (0)       /* Bits 0-7: Transmit Error Counter */
#define MCAN_ECR_TEC_MASK          (0xff << MCAN_ECR_TEC_SHIFT)
#  define MCAN_ECR_TEC(n)          ((uint32_t)(n) << MCAN_ECR_TEC_SHIFT)
#define MCAN_ECR_REC_SHIFT         (8)       /* Bits 8-14: Receive Error Counter */
#define MCAN_ECR_REC_MASK          (0x7f << MCAN_ECR_REC_SHIFT)
#  define MCAN_ECR_REC(n)          ((uint32_t)(n) << MCAN_ECR_REC_SHIFT)
#define MCAN_ECR_RP                (1 << 15) /* Bit 15: Receive Error Passive */
#define MCAN_ECR_CEL_SHIFT         (16)      /* Bits 16-23: CAN Error Logging */
#define MCAN_ECR_CEL_MASK          (0xff << MCAN_ECR_CEL_SHIFT)
#  define MCAN_ECR_CEL(n)          ((uint32_t)(n) << MCAN_ECR_CEL_SHIFT)

/* Protocol Status Register */
/* Error codes */

#define MCAN_PSR_EC_NO_ERROR       (0)       /* No error occurred since LEC has been reset */
#define MCAN_PSR_EC_STUFF_ERROR    (1)       /* More than 5 equal bits in a sequence */
#define MCAN_PSR_EC_FORM_ERROR     (2)       /* Part of a received frame has wrong format */
#define MCAN_PSR_EC_ACK_ERROR      (3)       /* Message not acknowledged by another node */
#define MCAN_PSR_EC_BIT1_ERROR     (4)       /* Send with recessive level, but bus value was dominant */
#define MCAN_PSR_EC_BIT0_ERROR     (5)       /* Send with dominant level, but bus value was recessive */
#define MCAN_PSR_EC_CRC_ERROR      (6)       /* CRC received message incorrect */
#define MCAN_PSR_EC_NO_CHANGE      (7)       /* No CAN bus event was detected since last read */

#define MCAN_PSR_LEC_SHIFT         (0)       /* Bits 0-2: Last Error Code */
#define MCAN_PSR_LEC_MASK          (7 << MCAN_PSR_LEC_SHIFT)
#  define MCAN_PSR_LEC(n)          ((uint32_t)(n) << MCAN_PSR_LEC_SHIFT) /* See error codes above */
#define MCAN_PSR_ACT_SHIFT         (3)       /* Bits 3-4: Activity */
#define MCAN_PSR_ACT_MASK          (3 << MCAN_PSR_ACT_SHIFT)
#  define MCAN_PSR_ACT_SYNCHING    (0 << MCAN_PSR_ACT_SHIFT) /* Node is synchronizing on CAN communication */
#  define MCAN_PSR_ACT_IDLE        (1 << MCAN_PSR_ACT_SHIFT) /* Node is neither receiver nor transmitter */
#  define MCAN_PSR_ACT_RECEIVER    (2 << MCAN_PSR_ACT_SHIFT) /* Node is operating as receiver */
#  define MCAN_PSR_ACT_TRANSMITTER (3 << MCAN_PSR_ACT_SHIFT) /* Node is operating as transmitter */
#define MCAN_PSR_EP                (1 << 5)  /* Bit 5:  Error Passive */
#define MCAN_PSR_EW                (1 << 6)  /* Bit 6:  Warning Status */
#define MCAN_PSR_BO                (1 << 7)  /* Bit 7:  Bus_Off Status */
#define MCAN_PSR_FLEC_SHIFT        (8)       /* Bits 8-10: Fast Last Error Code */
#define MCAN_PSR_FLEC_MASK         (7 << MCAN_PSR_FLEC_SHIFT)
#  define MCAN_PSR_FLEC(n)         ((uint32_t)(n) << MCAN_PSR_FLEC_SHIFT) /* See error codes above */
#define MCAN_PSR_RESI              (1 << 11) /* Bit 11: ESI Flag of Last Received CAN FD Message */
#define MCAN_PSR_RBRS              (1 << 12) /* Bit 12: BRS Flag of Last Received CAN FD Message */
#define MCAN_PSR_REDL              (1 << 13) /* Bit 13: Received a CAN FD Message */

/* Common bit definitions for Interrupt Register, Interrupt Enable Register, Interrupt
 * Line Select Register
 */ 

#define MCAN_INT_RF0N              (1 << 0)  /* Bit 0:  Receive FIFO 0 New Message */
#define MCAN_INT_RF0W              (1 << 1)  /* Bit 1:  Receive FIFO 0 Watermark Reached */
#define MCAN_INT_RF0F              (1 << 2)  /* Bit 2:  Receive FIFO 0 Full */
#define MCAN_INT_RF0L              (1 << 3)  /* Bit 3:  Receive FIFO 0 Message Lost */
#define MCAN_INT_RF1N              (1 << 4)  /* Bit 4:  Receive FIFO 1 New Message */
#define MCAN_INT_RF1W              (1 << 5)  /* Bit 5:  Receive FIFO 1 Watermark Reached */
#define MCAN_INT_RF1F              (1 << 6)  /* Bit 6:  Receive FIFO 1 Full */
#define MCAN_INT_RF1L              (1 << 7)  /* Bit 7:  Receive FIFO 1 Message Lost */
#define MCAN_INT_HPM               (1 << 8)  /* Bit 8:  High Priority Message */
#define MCAN_INT_TC                (1 << 9)  /* Bit 9:  Transmission Completed */
#define MCAN_INT_TCF               (1 << 10) /* Bit 10: Transmission Cancellation Finished */
#define MCAN_INT_TFE               (1 << 11) /* Bit 11: Tx FIFO Empty */
#define MCAN_INT_TEFN              (1 << 12) /* Bit 12: Tx Event FIFO New Entry */
#define MCAN_INT_TEFW              (1 << 13) /* Bit 13: Tx Event FIFO Watermark Reached */
#define MCAN_INT_TEFF              (1 << 14) /* Bit 14: Tx Event FIFO Full */
#define MCAN_INT_TEFL              (1 << 15) /* Bit 15: Tx Event FIFO Element Lost */
#define MCAN_INT_TSW               (1 << 16) /* Bit 16: Timestamp Wraparound */
#define MCAN_INT_MRAF              (1 << 17) /* Bit 17: Message RAM Access Failure */
#define MCAN_INT_TOO               (1 << 18) /* Bit 18: Timeout Occurred */
#define MCAN_INT_DRX               (1 << 19) /* Bit 19: Message stored to Dedicated Receive Buffer */
#define MCAN_INT_ELO               (1 << 22) /* Bit 22: Error Logging Overflow */
#define MCAN_INT_EP                (1 << 23) /* Bit 23: Error Passive */
#define MCAN_INT_EW                (1 << 24) /* Bit 24: Warning Status */
#define MCAN_INT_BO                (1 << 25) /* Bit 25: Bus_Off Status */
#define MCAN_INT_WDI               (1 << 26) /* Bit 26: Watchdog Interrupt */
#define MCAN_INT_CRCE              (1 << 27) /* Bit 27: CRC Error */
#define MCAN_INT_BE                (1 << 28) /* Bit 28: Bit Error */
#define MCAN_INT_ACKE              (1 << 29) /* Bit 29: Acknowledge Error */
#define MCAN_INT_FOE               (1 << 30) /* Bit 30: Format Error */
#define MCAN_INT_STE               (1 << 31) /* Bit 31: Stuff Error */

/* Interrupt Line Enable Register */

#define MCAN_ILE_EINT0             (1 << 0)  /* Bit 0:  Enable Interrupt Line 0 */
#define MCAN_ILE_EINT1             (1 << 1)  /* Bit 1:  Enable Interrupt Line 1 */

/* Global Filter Configuration Register */

#define MCAN_GFC_RRFE              (1 << 0)  /* Bit 0:  Reject Remote Frames Extended */
#define MCAN_GFC_RRFS              (1 << 1)  /* Bit 1:  Reject Remote Frames Standard */
#define MCAN_GFC_ANFE_SHIFT        (2)       /* Bits 2-3: Accept Non-matching Frames Extended */
#define MCAN_GFC_ANFE_MASK         (3 << MCAN_GFC_ANFE_SHIFT)
#  define MCAN_GFC_ANFE_RX_FIFO0   (0 << MCAN_GFC_ANFE_SHIFT) /* Message stored in Receive FIFO 0  */
#  define MCAN_GFC_ANFE_RX_FIFO1   (1 << MCAN_GFC_ANFE_SHIFT) /* Message stored in Receive FIFO 1 */
#  define MCAN_GFC_ANFE_REJECTED   (2 << MCAN_GFC_ANFE_SHIFT) /* 2-3 Message rejected */
#define MCAN_GFC_ANFS_SHIFT        (4)       /* Bits 4-5: Accept Non-matching Frames Standard */
#define MCAN_GFC_ANFS_MASK         (3 << MCAN_GFC_ANFS_SHIFT)
#  define MCAN_GFC_ANFS_RX_FIFO0   (0 << MCAN_GFC_ANFS_SHIFT) /* Message stored in Receive FIFO 0  */
#  define MCAN_GFC_ANFS_RX_FIFO1   (1 << MCAN_GFC_ANFS_SHIFT) /* Message stored in Receive FIFO 1 */
#  define MCAN_GFC_ANFS_REJECTED   (2 << MCAN_GFC_ANFS_SHIFT) /* 2-3 Message rejected */

/* Standard ID Filter Configuration Register */

#define MCAN_SIDFC_FLSSA_SHIFT     (2)       /* Bits 2-15: Filter List Standard Start Address */
#define MCAN_SIDFC_FLSSA_MASK      (0x3fff << MCAN_SIDFC_FLSSA_SHIFT)
#  define MCAN_SIDFC_FLSSA(n)      ((uint32_t)(n) << MCAN_SIDFC_FLSSA_SHIFT)
#define MCAN_SIDFC_LSS_SHIFT       (16)      /* Bits 16-23: List Size Standard */
#define MCAN_SIDFC_LSS_MASK        (0xff << MCAN_SIDFC_LSS_SHIFT)
#  define MCAN_SIDFC_LSS(n)        ((uint32_t)(n) << MCAN_SIDFC_LSS_SHIFT)

/* Extended ID Filter Configuration Register */

#define MCAN_XIDFC_FLESA_SHIFT     (2)       /* Bits 2-15: Filter List Extended Start Address */
#define MCAN_XIDFC_FLESA_MASK      (0x3fff << MCAN_XIDFC_FLESA_SHIFT)
#  define MCAN_XIDFC_FLESA(n)      ((uint32_t)(n) << MCAN_XIDFC_FLESA_SHIFT)
#define MCAN_XIDFC_LSE_SHIFT       (16)      /* Bits 16-22: List Size Extended */
#define MCAN_XIDFC_LSE_MASK        (0x7f << MCAN_XIDFC_LSE_SHIFT)
#  define MCAN_XIDFC_LSE(n)        ((uint32_t)(n) << MCAN_XIDFC_LSE_SHIFT)

/* Extended ID AND Mask Register */

#define MCAN_XIDAM_MASK            0x1fffffff /* Extended ID mask */

/* High Priority Message Status Register */

#define MCAN_HPMS_BIDX_SHIFT       (0)       /* Bits 0-5: Buffer Index */
#define MCAN_HPMS_BIDX_MASK        (0x3f << MCAN_HPMS_BIDX_SHIFT)
#  define MCAN_HPMS_BIDX(n)        ((uint32_t)(n) << MCAN_HPMS_BIDX_SHIFT)
#define MCAN_HPMS_MSI_SHIFT        (6)       /* Bits 6-7: Message Storage Indicator */
#define MCAN_HPMS_MSI_MASK         (3 << MCAN_HPMS_MSI_SHIFT)
#  define MCAN_HPMS_MSI_ NOFIFO    (0 << MCAN_HPMS_MSI_SHIFT) /* No FIFO selected. */
#  define MCAN_HPMS_MSI_ LOST      (1 << MCAN_HPMS_MSI_SHIFT) /* FIFO message. */
#  define MCAN_HPMS_MSI_ FIFO0     (2 << MCAN_HPMS_MSI_SHIFT) /* Message stored in FIFO 0. */
#  define MCAN_HPMS_MSI_ FIFO1     (3 << MCAN_HPMS_MSI_SHIFT) /* Message stored in FIFO 1. */
#define MCAN_HPMS_FIDX_SHIFT       (8)       /* Bits 8-14: Filter Index */
#define MCAN_HPMS_FIDX_MASK        (0x7f << MCAN_HPMS_FIDX_SHIFT)
#  define MCAN_HPMS_FIDX(n)        ((uint32_t)(n) << MCAN_HPMS_FIDX_SHIFT)
#define MCAN_HPMS_FLST             (1 << 15) /* Bit 15: Filter List */

/* New Data 1 Register */

#define MCAN_NDAT1(n)              (1 << (n)) /* New data for buffer n, n=0-31 */

/* New Data 2 Register */

#define MCAN_NDAT2(n)              (1 << ((n)-32)) /* New data for buffer n, n=32-63 */

/* Receive FIFO 0 Configuration Register */

#define MCAN_RXF0C_F0SA_SHIFT      (2)       /* Bits 2-15: Receive FIFO 0 Start Address */
#define MCAN_RXF0C_F0SA_MASK       (0x3fff << MCAN_RXF0C_F0SA_SHIFT)
#  define MCAN_RXF0C_F0SA(n)       ((uint32_t)(n) << MCAN_RXF0C_F0SA_SHIFT)
#define MCAN_RXF0C_F0S_SHIFT       (16)      /* Bits 16-22: Receive FIFO 0 Size */
#define MCAN_RXF0C_F0S_MASK        (0x7f << MCAN_RXF0C_F0S_SHIFT)
#  define MCAN_RXF0C_F0S(n)        ((uint32_t)(n) << MCAN_RXF0C_F0S_SHIFT)
#define MCAN_RXF0C_F0WM_SHIFT      (24)      /* Bits 24-30: Receive FIFO 0 Watermark */
#define MCAN_RXF0C_F0WM_MASK       (0x7f << MCAN_RXF0C_F0WM_SHIFT)
#  define MCAN_RXF0C_F0WM(n)       ((uint32_t)(n) << MCAN_RXF0C_F0WM_SHIFT)
#define MCAN_RXF0C_F0OM            (1 << 31) /* Bit 31: FIFO 0 Operation Mode */

/* Receive FIFO 0 Status Register */

#define MCAN_RXF0S_F0FL_SHIFT      (0)       /* Bits 0-6: Receive FIFO 0 Fill Level */
#define MCAN_RXF0S_F0FL_MASK       (0x7f << MCAN_RXF0S_F0FL_SHIFT)
#  define MCAN_RXF0S_F0FL(n)       ((uint32_t)(n) << MCAN_RXF0S_F0FL_SHIFT)
#define MCAN_RXF0S_F0GI_SHIFT      (8)       /* Bits 8-13: Receive FIFO 0 Get Index */
#define MCAN_RXF0S_F0GI_MASK       (0x3f << MCAN_RXF0S_F0GI_SHIFT)
#  define MCAN_RXF0S_F0GI(n)       ((uint32_t)(n) << MCAN_RXF0S_F0GI_SHIFT)
#define MCAN_RXF0S_F0PI_SHIFT      (16)      /* Bits 16-21: Receive FIFO 0 Put Index */
#define MCAN_RXF0S_F0PI_MASK       (0x3f << MCAN_RXF0S_F0PI_SHIFT)
#  define MCAN_RXF0S_F0PI(n)       ((uint32_t)(n) << MCAN_RXF0S_F0PI_SHIFT)
#define MCAN_RXF0S_F0F             (1 << 24) /* Bit 24: Receive FIFO 0 Full */
#define MCAN_RXF0S_RF0L            (1 << 25) /* Bit 25: Receive FIFO 0 Message Lost */

/* Receive FIFO 0 Acknowledge Register */

#define MCAN_RXF0A_MASK            0x0000003f /* Receive FIFO 0 acknowledge index mask */

/* Receive Rx Buffer Configuration Register */

#define MCAN_RXBC_MASK             0x0000fffc /* Receive buffer start address mask */

/* Receive FIFO 1 Configuration Register */

#define MCAN_RXF1C_F1SA_SHIFT      (2)       /* Bits 2-15: Receive FIFO 1 Start Address */
#define MCAN_RXF1C_F1SA_MASK       (0x3fff << MCAN_RXF1C_F1SA_SHIFT)
#  define MCAN_RXF1C_F1SA(n)       ((uint32_t)(n) << MCAN_RXF1C_F1SA_SHIFT)
#define MCAN_RXF1C_F1S_SHIFT       (16)      /* Bits 16-22: Receive FIFO 1 Size */
#define MCAN_RXF1C_F1S_MASK        (0x7f << MCAN_RXF1C_F1S_SHIFT)
#  define MCAN_RXF1C_F1S(n)        ((uint32_t)(n) << MCAN_RXF1C_F1S_SHIFT)
#define MCAN_RXF1C_F1WM_SHIFT      (24)      /* Bits 24-30: Receive FIFO 1 Watermark */
#define MCAN_RXF1C_F1WM_MASK       (0x7f << MCAN_RXF1C_F1WM_SHIFT)
#  define MCAN_RXF1C_F1WM(n)       ((uint32_t)(n) << MCAN_RXF1C_F1WM_SHIFT)
#define MCAN_RXF1C_F1OM            (1 << 31) /* Bit 31: FIFO 1 Operation Mode */

/* Receive FIFO 1 Status Register */

#define MCAN_RXF1S_F1FL_SHIFT      (0)       /* Bits 0-6: Receive FIFO 1 Fill Level */
#define MCAN_RXF1S_F1FL_MASK       (0x7f << MCAN_RXF1S_F1FL_SHIFT)
#  define MCAN_RXF1S_F1FL(n)       ((uint32_t)(n) << MCAN_RXF1S_F1FL_SHIFT)
#define MCAN_RXF1S_F1GI_SHIFT      (8)       /* Bits 8-13: Receive FIFO 1 Get Index */
#define MCAN_RXF1S_F1GI_MASK       (0x3f << MCAN_RXF1S_F1GI_SHIFT)
#  define MCAN_RXF1S_F1GI(n)       ((uint32_t)(n) << MCAN_RXF1S_F1GI_SHIFT)
#define MCAN_RXF1S_F1PI_SHIFT      (16)      /* Bits 16-21: Receive FIFO 1 Put Index */
#define MCAN_RXF1S_F1PI_MASK       (0x3f << MCAN_RXF1S_F1PI_SHIFT)
#  define MCAN_RXF1S_F1PI(n)       ((uint32_t)(n) << MCAN_RXF1S_F1PI_SHIFT)
#define MCAN_RXF1S_F1F             (1 << 24) /* Bit 24: Receive FIFO 1 Full */
#define MCAN_RXF1S_RF1L            (1 << 25) /* Bit 25: Receive FIFO 1 Message Lost */
#define MCAN_RXF1S_DMS_SHIFT       (30)      /* Bits 30-31: Debug Message Status */
#define MCAN_RXF1S_DMS_MASK        (3 << MCAN_RXF1S_DMS_SHIFT)
#  define MCAN_RXF1S_DMS_IDLE      (0 << MCAN_RXF1S_DMS_SHIFT) /* Idle state */
#  define MCAN_RXF1S_DMS_MSG_A     (1 << MCAN_RXF1S_DMS_SHIFT) /* Debug message A received. */
#  define MCAN_RXF1S_DMS_MSG_AB    (2 << MCAN_RXF1S_DMS_SHIFT) /* Debug messages A, B received. */
#  define MCAN_RXF1S_DMS_MSG_ABC   (3 << MCAN_RXF1S_DMS_SHIFT) /* Debug messages A, B, C received, DMA request is set. */

/* Receive FIFO 1 Acknowledge Register */

#define MCAN_RXF1A_MASK            0x0000003f /* Receive FIFO 1 acknowledge index mask */

/* Receive Buffer / FIFO Element Size Configuration Register */

#define MCAN_RXESC_F0DS_SHIFT      (0)       /* Bits 0-2: Receive FIFO 0 Data Field Size */
#define MCAN_RXESC_F0DS_MASK       (7 << MCAN_RXESC_F0DS_SHIFT)
#  define MCAN_RXESC_F0DS_8B       (0 << MCAN_RXESC_F0DS_SHIFT) /* 8-byte data field */
#  define MCAN_RXESC_F0DS_12B      (1 << MCAN_RXESC_F0DS_SHIFT) /* 12-byte data field */
#  define MCAN_RXESC_F0DS_16B      (2 << MCAN_RXESC_F0DS_SHIFT) /* 16-byte data field */
#  define MCAN_RXESC_F0DS_20B      (3 << MCAN_RXESC_F0DS_SHIFT) /* 20-byte data field */
#  define MCAN_RXESC_F0DS_24B      (4 << MCAN_RXESC_F0DS_SHIFT) /* 24-byte data field */
#  define MCAN_RXESC_F0DS_32B      (5 << MCAN_RXESC_F0DS_SHIFT) /* 32-byte data field */
#  define MCAN_RXESC_F0DS_48B      (6 << MCAN_RXESC_F0DS_SHIFT) /* 48-byte data field */
#  define MCAN_RXESC_F0DS_64B      (7 << MCAN_RXESC_F0DS_SHIFT) /* 64-byte data field */
#define MCAN_RXESC_F1DS_SHIFT      (4)       /* Bits 4-6: Receive FIFO 1 Data Field Size */
#define MCAN_RXESC_F1DS_MASK       (7 << MCAN_RXESC_F1DS_SHIFT)
#  define MCAN_RXESC_F1DS_8B       (0 << MCAN_RXESC_F1DS_SHIFT) /* 8-byte data field */
#  define MCAN_RXESC_F1DS_12B      (1 << MCAN_RXESC_F1DS_SHIFT) /* 12-byte data field */
#  define MCAN_RXESC_F1DS_16B      (2 << MCAN_RXESC_F1DS_SHIFT) /* 16-byte data field */
#  define MCAN_RXESC_F1DS_20B      (3 << MCAN_RXESC_F1DS_SHIFT) /* 20-byte data field */
#  define MCAN_RXESC_F1DS_24B      (4 << MCAN_RXESC_F1DS_SHIFT) /* 24-byte data field */
#  define MCAN_RXESC_F1DS_32B      (5 << MCAN_RXESC_F1DS_SHIFT) /* 32-byte data field */
#  define MCAN_RXESC_F1DS_48B      (6 << MCAN_RXESC_F1DS_SHIFT) /* 48-byte data field */
#  define MCAN_RXESC_F1DS_64B      (7 << MCAN_RXESC_F1DS_SHIFT) /* 64-byte data field */
#define MCAN_RXESC_RBDS_SHIFT      (8)       /* Bits 8-10: Receive Buffer Data Field Size */
#define MCAN_RXESC_RBDS_MASK       (7 << MCAN_RXESC_RBDS_SHIFT)
#  define MCAN_RXESC_RBDS_8B       (0 << MCAN_RXESC_RBDS_SHIFT) /* 8-byte data field */
#  define MCAN_RXESC_RBDS_12B      (1 << MCAN_RXESC_RBDS_SHIFT) /* 12-byte data field */
#  define MCAN_RXESC_RBDS_16B      (2 << MCAN_RXESC_RBDS_SHIFT) /* 16-byte data field */
#  define MCAN_RXESC_RBDS_20B      (3 << MCAN_RXESC_RBDS_SHIFT) /* 20-byte data field */
#  define MCAN_RXESC_RBDS_24B      (4 << MCAN_RXESC_RBDS_SHIFT) /* 24-byte data field */
#  define MCAN_RXESC_RBDS_32B      (5 << MCAN_RXESC_RBDS_SHIFT) /* 32-byte data field */
#  define MCAN_RXESC_RBDS_48B      (6 << MCAN_RXESC_RBDS_SHIFT) /* 48-byte data field */
#  define MCAN_RXESC_RBDS_64B      (7 << MCAN_RXESC_RBDS_SHIFT) /* 64-byte data field */

/* Transmit Buffer Configuration Register */

#define MCAN_TXBC_TBSA_SHIFT       (2)       /* Bits 2-15: Tx Buffers Start Address */
#define MCAN_TXBC_TBSA_MASK        (0x3fff << MCAN_TXBC_TBSA_SHIFT)
#  define MCAN_TXBC_TBSA(n)        ((uint32_t)(n) << MCAN_TXBC_TBSA_SHIFT)
#define MCAN_TXBC_NDTB_SHIFT       (16)      /* Bits 16-21: Number of Dedicated Transmit Buffers */
#define MCAN_TXBC_NDTB_MASK        (0x3f << MCAN_TXBC_NDTB_SHIFT)
#  define MCAN_TXBC_NDTB(n)        ((uint32_t)(n) << MCAN_TXBC_NDTB_SHIFT)
#define MCAN_TXBC_TFQS_SHIFT       (24)      /* Bits 24-29: Transmit FIFO/Queue Size */
#define MCAN_TXBC_TFQS_MASK        (0x3f << MCAN_TXBC_TFQS_SHIFT)
#  define MCAN_TXBC_TFQS(n)        ((uint32_t)(n) << MCAN_TXBC_TFQS_SHIFT)
#define MCAN_TXBC_TFQM             (1 << 30) /* Bit 30: Tx FIFO/Queue Mode */

/* Transmit FIFO/Queue Status Register */

#define MCAN_TXFQS_TFFL_SHIFT      (0)       /* Bits 0-5: Tx FIFO Free Level */
#define MCAN_TXFQS_TFFL_MASK       (0x3f << MCAN_TXFQS_TFFL_SHIFT)
#  define MCAN_TXFQS_TFFL(n)       ((uint32_t)(n) << MCAN_TXFQS_TFFL_SHIFT)
#define MCAN_TXFQS_TFGI_SHIFT      (8)       /* Bits 8-12: Tx FIFO Get Index */
#define MCAN_TXFQS_TFGI_MASK       (0x1f << MCAN_TXFQS_TFGI_SHIFT)
#  define MCAN_TXFQS_TFGI(n)       ((uint32_t)(n) << MCAN_TXFQS_TFGI_SHIFT)
#define MCAN_TXFQS_TFQPI_SHIFT     (16)      /* Bits 16-20: Tx FIFO/Queue Put Index */
#define MCAN_TXFQS_TFQPI_MASK      (0x1f << MCAN_TXFQS_TFQPI_SHIFT)
#  define MCAN_TXFQS_TFQPI(n)      ((uint32_t)(n) << MCAN_TXFQS_TFQPI_SHIFT)
#define MCAN_TXFQS_TFQF            (1 << 21) /* Bit 21: Tx FIFO/Queue Full */

/* Transmit Buffer Element Size Configuration Register */

#define MCAN_TXESC_TBDS_SHIFT      (0)       /* Bits 0-2: Tx Buffer Data Field Size */
#define MCAN_TXESC_TBDS_MASK       (7 << MCAN_TXESC_TBDS_SHIFT)
#  define MCAN_TXESC_TBDS_8B       (0 << MCAN_TXESC_TBDS_SHIFT) /* 8-byte data field */
#  define MCAN_TXESC_TBDS_12B      (1 << MCAN_TXESC_TBDS_SHIFT) /* 12-byte data field */
#  define MCAN_TXESC_TBDS_16B      (2 << MCAN_TXESC_TBDS_SHIFT) /* 16-byte data field */
#  define MCAN_TXESC_TBDS_20B      (3 << MCAN_TXESC_TBDS_SHIFT) /* 20-byte data field */
#  define MCAN_TXESC_TBDS_24B      (4 << MCAN_TXESC_TBDS_SHIFT) /* 24-byte data field */
#  define MCAN_TXESC_TBDS_32B      (5 << MCAN_TXESC_TBDS_SHIFT) /* 32-byte data field */
#  define MCAN_TXESC_TBDS_48B      (6 << MCAN_TXESC_TBDS_SHIFT) /* 48-byte data field */
#  define MCAN_TXESC_TBDS_64B      (7 << MCAN_TXESC_TBDS_SHIFT) /* 64-byte data field */

/* Transmit Buffer Request Pending Register */

#define MCAN_TXBRP(n)              (1 << (n)) /* Transmission request pending for buffer n, n=0-31 */

/* Transmit Buffer Add Request Register */

#define MCAN_TXBAR(n)              (1 << (n)) /* Add request for transmit buffer n, n=0-31 */

/* Transmit Buffer Cancellation Request Register */

#define MCAN_TXBCR(n)              (1 << (n)) /* Cancellation request for transmit buffer n, n=0-31 */

/* Transmit Buffer Transmission Occurred Register */

#define MCAN_TXBTO(n)              (1 << (n)) /* Transmission occurred for buffer n, n=0-31 */

/* Transmit Buffer Cancellation Finished Register */

#define MCAN_TXBCF(n)              (1 << (n)) /* Cancellation finished for transmit buffer n, n=0-31 */

/* Transmit Buffer Transmission Interrupt Enable Register */

#define MCAN_TXBTIE(n)             (1 << (n)) /* Transmission interrupt enable for buffer n, n=0-31 */

/* Transmit Buffer Cancellation Finished Interrupt Enable Register */

#define MCAN_TXBTIE(n)             (1 << (n)) /* Cancellation finished interrupt enable for transmit buffer n, n=0-31 */

/* Transmit Event FIFO Configuration Register */

#define MCAN_TXEFC_EFSA_SHIFT      (2)       /* Bits 2-15: Event FIFO Start Address */
#define MCAN_TXEFC_EFSA_MASK       (0x3fff << MCAN_TXEFC_EFSA_SHIFT)
#  define MCAN_TXEFC_EFSA(n)       ((uint32_t)(n) << MCAN_TXEFC_EFSA_SHIFT)
#define MCAN_TXEFC_EFS_SHIFT       (16)      /* Bits 16-21: Event FIFO Size */
#define MCAN_TXEFC_EFS_MASK        (0x3f << MCAN_TXEFC_EFS_SHIFT)
#  define MCAN_TXEFC_EFS(n)        ((uint32_t)(n) << MCAN_TXEFC_EFS_SHIFT)
#define MCAN_TXEFC_EFWM_SHIFT      (24)      /* Bits 24-29: Event FIFO Watermark */
#define MCAN_TXEFC_EFWM_MASK       (0x3f << MCAN_TXEFC_EFWM_SHIFT)
#  define MCAN_TXEFC_EFWM(n)       ((uint32_t)(n) << MCAN_TXEFC_EFWM_SHIFT)

/* Transmit Event FIFO Status Register */

#define MCAN_TXEFS_EFFL_SHIFT      (0)       /* Bits 0-5: Event FIFO Fill Level */
#define MCAN_TXEFS_EFFL_MASK       (0x3f << MCAN_TXEFS_EFFL_SHIFT)
#  define MCAN_TXEFS_EFFL(n)       ((uint32_t)(n) << MCAN_TXEFS_EFFL_SHIFT)
#define MCAN_TXEFS_EFGI_SHIFT      (8)       /* Bits 8-12: Event FIFO Get Index */
#define MCAN_TXEFS_EFGI_MASK       (0x1f << MCAN_TXEFS_EFGI_SHIFT)
#  define MCAN_TXEFS_EFGI(n)       ((uint32_t)(n) << MCAN_TXEFS_EFGI_SHIFT)
#define MCAN_TXEFS_EFPI_SHIFT      (16)      /* Bits 16-20: Event FIFO Put Index */
#define MCAN_TXEFS_EFPI_MASK       (0x1f << MCAN_TXEFS_EFPI_SHIFT)
#  define MCAN_TXEFS_EFPI(n)       ((uint32_t)(n) << MCAN_TXEFS_EFPI_SHIFT)
#define MCAN_TXEFS_EFF             (1 << 24) /* Bit 24: Event FIFO Full */
#define MCAN_TXEFS_TEFL            (1 << 25) /* Bit 25: Tx Event FIFO Element Lost */

/* Transmit Event FIFO Acknowledge Register */

#define MCAN_TXEFA_MASK            0x0000001f /* Event fifo acknowledge index mask */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_CHIP_SAM_MCAN_H */
