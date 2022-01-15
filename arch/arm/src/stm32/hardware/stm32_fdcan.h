/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_fdcan.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_FDCAN_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_FDCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only for STM32G4 */

#ifndef CONFIG_STM32_STM32G4XXX
#  error STM32 FDCAN was tested only for STM32G4
#endif

/* Register Offsets *********************************************************/

#define STM32_FDCAN_CREL_OFFSET    0x0000  /* FDCAN core release register */
#define STM32_FDCAN_ENDN_OFFSET    0x0004  /* FDCAN endian register */
                                           /* 0x0008 Reserved */
#define STM32_FDCAN_DBTP_OFFSET    0x000c  /* FDCAN data bit timing and prescaler register */
#define STM32_FDCAN_TEST_OFFSET    0x0010  /* FDCAN test register */
#define STM32_FDCAN_RWD_OFFSET     0x0014  /* FDCAN RAM watchdog register */
#define STM32_FDCAN_CCCR_OFFSET    0x0018  /* FDCAN CC control register */
#define STM32_FDCAN_NBTP_OFFSET    0x001c  /* FDCAN nominal bit timing and prescaler register */
#define STM32_FDCAN_TSCC_OFFSET    0x0020  /* FDCAN timestamp counter configuration register */
#define STM32_FDCAN_TSCV_OFFSET    0x0024  /* FDCAN timestamp counter value register */
#define STM32_FDCAN_TOCC_OFFSET    0x0028  /* FDCAN timeout counter configuration register */
#define STM32_FDCAN_TOCV_OFFSET    0x002c  /* FDCAN timeout counter value register */
                                           /* 0x0030 to 0x003c Reserved */
#define STM32_FDCAN_ECR_OFFSET     0x0040  /* FDCAN error counter register */
#define STM32_FDCAN_PSR_OFFSET     0x0044  /* FDCAN protocol status register */
#define STM32_FDCAN_TDCR_OFFSET    0x0048  /* FDCAN transmitter delay compensation register */
                                           /* 0x004c Reserved */
#define STM32_FDCAN_IR_OFFSET      0x0050  /* FDCAN interrupt register */
#define STM32_FDCAN_IE_OFFSET      0x0054  /* FDCAN interrupt enable register */
#define STM32_FDCAN_ILS_OFFSET     0x0058  /* FDCAN interrupt line select register */
#define STM32_FDCAN_ILE_OFFSET     0x005c  /* FDCAN interrupt line enable register */
                                           /* 0x0060 to 0x007c Reserved */
#define STM32_FDCAN_RXGFC_OFFSET   0x0080  /* FDCAN global filter configuration register */
#define STM32_FDCAN_XIDAM_OFFSET   0x0084  /* FDCAN extended ID and mask register */
#define STM32_FDCAN_HPMS_OFFSET    0x0088  /* FDCAN high-priority message status register */
#define STM32_FDCAN_RXFS_OFFSET(f) (0x0090 + ((f) << 3)
#define STM32_FDCAN_RXFA_OFFSET(f) (0x0094 + ((f) << 3)
#define STM32_FDCAN_RXF0S_OFFSET   0x0090  /* FDCAN Rx FIFO 0 status register */
#define STM32_FDCAN_RXF0A_OFFSET   0x0094  /* CAN Rx FIFO 0 acknowledge register */
#define STM32_FDCAN_RXF1S_OFFSET   0x0098  /* FDCAN Rx FIFO 1 status register */
#define STM32_FDCAN_RXF1A_OFFSET   0x009c  /* FDCAN Rx FIFO 1 acknowledge register */
                                           /* 0x00a0 to 0x00bc Reserved */
#define STM32_FDCAN_TXBC_OFFSET    0x00c0  /* FDCAN Tx buffer configuration register */
#define STM32_FDCAN_TXFQS_OFFSET   0x00c4  /* FDCAN Tx FIFO/queue status register */
#define STM32_FDCAN_TXBRP_OFFSET   0x00c8  /* FDCAN Tx buffer request pending register */
#define STM32_FDCAN_TXBAR_OFFSET   0x00cc  /* FDCAN Tx buffer add request register */
#define STM32_FDCAN_TXBCR_OFFSET   0x00d0  /* FDCAN Tx buffer cancellation request register */
#define STM32_FDCAN_TXBTO_OFFSET   0x00d4  /* FDCAN Tx buffer transmission occurred register */
#define STM32_FDCAN_TXBCNF_OFFSET  0x00d8  /* FDCAN Tx buffer cancellation finished register */
#define STM32_FDCAN_TXBTIE_OFFSET  0x00dc  /* FDCAN Tx buffer transmission interrupt enable register */
#define STM32_FDCAN_TXBCIE_OFFSET  0x00e0  /* FDCAN Tx buffer cancellation finished interrupt enable register */
#define STM32_FDCAN_TXEFS_OFFSET   0x00e4  /* FDCAN Tx event FIFO status register */
#define STM32_FDCAN_TXEFA_OFFSET   0x00e8  /* FDCAN Tx event FIFO acknowledge register */
#define STM32_FDCAN_CKDIV_OFFSET   0x0100  /* FDCAN CFG clock divider register */

/* Register Bitfield Definitions ********************************************/

/* FDCAN core release register */

#define FDCAN_CREL_DAY_SHIFT       (0)                                     /* Bits 0-7: DAY */
#define FDCAN_CREL_DAY_MASK        (0xff << FDCAN_CREL_DAY_SHIFT)
#define FDCAN_CREL_MON_SHIFT       (8)                                     /* Bits 8-15: MON */
#define FDCAN_CREL_MON_MASK        (0xff << FDCAN_CREL_MON_SHIFT)
#define FDCAN_CREL_YEAR_SHIFT      (16)                                    /* Bits 8-15: YEAR */
#define FDCAN_CREL_YEAR_MASK       (0x0f << FDCAN_CREL_YEAR_SHIFT)
#define FDCAN_CREL_SUBSTEP_SHIFT   (20)                                    /* Bits 20-23: SUBSTEP */
#define FDCAN_CREL_SUBSTEP_MASK    (0x0f << FDCAN_CREL_SUBSTEP_SHIFT)
#define FDCAN_CREL_STEP_SHIFT      (24)                                    /* Bits 24-27: STEP */
#define FDCAN_CREL_STEP_MASK       (0x0f << FDCAN_CREL_STEP_SHIFT)
#define FDCAN_CREL_REL_SHIFT       (28)                                    /* Bits 28-31: REL */
#define FDCAN_CREL_REL_MASK        (0x0f << FDCAN_CREL_REL_SHIFT)

/* FDCAN data bit timing and prescaler register */

#define FDCAN_DBTP_DSJW_SHIFT       (0)                                    /* Bits 0-3: Synchronization jump width */
#define FDCAN_DBTP_DSJW_MASK        (0x0f << FDCAN_DBTP_DSJW_SHIFT)
#  define FDCAN_DBTP_DSJW(value)    ((value) << FDCAN_DBTP_DSJW_SHIFT)
#  define FDCAN_DBTP_DSJW_MAX       (15)
#define FDCAN_DBTP_DTSEG2_SHIFT     (4)                                    /* Bits 4-7: Data time segment after sample point*/
#define FDCAN_DBTP_DTSEG2_MASK      (0x0f << FDCAN_DBTP_DTSEG2_SHIFT)
#  define FDCAN_DBTP_DTSEG2(value)  ((value) << FDCAN_DBTP_DTSEG2_SHIFT)
#  define FDCAN_DBTP_DTSEG2_MAX     (15)
#define FDCAN_DBTP_DTSEG1_SHIFT     (8)                                    /* Bits 8-12: Data time segment before sample point*/
#define FDCAN_DBTP_DTSEG1_MASK      (0x1f << FDCAN_DBTP_DTSEG1_SHIFT)
#  define FDCAN_DBTP_DTSEG1(value)  ((value) << FDCAN_DBTP_DTSEG1_SHIFT)
#  define FDCAN_DBTP_DTSEG1_MAX     (31)
#define FDCAN_DBTP_DBRP_SHIFT       (16)                                   /* Bits 16-20: Data bitrate prescaler */
#define FDCAN_DBTP_DBRP_MASK        (0x1f << FDCAN_DBTP_DBRP_SHIFT)
#  define FDCAN_DBTP_DBRP(value)    ((value) << FDCAN_DBTP_DBRP_SHIFT)
#  define FDCAN_DBTP_DBRP_MAX       (31)
#define FDCAN_DBTP_TDC_EN           (1 << 23)                              /* Bit 23: Transceiver delay compensation enable */

/* FDCAN test register */

#define FDCAN_TEST_LBCK             (1 << 4)                               /* Bit 4: Loop back mode */
#define FDCAN_TEST_TX_SHIFT         (5)                                    /* Bits 5-6: Control of transmit pin */
#define FDCAN_TEST_TX_MASK          (0x3 << FDCAN_TEST_TX_SHIFT)
#  define FDCAN_TEST_TX_RESET       (0 << FDCAN_TEST_TX_SHIFT)             /* 00: TX is controlled by CAN core */
#  define FDCAN_TEST_TX_SP          (1 << FDCAN_TEST_TX_SHIFT)             /* 01: Sample point can be monitored at TX pin */
#  define FDCAN_TEST_TX_DLVL        (2 << FDCAN_TEST_TX_SHIFT)             /* 10: Dominant (0) level at TX pin */
#  define FDCAN_TEST_TX_RLVL        (3 << FDCAN_TEST_TX_SHIFT)             /* 11: Recesive (1) level at TX pin */
#define FDCAN_TEST_RX               (1 << 7)                               /* Bit 7: Receive pin */

/* FDCAN RAM watchdog register */

#define FDCAN_RWD_WDC_SHIFT         (0)                                    /* Bits 0-7: RAM watchdog counter start value */
#define FDCAN_RWD_WDC_MASK          (0xff << FDCAN_RWD_WDC_SHIFT)
#  define FDCAN_RWD_WDC_DIS         (0 << FDCAN_RWD_WDC_SHIFT)             /* Counter disabled */
#  define FDCAN_RWD_WDC(value)      ((value) << FDCAN_RWD_WDC_SHIFT)
#define FDCAN_RWD_WDV_SHIFT         (8)                                    /* Bits 8-15: RAM watchdog counter value */
#define FDCAN_RWD_WDV_MASK          (0xff << FDCAN_RWD_WDV_SHIFT)

/* FDCAN CC control register */

#define FDCAN_CCCR_INIT             (1 << 0)                               /* Bit 0: Initialization */
#define FDCAN_CCCR_CCE              (1 << 1)                               /* Bit 1: Configuration change enable */
#define FDCAN_CCCR_ASM              (1 << 2)                               /* Bit 2: ASM restricted operation mode */
#define FDCAN_CCCR_CSA              (1 << 3)                               /* Bit 3: Clock stop acknowledge */
#define FDCAN_CCCR_CSR              (1 << 4)                               /* Bit 4: Clock stop request */
#define FDCAN_CCCR_MON              (1 << 5)                               /* Bit 5: Bus monitoring mode */
#define FDCAN_CCCR_DAR              (1 << 6)                               /* Bit 6: Disable automatic retransmission */
#define FDCAN_CCCR_TEST             (1 << 7)                               /* Bit 7: Test mode enable */
#define FDCAN_CCCR_FDOE             (1 << 8)                               /* Bit 8: FD operation enable */
#define FDCAN_CCCR_BRSE             (1 << 9)                               /* Bit 9: FDCAN Bitrate switching */
                                                                           /* Bits 10-11: Reserved */
#define FDCAN_CCCR_PXHD             (1 << 12)                              /* Bit 12: Protocol exception handling disable */
#define FDCAN_CCCR_EFBI             (1 << 13)                              /* Bit 13: Edge filtering during bus integration */
#define FDCAN_CCCR_TXP              (1 << 14)                              /* Bit 14: Tx pause */
#define FDCAN_CCCR_NISO             (1 << 15)                              /* Bit 15: Non ISO operation */

/* FDCAN nominal bit timing and prescaler register */

#define FDCAN_NBTP_NTSEG2_SHIFT     (0)                                    /* Bits 0-6: Nominal time segment after sample point */
#define FDCAN_NBTP_NTSEG2_MASK      (0x7f << FDCAN_NBTP_NTSEG2_SHIFT)
#  define FDCAN_NBTP_NTSEG2(value)  ((value) << FDCAN_NBTP_NTSEG2_SHIFT)
#  define FDCAN_NBTP_NTSEG2_MAX     (127)
#define FDCAN_NBTP_NTSEG1_SHIFT     (8)                                    /* Bits 8-15: Nominal time segment before sample point */
#define FDCAN_NBTP_NTSEG1_MASK      (0xff << FDCAN_NBTP_NTSEG1_SHIFT)
#  define FDCAN_NBTP_NTSEG1(value)  ((value) << FDCAN_NBTP_NTSEG1_SHIFT)
#  define FDCAN_NBTP_NTSEG1_MAX     (255)
#define FDCAN_NBTP_NBRP_SHIFT       (16)                                   /* Bits 16-24: Bitrate prescaler */
#define FDCAN_NBTP_NBRP_MASK        (0x1ff << FDCAN_NBTP_NBRP_SHIFT)
#  define FDCAN_NBTP_NBRP(value)    ((value) << FDCAN_NBTP_NBRP_SHIFT)
#  define FDCAN_NBTP_NBRP_MAX       (511)
#define FDCAN_NBTP_NSJW_SHIFT       (25)                                   /* Bits 25-31: Nominal (re)synchronization jump width */
#define FDCAN_NBTP_NSJW_MASK        (0x7f << FDCAN_NBTP_NSJW_SHIFT)
#  define FDCAN_NBTP_NSJW(value)    ((value) << FDCAN_NBTP_NSJW_SHIFT)
#  define FDCAN_NBTP_NSJW_MAX       (127)

/* FDCAN timestamp counter configuration register */

#define FDCAN_TSCC_TSS_SHIFT        (0)                                    /* Bits 0-1: Timestamp counter select */
#define FDCAN_TSCC_TSS_MASK         (0x3 << FDCAN_TSCC_TSS_SHIFT)
#  define FDCAN_TSCC_TSS_ZERO       (0 << FDCAN_TSCC_TSS_SHIFT)            /* 00: Always 0 */
#  define FDCAN_TSCC_TSS_TCP        (1 << FDCAN_TSCC_TSS_SHIFT)            /* 01: Incremented based on TCP */
#  define FDCAN_TSCC_TSS_TIM3       (2 << FDCAN_TSCC_TSS_SHIFT)            /* 10: Value from TIM3 used */
#define FDCAN_TSCC_TCP_SHIFT        (16)                                   /* Bits 16-19: Timestamp counter prescaler */
#define FDCAN_TSCC_TCP_MASK         (0x0f << FDCAN_TSCC_TCP_SHIFT)
#  define FDCAN_TSCC_TCP(value)     ((value) << FDCAN_TSCC_TCP_SHIFT)

/* FDCAN timestamp counter value register */

#define FDCAN_TSCV_TSC_SHIFT        (0)                                    /* Bits 0-15: Timestamp counter */
#define FDCAN_TSCV_TSC_MASK         (0xffff << FDCAN_TSCV_TSC_SHIFT)

/* FDCAN timeout counter configuration register */

#define FDCAN_TOCC_ETOC             (1 << 0)                               /* Bit 0: Enable timeout counter */
#define FDCAN_TOCC_TOS_SHIFT        (1)                                    /* Bits 1-2: Timeout select */
#define FDCAN_TOCC_TOS_MASK         (0x03 << FDCAN_TOCC_TOS_SHIFT)
#  define FDCAN_TOCC_TOS_CONT       (0 << FDCAN_TOCC_TOS_SHIFT)            /* 00: Continuous operation */
#  define FDCAN_TOCC_TOS_TXFIFO     (1 << FDCAN_TOCC_TOS_SHIFT)            /* 01: Tx event FIFO */
#  define FDCAN_TOCC_TOS_RX_FIFO0   (2 << FDCAN_TOCC_TOS_SHIFT)            /* 10: Rx FIFO 0 */
#  define FDCAN_TOCC_TOS_RX_FIFO1   (3 << FDCAN_TOCC_TOS_SHIFT)            /* 11: Rx FIFO 1 */
#define FDCAN_TOCC_TOP_SHIFT        (16)                                   /* Bits 16-31: Timeout period counter start value */
#define FDCAN_TOCC_TOP_MASK         (0xffff << FDCAN_TOCC_TOP_SHIFT)
#  define FDCAN_TOCC_TOP(value)     ((value) << FDCAN_TOCC_TOP_SHIFT)

/* FDCAN timeout counter value register */

#define FDCAN_TOCV_TOC_SHIFT        (0)                                    /* Bits 0-15: Timestamp counter */
#define FDCAN_TOCV_TOC_MASK         (0xffff << FDCAN_TOCV_TOC_SHIFT)

/* FDCAN error counter register */

#define FDCAN_ECR_TEC_SHIFT         (0)                                    /* Bits 0-7: Transmit error counter */
#define FDCAN_CR_TEC_MASK           (0xff << FDCAN_ECR_TEC_SHIFT)
#define FDCAN_ECR_REC_SHIFT         (8)                                    /* Bits 8-14: Receive error counter */
#define FDCAN_ECR_REC_MASK          (0x7f << FDCAN_ECR_REC_SHIFT)
#define FDCAN_ECR_RP                (1 << 15)                              /* Bit 15: Receive error passive */
#define FDCAN_ECR_CEL_SHIFT         (16)                                   /* Bits 16-23: CAN error logging */
#define FDCAN_ECR_CEL_MASK          (0xff << FDCAN_ECR_CEL_SHIFT)

/* FDCAN protocol status register */

/* Error codes */

#define FDCAN_PSR_EC_NO_ERROR       (0)                                    /* No error occurred since LEC has been reset */
#define FDCAN_PSR_EC_STUFF_ERROR    (1)                                    /* More than 5 equal bits in a sequence */
#define FDCAN_PSR_EC_FORM_ERROR     (2)                                    /* Part of a received frame has wrong format */
#define FDCAN_PSR_EC_ACK_ERROR      (3)                                    /* Message not acknowledged by another node */
#define FDCAN_PSR_EC_BIT1_ERROR     (4)                                    /* Send with recessive level, but bus value was dominant */
#define FDCAN_PSR_EC_BIT0_ERROR     (5)                                    /* Send with dominant level, but bus value was recessive */
#define FDCAN_PSR_EC_CRC_ERROR      (6)                                    /* CRC received message incorrect */
#define FDCAN_PSR_EC_NO_CHANGE      (7)                                    /* No CAN bus event was detected since last read */

#define FDCAN_PSR_LEC_SHIFT         (0)                                    /* Bits 0-2: Last error code */
#define FDCAN_PSR_LEC_MASK          (0x7 << FDCAN_PSR_LEC_SHIFT)
#  define FDCAN_PSR_LEC(n)          ((uint32_t)(n) << FDCAN_PSR_LEC_SHIFT) /* See error codes above */
#define FDCAN_PSR_ACT_SHIFT         (3)                                    /* Bits 3-4: Activity */
#define FDCAN_PSR_ACT_MASK          (3 << FDCAN_PSR_ACT_SHIFT)
#  define FDCAN_PSR_ACT_SYNC        (0 << FDCAN_PSR_ACT_SHIFT)             /* 00: Synchronizing */
#  define FDCAN_PSR_ACT_IDLE        (1 << FDCAN_PSR_ACT_SHIFT)             /* 01: Idle */
#  define FDCAN_PSR_ACT_RECV        (2 << FDCAN_PSR_ACT_SHIFT)             /* 10: Receiver */
#  define FDCAN_PSR_ACT_TRANS       (3 << FDCAN_PSR_ACT_SHIFT)             /* 11: Transmitter */
#define FDCAN_PSR_EP                (1 << 5)                               /* Bit 5: Error passive */
#define FDCAN_PSR_EW                (1 << 6)                               /* Bit 6: Warning status */
#define FDCAN_PSR_BO                (1 << 7)                               /* Bit 7: Bus_off status */
#define FDCAN_PSR_DLEC_SHIFT        (8)                                    /* Bits 8-10: Data last error code */
#define FDCAN_PSR_DLEC_MASK         (0x7 << FDCAN_PSR_DLEC_SHIFT)
#  define FDCAN_PSR_DLEC(n)        ((uint32_t)(n) << FDCAN_PSR_DLEC_SHIFT) /* See error codes above */
#define FDCAN_PSR_RESI              (1 << 11)                              /* Bit 11: ESI flag of last message */
#define FDCAN_PSR_RBRS              (1 << 12)                              /* Bit 12: BRS flag of last message */
#define FDCAN_PSR_REDL              (1 << 13)                              /* Bit 13: Received message */
#define FDCAN_PSR_PXE               (1 << 14)                              /* Bit 14: Protocol exception event */
#define FDCAN_PSR_TDCV_SHIFT        (16)                                   /* Bits 16-22: Transmitter delay compensation */
#define FDCAN_PSR_TDCV_MASK         (0x7f << FDCAN_PSR_TDCV_SHIFT)

/* FDCAN transmitter delay compensation register */

#define FDCAN_TDCR_TDCF_SHIFT       (0)                                    /* Bits 0-6: Transmitter delay compensation filter window length */
#define FDCAN_TDCR_TDCF_MASK        (0x7f << FDCAN_TDCR_TDCF_SHIFT)
#  define FDCAN_TDCR_TDCF(value)    ((value) << FDCAN_TDCR_TDCF_SHIFT)
#define FDCAN_TDCR_TDCO_SHIFT       (8)                                    /* Bits 8-14: Transmiiter delay compensation offset */
#define FDCAN_TDCR_TDCO_MASK        (0x7f << FDCAN_TDCR_TDCO_SHIFT)
#  define FDCAN_TDCR_TDCO(value)    ((value) << FDCAN_TDCR_TDCO_SHIFT)

/* FDCAN interrupt register and interrupt enable register */

#define FDCAN_INT_RF0N               (1 << 0)                              /* Bit 0: Rx FIFO 0 new message */
#define FDCAN_INT_RF0F               (1 << 1)                              /* Bit 1: Rx FIFO 0 full */
#define FDCAN_INT_RF0L               (1 << 2)                              /* Bit 2: Rx FIFO 0 message lost */
#define FDCAN_INT_RF1N               (1 << 3)                              /* Bit 3: Rx FIFO 1 new message */
#define FDCAN_INT_RF1F               (1 << 4)                              /* Bit 4: Rx FIFO 1 full */
#define FDCAN_INT_RF1L               (1 << 5)                              /* Bit 5: Rx FIFO 1 message lost */
#define FDCAN_INT_HPM                (1 << 6)                              /* Bit 6: High priority message */
#define FDCAN_INT_TC                 (1 << 7)                              /* Bit 7: Transmission completed */
#define FDCAN_INT_TCF                (1 << 8)                              /* Bit 8: Transmission cancellation finished */
#define FDCAN_INT_TFE                (1 << 9)                              /* Bit 9: Tx FIFO empty */
#define FDCAN_INT_TEFN               (1 << 10)                             /* Bit 10: Tx event FIFO new entry */
#define FDCAN_INT_TEFF               (1 << 11)                             /* Bit 11: Tx event FIFO full */
#define FDCAN_INT_TEFL               (1 << 12)                             /* Bit 12: Tx event FIFO element lost */
#define FDCAN_INT_TSW                (1 << 13)                             /* Bit 13: Timestamp wraparound */
#define FDCAN_INT_MRAF               (1 << 14)                             /* Bit 14: Message RAM access failure */
#define FDCAN_INT_TOO                (1 << 15)                             /* Bit 15: Timeout occurred */
#define FDCAN_INT_ELO                (1 << 16)                             /* Bit 16: Error logging overflow */
#define FDCAN_INT_EP                 (1 << 17)                             /* Bit 17: Error_passive status */
#define FDCAN_INT_EW                 (1 << 18)                             /* Bit 18: Error_warning status */
#define FDCAN_INT_BO                 (1 << 19)                             /* Bit 19: Buss_off status */
#define FDCAN_INT_WDI                (1 << 20)                             /* Bit 20: Watchdog interrupt */
#define FDCAN_INT_PEA                (1 << 21)                             /* Bit 21: Protocol error arbitration phase */
#define FDCAN_INT_PED                (1 << 22)                             /* Bit 22: Protocol error data phase */
#define FDCAN_INT_ARA                (1 << 23)                             /* Bit 23: Access to reserved address */

/* FDCAN interrupt line select register */

#define FDCAN_ILS_RXFIFO0            (1 << 0)                              /* Bit 0: RXFIFO 0 */
#define FDCAN_ILS_RXFIFO1            (1 << 1)                              /* Bit 1: RXFIFO 1 */
#define FDCAN_ILS_SMG                (1 << 2)                              /* Bit 2: SMSG */
#define FDCAN_ILS_TFERR              (1 << 3)                              /* Bit 3: TFERR */
#define FDCAN_ILS_MISC               (1 << 4)                              /* Bit 4: MISC */
#define FDCAN_ILS_BERR               (1 << 5)                              /* Bit 5: BERR */
#define FDCAN_ILS_PERR               (1 << 6)                              /* Bit 6: PERR */

/* FDCAN interrupt line enable register */

#define FDCAN_ILE_EINT0              (1 << 0)                              /* Bit 0: Enable interrupt line 0 */
#define FDCAN_ILE_EINT1              (1 << 1)                              /* Bit 1: Enable interrupt line 1 */

/* FDCAN global filter configuration register */

#define FDCAN_RXGFC_RRFE              (1 << 0)                             /* Bit 0: Reject remote frames ext */
#define FDCAN_RXGFC_RRFS              (1 << 1)                             /* Bit 1: Reject remote frames std */
#define FDCAN_RXGFC_ANFE_SHIFT        (2)                                  /* Bits 2-3: Accept non-matching frames ext */
#define FDCAN_RXGFC_ANFE_MASK         (0x3 << FDCAN_RXGFC_ANFE_SHIFT)
#  define FDCAN_RXGFC_ANFE_RX_FIFO0   (0 << FDCAN_RXGFC_ANFE_SHIFT)        /* 00: Accept in Rx FIFO 0 */
#  define FDCAN_RXGFC_ANFE_RX_FIFO1   (1 << FDCAN_RXGFC_ANFE_SHIFT)        /* 01: Accept in Rx FIFO 1 */
#  define FDCAN_RXGFC_ANFE_REJECTED   (2 << FDCAN_RXGFC_ANFE_SHIFT)        /* 10: Reject */
#define FDCAN_RXGFC_ANFS_SHIFT        (4)                                  /* Bits 5-4: Accept non-matching frames std */
#define FDCAN_RXGFC_ANFS_MASK         (0x3 << FDCAN_RXGFC_ANFS_SHIFT)
#  define FDCAN_RXGFC_ANFS_RX_FIFO0   (0 << FDCAN_RXGFC_ANFS_SHIFT)        /* 00: Accept in Rx FIFO 0 */
#  define FDCAN_RXGFC_ANFS_RX_FIFO1   (1 << FDCAN_RXGFC_ANFS_SHIFT)        /* 01: Accept in Rx FIFO 1 */
#  define FDCAN_RXGFC_ANFS_REJECTED   (2 << FDCAN_RXGFC_ANFS_SHIFT)        /* 10: Reject */
#define FDCAN_RXGFC_F1OM              (1 << 8)                             /* Bit 8: FIFO 1 operation mode */
#define FDCAN_RXGFC_F0OM              (1 << 9)                             /* Bit 9: FIFO 0 operation mode */
#define FDCAN_RXGFC_LSS_SHIFT         (16)                                 /* Bits 16-20: List size std */
#define FDCAN_RXGFC_LSS_MASK          (0x1f << FDCAN_RXGFC_LSS_SHIFT)
#  define FDCAN_RXGFC_LSS(value)      ((value) << FDCAN_RXGFC_LSS_SHIFT)
#  define FDCAN_RXGFC_LSS_MAX         (28)
#define FDCAN_RXGFC_LSE_SHIFT         (24)                                 /* Bits 24-27: List size ext */
#define FDCAN_RXGFC_LSE_MASK          (0x1f << FDCAN_RXGFC_LSE_SHIFT)
#  define FDCAN_RXGFC_LSE(value)      ((value) << FDCAN_RXGFC_LSE_SHIFT)
#  define FDCAN_RXGFC_LSE_MAX         (8)

/* FDCAN extended ID and mask register */

#define FDCAN_XIDAM_EIDM_SHIFT      (0)                                    /* Bits 0-28: Extended ID mask */
#define FDCAN_XIDAM_EIDM_MASK       (0x1fffffff << FDCAN_XIDAM_EIDM_SHIFT)

/* FDCAN high-priority message status register */

#define FDCAN_HPMS_BIDX_SHIFT       (0)                                    /* Bits 0-2: Buffer index */
#define FDCAN_HPMS_BIDX_MASK        (0x7 << FDCAN_HPMS_BIDX_SHIFT)
#  define FDCAN_HPMS_BIDX(value)    ((value) << FDCAN_HPMS_BIDX_SHIFT)
#define FDCAN_HPMS_MSI_SHIFT        (6)                                    /* Bits 6-7: Message storage indicator */
#define FDCAN_HPMS_MSI_MASK         (0x3 << FDCAN_HPMS_MSI_SHIFT)
#  define FDCAN_HPMS_MSI(value)     ((value) << FDCAN_HPMS_MSI_SHIFT)
#define FDCAN_HPMS_FIDX_SHIFT       (8)                                    /* Bits 8-12: Filter index */
#define FDCAN_HPMS_FIDX_MASK        (0x1f << FDCAN_HPMS_FIDX_SHIFT)
#  define FDCAN_HPMS_FIDX(value)    ((value) << FDCAN_HPMS_FIDX_SHIFT)
#define FDCAN_HPMS_FLST             (1 << 15)                              /* Bit 15: Filter list */

/* FDCAN Rx FIFO x status register */

#define FDCAN_RXFS_FFL_SHIFT        (0)                                    /* Bits 0-3: FIFO fill level */
#define FDCAN_RXFS_FFL_MASK         (0xf << FDCAN_RXFS_FFL_SHIFT)
#  define FDCAN_RXFS_FFL(value)     ((value) << FDCAN_RXFS_FFL_SHIFT)
#define FDCAN_RXFS_FGI_SHIFT        (8)                                    /* Bits 8-9: FIFO get index */
#define FDCAN_RXFS_FGI_MASK         (0x3 << FDCAN_RXFS_FGI_SHIFT)
#  define FDCAN_RXFS_FGI(value)     ((value) << FDCAN_RXFS_FGI_SHIFT)
#define FDCAN_RXFS_FPI_SHIFT        (16)                                   /* Bits 16-17: FIFO put index */
#define FDCAN_RXFS_FPI_MASK         (0x3 << FDCAN_RXFS_FPI_SHIFT)
#  define FDCAN_RXFS_FPI(value)     ((value) << FDCAN_RXFS_FPI_SHIFT)
#define FDCAN_RXFS_FF               (1 << 24)                              /* Bit 24: FIFO full */
#define FDCAN_RXFS_RFL              (1 << 25)                              /* Bit 25: FIFO message lost */

/* FDCAN Rx FIFO x acknowledge register */

#define FDCAN_RXFA_FAI_SHIFT        (0)                                    /* Bits 0-2: FIFO 0 acknowledge index */
#define FDCAN_RXFA_FAI_MASK         (0x7 << FDCAN_RXFA_FAI_SHIFT)

/* FDCAN Tx buffer configuration register */

#define FDCAN_TXBC_TFQM             (1 << 24)                              /* Bit 24: FIFO/queue mode */

/* FDCAN Tx FIFO/queue status register */

#define FDCAN_TXFQS_TFFL_SHIFT      (0)                                    /* Bits 0-2: FIFO free level */
#define FDCAN_TXFQS_TFFL_MASK       (0x7 << FDCAN_TXFQS_TFFL_SHIFT)
#define FDCAN_TXFQS_TFGI_SHIFT      (8)                                    /* Bits 8-9: FIFO get index */
#define FDCAN_TXFQS_TFGI_MASK       (0x3 << FDCAN_TXFQS_TFGI_SHIFT)
#define FDCAN_TXFQS_TFQPI_SHIFT     (16)                                   /* Bits 20-16: FIFO/queue put index */
#define FDCAN_TXFQS_TFQPI_MASK      (0x3 << FDCAN_TXFQS_TFQPI_SHIFT)
#define FDCAN_TXFQS_TFQF            (1 << 21)                              /* Bit 21: FIFO/queue full */

/* FDCAN Tx buffer request pending register */

#define FDCAN_TXBRP_TRP_SHIFT       (0)                                    /* Bits 0-2: Transmission request pending */
#define FDCAN_TXBRP_TRP_MASK        (0x7 << FDCAN_TXBRP_TRP_SHIFT)
#  define FDCAN_TXBRP_TRP(value)    ((value) << FDCAN_TXBRP_TRP_SHIFT)

/* FDCAN Tx buffer add request register */

#define FDCAN_TXBAR_AR_SHIFT        (0)                                    /* Bits 0-2: Add request */
#define FDCAN_TXBAR_AR_MASK         (0x7 << FDCAN_TXBAR_AR_SHIFT)
#  define FDCAN_TXBAR_AR(value)     ((value) << FDCAN_TXBAR_AR_SHIFT)

/* FDCAN Tx buffer cancellation request register */

#define FDCAN_TXBCR_CR_SHIFT        (0)                                    /* Bits 0-2: Cancellation request */
#define FDCAN_TXBCR_CR_MASK         (0x7 << FDCAN_TXBCR_CR_SHIFT)
#  define FDCAN_TXBCR_CR(value)     ((value) << FDCAN_TXBCR_CR_SHIFT)

/* FDCAN Tx buffer transmission occurred register */

#define FDCAN_TXBTO_TO_SHIFT        (0)                                    /* Bits 0-2: Transmission occurred */
#define FDCAN_TXBTO_TO_MASK         (0x7 << FDCAN_TXBTO_TO_SHIFT)

/* FDCAN Tx buffer cancellation finished register */

#define FDCAN_TXBCF_CF_SHIFT        (0)                                    /* Bits 0-2: Cancellation finished */
#define FDCAN_TXBCF_CF_MASK         (0x7 << FDCAN_TXBCF_CF_SHIFT)

/* FDCAN Tx buffer transmission interrupt enable register */

#define FDCAN_TXBTIE_TIE_SHIFT      (0)                                    /* Bits 0-2: Transmission interrupt enable */
#define FDCAN_TXBTIE_TIE_MASK       (0x7 << FDCAN_TXBTIE_TIE_SHIFT)
#  define FDCAN_TXBTIE_TIE(value)   ((value) << FDCAN_TXBTIE_TIE_SHIFT)

/* FDCAN Tx buffer cancellation finished interrupt enable register */

#define FDCAN_TXBCIE_CFIE_SHIFT      (0)                                   /* Bits 0-2: Cancellation finished interrupt enable */
#define FDCAN_TXBCIE_CFIE_MASK       (0x7 << FDCAN_TXBCIE_CFIE_SHIFT)
#  define FDCAN_TXBCIE_CFIE(value)   ((value) << FDCAN_TXBCIE_CFIE_SHIFT)

/* FDCAN Tx event FIFO status register */

#define FDCAN_TXEFS_EFFL_SHIFT       (2)                                   /* Bits 0-2: Event FIFO fill level */
#define FDCAN_TXEFS_EFFL_MASK        (0x7 << FDCAN_TXEFC_EFFL_SHIFT)
#  define FDCAN_TXEFC_EFFL(value)    ((value) << FDCAN_TXEFC_EFFL_SHIFT)
#define FDCAN_TXEFS_EFGI_SHIFT       (8)                                   /* Bits 8-9: Event FIFO get index */
#define FDCAN_TXEFS_EFGI_MASK        (0x3 << FDCAN_TXEFS_EFGI_SHIFT)
#  define FDCAN_TXEFS_EFGI(value)    ((value) << FDCAN_TXEFS_EFGI_SHIFT)
#define FDCAN_TXEFS_EFPI_SHIFT       (16)                                  /* Bits 16-17: Event FIFO put index */
#define FDCAN_TXEFS_EFPI_MASK        (0x3 << FDCAN_TXEFS_EFPI_SHIFT)
#  define FDCAN_TXEFS_EFPI(value)    ((value) << FDCAN_TXEFS_EFPI_SHIFT)
#define FDCAN_TXEFS_EFF              (1 << 24)                             /* Bit 24: Event FIFO full */
#define FDCAN_TXEFS_TEFL             (1 << 25)                             /* Bit 25: Tx Event FIFO element lost */
                                                                           /* Bits 26-31: Reserved */

/* FDCAN Tx event FIFO acknowledge register */

#define FDCAN_TXEFA_EFAI_SHIFT      (0)                                    /* Bits 0-3: Event FIFO acknowledge index */
#define FDCAN_TXEFA_EFAI_MASK       (0x3 << FDCAN_TXEFA_EFAI_SHIFT)

/* FDCAN CFG clock divider register */

#define FDCAN_CKDIV_PDIV_SHIFT      (0)                                    /* Bits 0-3: Input clock divider */
#define FDCAN_CKDIV_PDIV_MASK       (0xf << FDCAN_CKDIV_PDIV_SHIFT)

/* Message RAM Definitions **************************************************/

/* Common Buffer and FIFO element bit definitions:
 *
 *   --------------- ------------------- --------------------------------
 *   RESOURCE               R0                        R1
 *   --------------- ------------------- --------------------------------
 *   RX FIFO:        ESI, XTD, RTR, ID,  ANMF, FIDX, EDL, BRS, DLC, RXTS
 *   TX buffer:           XTD, RTR, ID,  MM,   EFC,            DLC
 *   TX Event FIFO:  ESI, XTD, RTR, ID,  MM,   ET,   EDL, BRS, DLC, TXTS
 *   --------------- ------------------- --------------------------------
 */

/* Common */

#define BUFFER_R0_EXTID_SHIFT      (0)                                     /* Bits 0-28: Extended identifier */
#define BUFFER_R0_EXTID_MASK       (0x1fffffff << BUFFER_R0_EXTID_SHIFT)
#  define BUFFER_R0_EXTID(n)       ((uint32_t)(n) << BUFFER_R0_EXTID_SHIFT)
#define BUFFER_R0_STDID_SHIFT      (18)                                    /* Bits 18-28: Standard identifier */
#define BUFFER_R0_STDID_MASK       (0x7ff << BUFFER_R0_STDID_SHIFT)
#  define BUFFER_R0_STDID(n)       ((uint32_t)(n) << BUFFER_R0_STDID_SHIFT)
#define BUFFER_R0_RTR              (1 << 29)                               /* Bit 29: Remote Transmission Request */
#define BUFFER_R0_XTD              (1 << 30)                               /* Bit 30: Extended Identifier */
#define BUFFER_R0_ESI              (1 << 31)                               /* Bit 31: Error State Indicator */

/* Common */

#define BUFFER_R1_DLC_SHIFT        (16)                                    /* Bits 16-19: Date length code */
#define BUFFER_R1_DLC_MASK         (15 << BUFFER_R1_DLC_SHIFT)
#  define BUFFER_R1_DLC(n)         ((uint32_t)(n) << BUFFER_R1_DLC_SHIFT)
#define BUFFER_R1_BRS              (1 << 20)                               /* Bit 20: Bit Rate Switch */
#define BUFFER_R1_FDF              (1 << 21)                               /* Bit 21: FD Format */

/* RX buffer/RX FIFOs */

#define BUFFER_R1_RXTS_SHIFT       (0)                                     /* Bits 0-15: RX Timestamp */
#define BUFFER_R1_RXTS_MASK        (0xffff << BUFFER_R1_RXTS_SHIFT)
#  define BUFFER_R1_RXTS(n)        ((uint32_t)(n) << BUFFER_R1_RXTS_SHIFT)
#define BUFFER_R1_FIDX_SHIFT       (24)                                    /* Bits 24-30: Filter index */
#define BUFFER_R1_FIDX_MASK        (0x7f << BUFFER_R1_FIDX_SHIFT)
#  define BUFFER_R1_FIDX(n)        ((uint32_t)(n) << BUFFER_R1_FIDX_SHIFT)
#define BUFFER_R1_ANMF             (1 << 31)                               /* Bit 31: Accepted Non-matching Frame */

/* TX buffer/TX Event FIFO */

#define BUFFER_R1_MM_SHIFT         (24)                                    /* Bits 24-31: Message Marker */
#define BUFFER_R1_MM_MASK          (0xff << BUFFER_R1_MM_SHIFT)
#  define BUFFER_R1_MM(n)          ((uint32_t)(n) << BUFFER_R1_MM_SHIFT)

/* TX buffer */

#define BUFFER_R1_EFC              (1 << 23)                               /* Bit 23: Event FIFO Control */

/* TX Event FIFO */

#define BUFFER_R1_TXTS_SHIFT       (0)                                     /* Bits 0-15: TX Timestamp */
#define BUFFER_R1_TXTS_MASK        (0xffff << BUFFER_R1_TXTS_SHIFT)
#  define BUFFER_R1_TXTS(n)        ((uint32_t)(n) << BUFFER_R1_TXTS_SHIFT)
#define BUFFER_R1_EDL              (1 << 21)                               /* Bit 21: Extended Data Length */
#define BUFFER_R1_ET_SHIFT         (22)                                    /* Bits 22-23: Event Type */
#define BUFFER_R1_ET_MASK          (3 << BUFFER_R1_ET_SHIFT)
#  define BUFFER_R1_ET_TXEVENT     (1 << BUFFER_R1_ET_SHIFT)               /* Tx event */
#  define BUFFER_R1_ET_TXCANCEL    (2 << BUFFER_R1_ET_SHIFT)               /* Transmission despite cancellation */

/* Standard Message ID Filter Element */

#define STDFILTER_S0_SFID2_SHIFT      (0)                                  /* Bits 0-10: Standard Filter ID 2 */
#define STDFILTER_S0_SFID2_MASK       (0x7ff << STDFILTER_S0_SFID2_SHIFT)
#  define STDFILTER_S0_SFID2(n   )    ((uint32_t)(n) << STDFILTER_S0_SFID2_SHIFT)
#define STDFILTER_S0_BUFFER_SHIFT     (0)                                  /* Bits 0-5: RX buffer start address */
#define STDFILTER_S0_BUFFER_MASK      (63 << STDFILTER_S0_BUFFER_SHIFT)
#  define STDFILTER_S0_BUFFER(n)      ((uint32_t)(n) << STDFILTER_S0_BUFFER_SHIFT)
#define STDFILTER_S0_ACTION_SHIFT     (9)                                  /* Bits 9-10: Action taken */
#define STDFILTER_S0_ACTION_MASK      (3 << STDFILTER_S0_ACTION_SHIFT)
#  define STDFILTER_S0_RXBUFFER       (0 << STDFILTER_S0_ACTION_SHIFT)     /* Store message in a Rx buffer */
#  define STDFILTER_S0_DEBUGA         (1 << STDFILTER_S0_ACTION_SHIFT)     /* Debug Message A */
#  define STDFILTER_S0_DEBUGB         (2 << STDFILTER_S0_ACTION_SHIFT)     /* Debug Message B */
#  define STDFILTER_S0_DEBUGC         (3 << STDFILTER_S0_ACTION_SHIFT)     /* Debug Message C */
#define STDFILTER_S0_SFID1_SHIFT      (16)                                 /* Bits 16-26: Standard Filter ID 2 */
#define STDFILTER_S0_SFID1_MASK       (0x7ff << STDFILTER_S0_SFID1_SHIFT)
#  define STDFILTER_S0_SFID1(n)       ((uint32_t)(n) << STDFILTER_S0_SFID1_SHIFT)
#define STDFILTER_S0_SFEC_SHIFT       (27)                                 /* Bits 27-29: Standard Filter Element Configuration */
#define STDFILTER_S0_SFEC_MASK        (7 << STDFILTER_S0_SFEC_SHIFT)
#  define STDFILTER_S0_SFEC_DISABLE   (0 << STDFILTER_S0_SFEC_SHIFT)       /* Disable filter element */
#  define STDFILTER_S0_SFEC_FIFO0     (1 << STDFILTER_S0_SFEC_SHIFT)       /* Store in Rx FIFO 0 on match */
#  define STDFILTER_S0_SFEC_FIFO1     (2 << STDFILTER_S0_SFEC_SHIFT)       /* Store in Rx FIFO 1 on match */
#  define STDFILTER_S0_SFEC_REJECT    (3 << STDFILTER_S0_SFEC_SHIFT)       /* Reject ID on match */
#  define STDFILTER_S0_SFEC_PRIORITY  (4 << STDFILTER_S0_SFEC_SHIFT)       /* Set priority ion match */
#  define STDFILTER_S0_SFEC_PRIOFIFO0 (5 << STDFILTER_S0_SFEC_SHIFT)       /* Set priority and store in FIFO 0 on match */
#  define STDFILTER_S0_SFEC_PRIOFIFO1 (6 << STDFILTER_S0_SFEC_SHIFT)       /* Set priority and store in FIFO 1 on match */
#  define STDFILTER_S0_SFEC_BUFFER    (7 << STDFILTER_S0_SFEC_SHIFT)       /* Store into Rx Buffer or as debug message */
#define STDFILTER_S0_SFT_SHIFT        (30)                                 /* Bits 30-31: Standard Filter Type */
#define STDFILTER_S0_SFT_MASK         (3 << STDFILTER_S0_SFT_SHIFT)
#  define STDFILTER_S0_SFT_RANGE      (0 << STDFILTER_S0_SFT_SHIFT)        /* Range filter from SF1ID to SF2ID */
#  define STDFILTER_S0_SFT_DUAL       (1 << STDFILTER_S0_SFT_SHIFT)        /* Dual ID filter for SF1ID or SF2ID */
#  define STDFILTER_S0_SFT_CLASSIC    (2 << STDFILTER_S0_SFT_SHIFT)        /* Classic filter: SF1ID=filter SF2ID=mask */

/* Extended Message ID Filter Element */

#define EXTFILTER_F0_EFID1_SHIFT      (0)                                  /* Bits 0-28: Extended Filter ID 1 */
#define EXTFILTER_F0_EFID1_MASK       (0x1fffffff << EXTFILTER_F0_EFID1_SHIFT)
#  define EXTFILTER_F0_EFID1(n)       ((uint32_t)(n) << EXTFILTER_F0_EFID1_SHIFT)
#define EXTFILTER_F0_EFEC_SHIFT       (29)                                 /* Bits 29-31: Extended Filter Element Configuration */
#define EXTFILTER_F0_EFEC_MASK        (7 << EXTFILTER_F0_EFEC_SHIFT)
#  define EXTFILTER_F0_EFEC_DISABLE   (0 << EXTFILTER_F0_EFEC_SHIFT)       /* Disable filter element */
#  define EXTFILTER_F0_EFEC_FIFO0     (1 << EXTFILTER_F0_EFEC_SHIFT)       /* Store in Rx FIFO 0 on match */
#  define EXTFILTER_F0_EFEC_FIFO1     (2 << EXTFILTER_F0_EFEC_SHIFT)       /* Store in Rx FIFO 1 on match */
#  define EXTFILTER_F0_EFEC_REJECT    (3 << EXTFILTER_F0_EFEC_SHIFT)       /* Reject ID on match */
#  define EXTFILTER_F0_EFEC_PRIORITY  (4 << EXTFILTER_F0_EFEC_SHIFT)       /* Set priority on match */
#  define EXTFILTER_F0_EFEC_PRIOFIFO0 (5 << EXTFILTER_F0_EFEC_SHIFT)       /* Set priority and store in FIFO 0 on match */
#  define EXTFILTER_F0_EFEC_PRIOFIFO1 (6 << EXTFILTER_F0_EFEC_SHIFT)       /* Set priority and store in FIFO 1 on match */
#  define EXTFILTER_F0_EFEC_BUFFER    (7 << EXTFILTER_F0_EFEC_SHIFT)       /* Store into Rx Buffer or as debug message */

#define EXTFILTER_F1_EFID2_SHIFT      (0)                                  /* Bits 0-28: Extended Filter ID 2 */
#define EXTFILTER_F1_EFID2_MASK       (0x1fffffff << EXTFILTER_F1_EFID2_SHIFT)
#  define EXTFILTER_F1_EFID2(n)       ((uint32_t)(n) << EXTFILTER_F1_EFID2_SHIFT)
#define EXTFILTER_F1_BUFFER_SHIFT     (0)                                  /* Bits 0-5: RX buffer start address */
#define EXTFILTER_F1_BUFFER_MASK      (63 << EXTFILTER_F1_BUFFER_SHIFT)
#  define EXTFILTER_F1_BUFFER(n)      ((uint32_t)(n) << EXTFILTER_F1_BUFFER_SHIFT)
#define EXTFILTER_F1_ACTION_SHIFT     (9)                                  /* Bits 9-10: Action taken */
#define EXTFILTER_F1_ACTION_MASK      (3 << EXTFILTER_F1_ACTION_SHIFT)
#  define EXTFILTER_F1_RXBUFFER       (0 << EXTFILTER_F1_ACTION_SHIFT)     /* Store message in a Rx buffer */
#  define EXTFILTER_F1_DEBUGA         (1 << EXTFILTER_F1_ACTION_SHIFT)     /* Debug Message A */
#  define EXTFILTER_F1_DEBUGB         (2 << EXTFILTER_F1_ACTION_SHIFT)     /* Debug Message B */
#  define EXTFILTER_F1_DEBUGC         (3 << EXTFILTER_F1_ACTION_SHIFT)     /* Debug Message C */
#define EXTFILTER_F1_EFT_SHIFT        (30)                                 /* Bits 30-31: Extended Filter Type */
#define EXTFILTER_F1_EFT_MASK         (3 << EXTFILTER_F1_EFT_SHIFT)
#  define EXTFILTER_F1_EFT_RANGE      (0 << EXTFILTER_F1_EFT_SHIFT)        /* Range filter from SF1ID to SF2ID */
#  define EXTFILTER_F1_EFT_DUAL       (1 << EXTFILTER_F1_EFT_SHIFT)        /* Dual ID filter for SF1ID or SF2ID */
#  define EXTFILTER_F1_EFT_CLASSIC    (2 << EXTFILTER_F1_EFT_SHIFT)        /* Classic filter: SF1ID=filter SF2ID=mask */
#  define EXTFILTER_F1_EFT_NOXIDAM    (3 << EXTFILTER_F1_EFT_SHIFT)        /* Range filter from EF1ID to EF2ID, no XIDAM */

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_FDCAN_H */
