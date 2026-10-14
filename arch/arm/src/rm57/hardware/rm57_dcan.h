/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_dcan.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* DCAN (Bosch D_CAN) register layout for the RM57L843, transcribed from
 * SPNU562A chapter 26 and cross-checked against TI's HALCoGen
 * HL_reg_can.h.  All four instances (DCAN1-DCAN4) share this layout and
 * have 64 message objects each.
 *
 * The IFxCMD register is documented as four byte fields (MsgNum, STAT,
 * CMD); on this port it is always accessed as a single 32-bit register.
 * The Hercules peripheral bus is BE-32, so sub-word accesses swap byte
 * lanes on the little-endian R5F (see the note in rm57_sci.h) - full-word
 * accesses see the register exactly as drawn in the TRM figures.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_DCAN_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_DCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Message objects **********************************************************/

#define RM57_DCAN_NMSGOBJ            64     /* Message objects per instance */

/* Message RAM addressing: object n (1..64) lives at RAM base + n * 0x20;
 * object 64 wraps to offset 0.  The ECC bits of object n are memory
 * mapped at RAM base + 0x1000 + n * 0x20 (SECDED diagnostic mode only).
 * The RAM is only CPU-visible in Debug/Suspend or RAM Direct Access mode;
 * normal operation goes through the IF1/IF2/IF3 register sets.
 */

#define RM57_DCAN_RAM_MBOX(ram,n)    ((ram) + (((n) & 63) << 5))
#define RM57_DCAN_RAM_ECC(ram,n)     ((ram) + 0x1000 + (((n) & 63) << 5))

/* Register Offsets *********************************************************/

#define RM57_DCAN_CTL_OFFSET         0x0000 /* CAN Control Register */
#define RM57_DCAN_ES_OFFSET          0x0004 /* Error and Status Register */
#define RM57_DCAN_ERRC_OFFSET        0x0008 /* Error Counter Register */
#define RM57_DCAN_BTR_OFFSET         0x000c /* Bit Timing Register */
#define RM57_DCAN_INT_OFFSET         0x0010 /* Interrupt Register */
#define RM57_DCAN_TEST_OFFSET        0x0014 /* Test Register */
#define RM57_DCAN_PERR_OFFSET        0x001c /* Parity/SECDED Error Code Register */
#define RM57_DCAN_REL_OFFSET         0x0020 /* Core Release Register */
#define RM57_DCAN_ECCDIAG_OFFSET     0x0024 /* ECC Diagnostic Register */
#define RM57_DCAN_ECCDIAGSTAT_OFFSET 0x0028 /* ECC Diagnostic Status Register */
#define RM57_DCAN_ECCCS_OFFSET       0x002c /* ECC Control and Status Register */
#define RM57_DCAN_ECCSERR_OFFSET     0x0030 /* ECC Single-Bit Error Code Register */
#define RM57_DCAN_ABOTR_OFFSET       0x0080 /* Auto-Bus-On Time Register */
#define RM57_DCAN_TXRQX_OFFSET       0x0084 /* Transmission Request X (summary) */
#define RM57_DCAN_NWDATX_OFFSET      0x0098 /* New Data X (summary) */
#define RM57_DCAN_INTPNDX_OFFSET     0x00ac /* Interrupt Pending X (summary) */
#define RM57_DCAN_MSGVALX_OFFSET     0x00c0 /* Message Valid X (summary) */

/* TXRQ12/34/56/78, NWDAT12/34/56/78, INTPND12/34/56/78, MSGVAL12/34/56/78
 * and INTMUX12/34/56/78: four consecutive registers each, g = 0-3.
 */

#define RM57_DCAN_TXRQ_OFFSET(g)     (0x0088 + ((g) << 2))
#define RM57_DCAN_NWDAT_OFFSET(g)    (0x009c + ((g) << 2))
#define RM57_DCAN_INTPND_OFFSET(g)   (0x00b0 + ((g) << 2))
#define RM57_DCAN_MSGVAL_OFFSET(g)   (0x00c4 + ((g) << 2))
#define RM57_DCAN_INTMUX_OFFSET(g)   (0x00d8 + ((g) << 2))

/* IF1/IF2 interface register sets.  RM57_DCAN_IF1 / RM57_DCAN_IF2 may be
 * passed as the 'ifbase' offset to the IF register macros.
 */

#define RM57_DCAN_IF1                0x0100
#define RM57_DCAN_IF2                0x0120

#define RM57_DCAN_IFCMD_OFFSET(if)   ((if) + 0x0000) /* IFx Command Register */
#define RM57_DCAN_IFMSK_OFFSET(if)   ((if) + 0x0004) /* IFx Mask Register */
#define RM57_DCAN_IFARB_OFFSET(if)   ((if) + 0x0008) /* IFx Arbitration Register */
#define RM57_DCAN_IFMCTL_OFFSET(if)  ((if) + 0x000c) /* IFx Message Control Register */
#define RM57_DCAN_IFDATA_OFFSET(if)  ((if) + 0x0010) /* IFx Data A (bytes 0-3) */
#define RM57_DCAN_IFDATB_OFFSET(if)  ((if) + 0x0014) /* IFx Data B (bytes 4-7) */

#define RM57_DCAN_IF1CMD_OFFSET      0x0100 /* IF1 Command Register */
#define RM57_DCAN_IF1MSK_OFFSET      0x0104 /* IF1 Mask Register */
#define RM57_DCAN_IF1ARB_OFFSET      0x0108 /* IF1 Arbitration Register */
#define RM57_DCAN_IF1MCTL_OFFSET     0x010c /* IF1 Message Control Register */
#define RM57_DCAN_IF1DATA_OFFSET     0x0110 /* IF1 Data A Register */
#define RM57_DCAN_IF1DATB_OFFSET     0x0114 /* IF1 Data B Register */
#define RM57_DCAN_IF2CMD_OFFSET      0x0120 /* IF2 Command Register */
#define RM57_DCAN_IF2MSK_OFFSET      0x0124 /* IF2 Mask Register */
#define RM57_DCAN_IF2ARB_OFFSET      0x0128 /* IF2 Arbitration Register */
#define RM57_DCAN_IF2MCTL_OFFSET     0x012c /* IF2 Message Control Register */
#define RM57_DCAN_IF2DATA_OFFSET     0x0130 /* IF2 Data A Register */
#define RM57_DCAN_IF2DATB_OFFSET     0x0134 /* IF2 Data B Register */

/* IF3 auto-update interface register set (read-only mirror + control) */

#define RM57_DCAN_IF3OBS_OFFSET      0x0140 /* IF3 Observation Register */
#define RM57_DCAN_IF3MSK_OFFSET      0x0144 /* IF3 Mask Register (RO) */
#define RM57_DCAN_IF3ARB_OFFSET      0x0148 /* IF3 Arbitration Register (RO) */
#define RM57_DCAN_IF3MCTL_OFFSET     0x014c /* IF3 Message Control Register (RO) */
#define RM57_DCAN_IF3DATA_OFFSET     0x0150 /* IF3 Data A Register (RO) */
#define RM57_DCAN_IF3DATB_OFFSET     0x0154 /* IF3 Data B Register (RO) */

/* IF3 Update Enable registers IF3UPD12/34/56/78, g = 0-3 */

#define RM57_DCAN_IF3UPD_OFFSET(g)   (0x0160 + ((g) << 2))

#define RM57_DCAN_TIOC_OFFSET        0x01e0 /* CAN TX IO Control Register */
#define RM57_DCAN_RIOC_OFFSET        0x01e4 /* CAN RX IO Control Register */

/* Mailbox group helpers for the TXRQ/NWDAT/INTPND/MSGVAL/INTMUX/IF3UPD
 * register arrays: mailbox n (1..64) is bit (n-1) % 32 of group (n-1) / 32.
 */

#define RM57_DCAN_MBOX_GROUP(n)      (((n) - 1) >> 5)
#define RM57_DCAN_MBOX_BIT(n)        (1u << (((n) - 1) & 31))

/* Register Bit-Field Definitions *******************************************/

/* CAN Control Register (CTL) */

#define DCAN_CTL_INIT                (1 << 0)  /* Initialization mode */
#define DCAN_CTL_IE0                 (1 << 1)  /* Interrupt line 0 enable */
#define DCAN_CTL_SIE                 (1 << 2)  /* Status change interrupt enable */
#define DCAN_CTL_EIE                 (1 << 3)  /* Error interrupt enable */
#define DCAN_CTL_DAR                 (1 << 5)  /* Disable automatic retransmission */
#define DCAN_CTL_CCE                 (1 << 6)  /* Configuration change enable */
#define DCAN_CTL_TEST                (1 << 7)  /* Test mode enable */
#define DCAN_CTL_IDS                 (1 << 8)  /* Interruption debug support */
#define DCAN_CTL_ABO                 (1 << 9)  /* Auto-Bus-On enable */
#define DCAN_CTL_PMD_SHIFT           10        /* SECDED mode: 0x5 = disabled */
#define DCAN_CTL_PMD_MASK            (15 << DCAN_CTL_PMD_SHIFT)
#define DCAN_CTL_PMD_DISABLE         (5 << DCAN_CTL_PMD_SHIFT)
#define DCAN_CTL_PMD_ENABLE          (10 << DCAN_CTL_PMD_SHIFT)
#define DCAN_CTL_SWR                 (1 << 15) /* Software reset */
#define DCAN_CTL_INITDBG             (1 << 16) /* Init state while debug access */
#define DCAN_CTL_IE1                 (1 << 17) /* Interrupt line 1 enable */
#define DCAN_CTL_DE1                 (1 << 18) /* DMA request line IF1 enable */
#define DCAN_CTL_DE2                 (1 << 19) /* DMA request line IF2 enable */
#define DCAN_CTL_DE3                 (1 << 20) /* DMA request line IF3 enable */
#define DCAN_CTL_PDR                 (1 << 24) /* Request local power down */
#define DCAN_CTL_WUBA                (1 << 25) /* Wake up on bus activity */

/* Error and Status Register (ES).  Reading clears WakeUpPnd/PER/RxOK/TxOK
 * and sets LEC to 7 - service with a single read.
 */

#define DCAN_ES_LEC_SHIFT            0         /* Last error code */
#define DCAN_ES_LEC_MASK             (7 << DCAN_ES_LEC_SHIFT)
#  define DCAN_ES_LEC_NONE           (0 << DCAN_ES_LEC_SHIFT)
#  define DCAN_ES_LEC_STUFF          (1 << DCAN_ES_LEC_SHIFT)
#  define DCAN_ES_LEC_FORM           (2 << DCAN_ES_LEC_SHIFT)
#  define DCAN_ES_LEC_ACK            (3 << DCAN_ES_LEC_SHIFT)
#  define DCAN_ES_LEC_BIT1           (4 << DCAN_ES_LEC_SHIFT)
#  define DCAN_ES_LEC_BIT0           (5 << DCAN_ES_LEC_SHIFT)
#  define DCAN_ES_LEC_CRC            (6 << DCAN_ES_LEC_SHIFT)
#  define DCAN_ES_LEC_NOEVENT        (7 << DCAN_ES_LEC_SHIFT)
#define DCAN_ES_TXOK                 (1 << 3)  /* Transmitted successfully */
#define DCAN_ES_RXOK                 (1 << 4)  /* Received successfully */
#define DCAN_ES_EPASS                (1 << 5)  /* Error passive state */
#define DCAN_ES_EWARN                (1 << 6)  /* Warning state (counter >= 96) */
#define DCAN_ES_BOFF                 (1 << 7)  /* Bus-Off state */
#define DCAN_ES_PER                  (1 << 8)  /* Parity/SECDED error detected */
#define DCAN_ES_WAKEUPPND            (1 << 9)  /* Wake up pending */
#define DCAN_ES_PDA                  (1 << 10) /* Power down acknowledge */

/* Error Counter Register (ERRC) */

#define DCAN_ERRC_TEC_SHIFT          0         /* Transmit error counter */
#define DCAN_ERRC_TEC_MASK           (0xff << DCAN_ERRC_TEC_SHIFT)
#define DCAN_ERRC_REC_SHIFT          8         /* Receive error counter */
#define DCAN_ERRC_REC_MASK           (0x7f << DCAN_ERRC_REC_SHIFT)
#define DCAN_ERRC_RP                 (1 << 15) /* Receive error passive */

/* Bit Timing Register (BTR).  All fields are programmed as (value - 1).
 * Writable only with CTL.INIT and CTL.CCE both set.  Total bit time is
 * 8..25 tq; effective prescaler BRP+1 + 64*BRPE spans 1..1024.
 */

#define DCAN_BTR_BRP_SHIFT           0         /* Baud rate prescaler */
#define DCAN_BTR_BRP_MASK            (0x3f << DCAN_BTR_BRP_SHIFT)
#define DCAN_BTR_BRP(n)              (((n) & 0x3f) << DCAN_BTR_BRP_SHIFT)
#define DCAN_BTR_SJW_SHIFT           6         /* Synchronization jump width */
#define DCAN_BTR_SJW_MASK            (3 << DCAN_BTR_SJW_SHIFT)
#define DCAN_BTR_SJW(n)              (((n) & 3) << DCAN_BTR_SJW_SHIFT)
#define DCAN_BTR_TSEG1_SHIFT         8         /* Segment before sample point */
#define DCAN_BTR_TSEG1_MASK          (15 << DCAN_BTR_TSEG1_SHIFT)
#define DCAN_BTR_TSEG1(n)            (((n) & 15) << DCAN_BTR_TSEG1_SHIFT)
#define DCAN_BTR_TSEG2_SHIFT         12        /* Segment after sample point */
#define DCAN_BTR_TSEG2_MASK          (7 << DCAN_BTR_TSEG2_SHIFT)
#define DCAN_BTR_TSEG2(n)            (((n) & 7) << DCAN_BTR_TSEG2_SHIFT)
#define DCAN_BTR_BRPE_SHIFT          16        /* Prescaler extension */
#define DCAN_BTR_BRPE_MASK           (15 << DCAN_BTR_BRPE_SHIFT)
#define DCAN_BTR_BRPE(n)             (((n) & 15) << DCAN_BTR_BRPE_SHIFT)

/* Interrupt Register (INT) */

#define DCAN_INT_INT0ID_SHIFT        0         /* Line 0: mbox 1..64 or status */
#define DCAN_INT_INT0ID_MASK         (0xffff << DCAN_INT_INT0ID_SHIFT)
#define DCAN_INT_INT0ID_STATUS       0x8000    /* ES register updated */
#define DCAN_INT_INT1ID_SHIFT        16        /* Line 1: mbox 1..64 */
#define DCAN_INT_INT1ID_MASK         (0xff << DCAN_INT_INT1ID_SHIFT)

/* Test Register (TEST).  Writable only with CTL.TEST set. */

#define DCAN_TEST_SILENT             (1 << 3)  /* Silent (bus monitor) mode */
#define DCAN_TEST_LBACK              (1 << 4)  /* Internal loopback mode */
#define DCAN_TEST_TX_SHIFT           5         /* CAN_TX pin control */
#define DCAN_TEST_TX_MASK            (3 << DCAN_TEST_TX_SHIFT)
#define DCAN_TEST_RX                 (1 << 7)  /* CAN_RX pin monitor */
#define DCAN_TEST_EXL                (1 << 8)  /* External loopback mode */
#define DCAN_TEST_RDA                (1 << 9)  /* RAM direct access enable */

/* Parity Error Code Register (PERR) */

#define DCAN_PERR_MSGNUM_SHIFT       0         /* Erroneous message object */
#define DCAN_PERR_MSGNUM_MASK        (0xff << DCAN_PERR_MSGNUM_SHIFT)
#define DCAN_PERR_WORDNUM_SHIFT      8         /* Erroneous word (0..5) */
#define DCAN_PERR_WORDNUM_MASK       (7 << DCAN_PERR_WORDNUM_SHIFT)

/* ECC Diagnostic Register (ECCDIAG): 0x5 disables diagnostic mode */

#define DCAN_ECCDIAG_SHIFT           0
#define DCAN_ECCDIAG_MASK            (15 << DCAN_ECCDIAG_SHIFT)
#define DCAN_ECCDIAG_DISABLE         (5 << DCAN_ECCDIAG_SHIFT)
#define DCAN_ECCDIAG_ENABLE          (10 << DCAN_ECCDIAG_SHIFT)

/* ECC Diagnostic Status Register (ECCDIAG_STAT) */

#define DCAN_ECCDIAGSTAT_SEFLG       (1 << 0)  /* Single-bit error (diag mode) */
#define DCAN_ECCDIAGSTAT_DEFLG       (1 << 8)  /* Double-bit error (diag mode) */

/* ECC Control and Status Register (ECC_CS) */

#define DCAN_ECCCS_ECCMODE_SHIFT     0         /* 0xA enables SBE correction */
#define DCAN_ECCCS_ECCMODE_MASK      (15 << DCAN_ECCCS_ECCMODE_SHIFT)
#define DCAN_ECCCS_ECCMODE_ENABLE    (10 << DCAN_ECCCS_ECCMODE_SHIFT)
#define DCAN_ECCCS_SBERR             (1 << 8)  /* Single-bit error detected */
#define DCAN_ECCCS_DEDIS             (1 << 16) /* Disable double-bit detection */

/* ECC Single-Bit Error Code Register (ECC_SERR) */

#define DCAN_ECCSERR_MSGNUM_SHIFT    0
#define DCAN_ECCSERR_MSGNUM_MASK     (0xff << DCAN_ECCSERR_MSGNUM_SHIFT)

/* IF1/IF2 Command Register (IFxCMD).  Writing the message number starts
 * the transfer; the register set is write-protected while BUSY is set.
 */

#define DCAN_IFCMD_MSGNUM_SHIFT      0         /* Message object 1..64 */
#define DCAN_IFCMD_MSGNUM_MASK       (0xff << DCAN_IFCMD_MSGNUM_SHIFT)
#define DCAN_IFCMD_MSGNUM(n)         (((n) & 0xff) << DCAN_IFCMD_MSGNUM_SHIFT)
#define DCAN_IFCMD_DMAACTIVE         (1 << 14) /* Request DMA after transfer */
#define DCAN_IFCMD_BUSY              (1 << 15) /* Transfer in progress */
#define DCAN_IFCMD_DATAB             (1 << 16) /* Transfer data bytes 4-7 */
#define DCAN_IFCMD_DATAA             (1 << 17) /* Transfer data bytes 0-3 */
#define DCAN_IFCMD_TXRQST            (1 << 18) /* WR: set TxRqst; RD: clear NewDat */
#define DCAN_IFCMD_CLRINTPND         (1 << 19) /* RD: clear IntPnd */
#define DCAN_IFCMD_CONTROL           (1 << 20) /* Transfer control bits */
#define DCAN_IFCMD_ARB               (1 << 21) /* Transfer arbitration bits */
#define DCAN_IFCMD_MASK              (1 << 22) /* Transfer mask bits */
#define DCAN_IFCMD_WR                (1 << 23) /* 1: IF -> RAM, 0: RAM -> IF */

/* IF1/IF2/IF3 Mask Register (IFxMSK) */

#define DCAN_IFMSK_MSK_SHIFT         0         /* Identifier mask */
#define DCAN_IFMSK_MSK_MASK          (0x1fffffff << DCAN_IFMSK_MSK_SHIFT)
#define DCAN_IFMSK_MSK_STD_SHIFT     18        /* 11-bit ID mask position */
#define DCAN_IFMSK_MDIR              (1 << 30) /* Use Dir for filtering */
#define DCAN_IFMSK_MXTD              (1 << 31) /* Use IDE for filtering */

/* IF1/IF2/IF3 Arbitration Register (IFxARB) */

#define DCAN_IFARB_ID_SHIFT          0         /* 29-bit extended identifier */
#define DCAN_IFARB_ID_MASK           (0x1fffffff << DCAN_IFARB_ID_SHIFT)
#define DCAN_IFARB_ID_STD_SHIFT      18        /* 11-bit ID at bits 28:18 */
#define DCAN_IFARB_DIR               (1 << 29) /* 1 = transmit object */
#define DCAN_IFARB_XTD               (1 << 30) /* Extended identifier */
#define DCAN_IFARB_MSGVAL            (1 << 31) /* Message object valid */

/* IF1/IF2/IF3 Message Control Register (IFxMCTL) */

#define DCAN_IFMCTL_DLC_SHIFT        0         /* Data length code 0..8 */
#define DCAN_IFMCTL_DLC_MASK         (15 << DCAN_IFMCTL_DLC_SHIFT)
#define DCAN_IFMCTL_EOB              (1 << 7)  /* End of FIFO block */
#define DCAN_IFMCTL_TXRQST           (1 << 8)  /* Transmit request pending */
#define DCAN_IFMCTL_RMTEN            (1 << 9)  /* Remote frame sets TxRqst */
#define DCAN_IFMCTL_RXIE             (1 << 10) /* Receive interrupt enable */
#define DCAN_IFMCTL_TXIE             (1 << 11) /* Transmit interrupt enable */
#define DCAN_IFMCTL_UMASK            (1 << 12) /* Use acceptance mask */
#define DCAN_IFMCTL_INTPND           (1 << 13) /* Interrupt pending */
#define DCAN_IFMCTL_MSGLST           (1 << 14) /* Message lost (RX overwrite) */
#define DCAN_IFMCTL_NEWDAT           (1 << 15) /* New data available */

/* IF3 Observation Register (IF3OBS).  The low observation flags select
 * which IF3 sections must be read (each byte at least once) before the
 * hardware re-arms IF3 with the next queued message object; the status
 * flags report the sections still unread in the current cycle.
 */

#define DCAN_IF3OBS_MASK_OBS         (1 << 0)  /* Mask section must be read */
#define DCAN_IF3OBS_ARB_OBS          (1 << 1)  /* Arbitration must be read */
#define DCAN_IF3OBS_CTRL_OBS         (1 << 2)  /* Message control must be read */
#define DCAN_IF3OBS_DATAA_OBS        (1 << 3)  /* Data A must be read */
#define DCAN_IF3OBS_DATAB_OBS        (1 << 4)  /* Data B must be read */
#define DCAN_IF3OBS_SM               (1 << 8)  /* Mask section unread */
#define DCAN_IF3OBS_SA               (1 << 9)  /* Arbitration unread */
#define DCAN_IF3OBS_SC               (1 << 10) /* Control unread */
#define DCAN_IF3OBS_SDA              (1 << 11) /* Data A unread */
#define DCAN_IF3OBS_SDB              (1 << 12) /* Data B unread */
#define DCAN_IF3OBS_UPD              (1 << 15) /* New data loaded into IF3 */

/* TX/RX IO Control Registers (TIOC/RIOC).  Writable only with CTL.INIT
 * set; Func/Dir/OD are forced to functional values when INIT is cleared.
 */

#define DCAN_IOC_IN                  (1 << 0)  /* Pin input value */
#define DCAN_IOC_OUT                 (1 << 1)  /* Pin output value (GIO mode) */
#define DCAN_IOC_DIR                 (1 << 2)  /* Pin direction (GIO mode) */
#define DCAN_IOC_FUNC                (1 << 3)  /* 1 = CAN functional mode */
#define DCAN_IOC_OD                  (1 << 16) /* Open drain enable */
#define DCAN_IOC_PD                  (1 << 17) /* Pull disable */
#define DCAN_IOC_PU                  (1 << 18) /* Pullup select */

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_DCAN_H */
