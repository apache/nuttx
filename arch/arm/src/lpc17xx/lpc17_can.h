/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_can.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_CAN_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_CAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "lp17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* CAN acceptance filter registers */

#define LPC17_CANAF_AFMR_OFFSET     0x0000 /* Acceptance Filter Register */
#define LPC17_CANAF_SFFSA_OFFSET    0x0004 /* Standard Frame Individual Start Address Register */
#define LPC17_CANAF_SFFGRPSA_OFFSET 0x0008 /* Standard Frame Group Start Address Register */
#define LPC17_CANAF_EFFSA_OFFSET    0x000c /* Extended Frame Start Address Register */
#define LPC17_CANAF_EFFGRPSA_OFFSET 0x0010 /* Extended Frame Group Start Address Register */
#define LPC17_CANAF_EOT_OFFSET      0x0014 /* End of AF Tables register */
#define LPC17_CANAF_LUTERRAD_OFFSET 0x0018 /* LUT Error Address register */
#define LPC17_CANAF_LUTERR_OFFSET   0x001c /* LUT Error Register */
#define LPC17_CANAF_FCANIE_OFFSET   0x0020 /* FullCAN interrupt enable register */
#define LPC17_CANAF_FCANIC0_OFFSET  0x0024 /* FullCAN interrupt and capture register 0 */
#define LPC17_CANAF_FCANIC1_OFFSET  0x0028 /* FullCAN interrupt and capture register 1 */

/* Central CAN registers */

#define LPC17_CAN_TXSR_OFFSET       0x0000 /* CAN Central Transmit Status Register */
#define LPC17_CAN_RXSR_OFFSET       0x0004 /* CAN Central Receive Status Register */
#define LPC17_CAN_MSR_OFFSET        0x0008 /* CAN Central Miscellaneous Register */

/* CAN1/2 registers */

#define LPC17_CAN_MOD_OFFSET        0x0000 /* CAN operating mode */
#define LPC17_CAN_CMR_OFFSET        0x0004 /* Command bits */
#define LPC17_CAN_GSR_OFFSET        0x0008 /* Controller Status and Error Counters */
#define LPC17_CAN_ICR_OFFSET        0x000c /* Interrupt status */
#define LPC17_CAN_IER_OFFSET        0x0010 /* Interrupt Enable */
#define LPC17_CAN_BTR_OFFSET        0x0014 /* Bus Timing */
#define LPC17_CAN_EWL_OFFSET        0x0018 /* Error Warning Limit */
#define LPC17_CAN_SR_OFFSET         0x001c /* Status Register */
#define LPC17_CAN_RFS_OFFSET        0x0020 /* Receive frame status */
#define LPC17_CAN_RID_OFFSET        0x0024 /* Received Identifier */
#define LPC17_CAN_RDA_OFFSET        0x0028 /* Received data bytes 1-4 */
#define LPC17_CAN_RDB_OFFSET        0x002c /* Received data bytes 5-8 */
#define LPC17_CAN_TFI1_OFFSET       0x0030 /* Transmit frame info (Tx Buffer 1) */
#define LPC17_CAN_TID1_OFFSET       0x0034 /* Transmit Identifier (Tx Buffer 1) */
#define LPC17_CAN_TDA1_OFFSET       0x0038 /* Transmit data bytes 1-4 (Tx Buffer 1) */
#define LPC17_CAN_TDB1_OFFSET       0x003c /* Transmit data bytes 5-8 (Tx Buffer 1) */
#define LPC17_CAN_TFI2_OFFSET       0x0040 /* Transmit frame info (Tx Buffer 2) */
#define LPC17_CAN_TID2_OFFSET       0x0044 /* Transmit Identifier (Tx Buffer 2) */
#define LPC17_CAN_TDA2_OFFSET       0x0048 /* Transmit data bytes 1-4 (Tx Buffer 2) */
#define LPC17_CAN_TDB2_OFFSET       0x004c /* Transmit data bytes 5-8 (Tx Buffer 2) */
#define LPC17_CAN_TFI3_OFFSET       0x0050 /* Transmit frame info (Tx Buffer 3) */
#define LPC17_CAN_TID3_OFFSET       0x0054 /* Transmit Identifier (Tx Buffer 3) */
#define LPC17_CAN_TDA3_OFFSET       0x0058 /* Transmit data bytes 1-4 (Tx Buffer 3) */
#define LPC17_CAN_TDB3_OFFSET       0x005c /* Transmit data bytes 5-8 (Tx Buffer 3) */

/* Register addresses ***************************************************************/
/* CAN acceptance filter registers */

#define LPC17_CANAF_AFMR            (LPC17_CANAF_BASE+LPC17_CANAF_AFMR_OFFSET)
#define LPC17_CANAF_SFFSA           (LPC17_CANAF_BASE+LPC17_CANAF_SFFSA_OFFSET)
#define LPC17_CANAF_SFFGRPSA        (LPC17_CANAF_BASE+LPC17_CANAF_SFFGRPSA_OFFSET)
#define LPC17_CANAF_EFFSA           (LPC17_CANAF_BASE+LPC17_CANAF_EFFSA_OFFSET)
#define LPC17_CANAF_EFFGRPSA        (LPC17_CANAF_BASE+LPC17_CANAF_EFFGRPSA_OFFSET)
#define LPC17_CANAF_EOT             (LPC17_CANAF_BASE+LPC17_CANAF_EOT_OFFSET)
#define LPC17_CANAF_LUTERRAD        (LPC17_CANAF_BASE+LPC17_CANAF_LUTERRAD_OFFSET)
#define LPC17_CANAF_LUTERR          (LPC17_CANAF_BASE+LPC17_CANAF_LUTERR_OFFSET)
#define LPC17_CANAF_FCANIE          (LPC17_CANAF_BASE+LPC17_CANAF_FCANIE_OFFSET)
#define LPC17_CANAF_FCANIC0         (LPC17_CANAF_BASE+LPC17_CANAF_FCANIC0_OFFSET)
#define LPC17_CANAF_FCANIC1         (LPC17_CANAF_BASE+LPC17_CANAF_FCANIC1_OFFSET)

/* Central CAN registers */

#define LPC17_CAN_TXSR              (LPC17_CAN_BASE+LPC17_CAN_TXSR_OFFSET)
#define LPC17_CAN_RXSR              (LPC17_CAN_BASE+LPC17_CAN_RXSR_OFFSET)
#define LPC17_CAN_MSR               (LPC17_CAN_BASE+LPC17_CAN_MSR_OFFSET)

/* CAN1/2 registers */

#define LPC17_CAN1_MOD              (LPC17_CAN1_BASE+LPC17_CAN_MOD_OFFSET)
#define LPC17_CAN1_CMR              (LPC17_CAN1_BASE+LPC17_CAN_CMR_OFFSET)
#define LPC17_CAN1_GSR              (LPC17_CAN1_BASE+LPC17_CAN_GSR_OFFSET)
#define LPC17_CAN1_ICR              (LPC17_CAN1_BASE+LPC17_CAN_ICR_OFFSET)
#define LPC17_CAN1_IER              (LPC17_CAN1_BASE+LPC17_CAN_IER_OFFSET)
#define LPC17_CAN1_BTR              (LPC17_CAN1_BASE+LPC17_CAN_BTR_OFFSET)
#define LPC17_CAN1_EWL              (LPC17_CAN1_BASE+LPC17_CAN_EWL_OFFSET)
#define LPC17_CAN1_SR               (LPC17_CAN1_BASE+LPC17_CAN_SR_OFFSET)
#define LPC17_CAN1_RFS              (LPC17_CAN1_BASE+LPC17_CAN_RFS_OFFSET)
#define LPC17_CAN1_RID              (LPC17_CAN1_BASE+LPC17_CAN_RID_OFFSET)
#define LPC17_CAN1_RDA              (LPC17_CAN1_BASE+LPC17_CAN_RDA_OFFSET)
#define LPC17_CAN1_RDB              (LPC17_CAN1_BASE+LPC17_CAN_RDB_OFFSET)
#define LPC17_CAN1_TFI1             (LPC17_CAN1_BASE+LPC17_CAN_TFI1_OFFSET)
#define LPC17_CAN1_TID1             (LPC17_CAN1_BASE+LPC17_CAN_TID1_OFFSET)
#define LPC17_CAN1_TDA1             (LPC17_CAN1_BASE+LPC17_CAN_TDA1_OFFSET)
#define LPC17_CAN1_TDB1             (LPC17_CAN1_BASE+LPC17_CAN_TDB1_OFFSET)
#define LPC17_CAN1_TFI2             (LPC17_CAN1_BASE+LPC17_CAN_TFI2_OFFSET)
#define LPC17_CAN1_TID2             (LPC17_CAN1_BASE+LPC17_CAN_TID2_OFFSET)
#define LPC17_CAN1_TDA2             (LPC17_CAN1_BASE+LPC17_CAN_TDA2_OFFSET)
#define LPC17_CAN1_TDB2             (LPC17_CAN1_BASE+LPC17_CAN_TDB2_OFFSET)
#define LPC17_CAN1_TFI3             (LPC17_CAN1_BASE+LPC17_CAN_TFI3_OFFSET)
#define LPC17_CAN1_TID3             (LPC17_CAN1_BASE+LPC17_CAN_TID3_OFFSET)
#define LPC17_CAN1_TDA3             (LPC17_CAN1_BASE+LPC17_CAN_TDA3_OFFSET)
#define LPC17_CAN1_TDB3             (LPC17_CAN1_BASE+LPC17_CAN_TDB3_OFFSET)

#define LPC17_CAN2_MOD              (LPC17_CAN2_BASE+LPC17_CAN_MOD_OFFSET)
#define LPC17_CAN2_CMR              (LPC17_CAN2_BASE+LPC17_CAN_CMR_OFFSET)
#define LPC17_CAN2_GSR              (LPC17_CAN2_BASE+LPC17_CAN_GSR_OFFSET)
#define LPC17_CAN2_ICR              (LPC17_CAN2_BASE+LPC17_CAN_ICR_OFFSET)
#define LPC17_CAN2_IER              (LPC17_CAN2_BASE+LPC17_CAN_IER_OFFSET)
#define LPC17_CAN2_BTR              (LPC17_CAN2_BASE+LPC17_CAN_BTR_OFFSET)
#define LPC17_CAN2_EWL              (LPC17_CAN2_BASE+LPC17_CAN_EWL_OFFSET)
#define LPC17_CAN2_SR               (LPC17_CAN2_BASE+LPC17_CAN_SR_OFFSET)
#define LPC17_CAN2_RFS              (LPC17_CAN2_BASE+LPC17_CAN_RFS_OFFSET)
#define LPC17_CAN2_RID              (LPC17_CAN2_BASE+LPC17_CAN_RID_OFFSET)
#define LPC17_CAN2_RDA              (LPC17_CAN2_BASE+LPC17_CAN_RDA_OFFSET)
#define LPC17_CAN2_RDB              (LPC17_CAN2_BASE+LPC17_CAN_RDB_OFFSET)
#define LPC17_CAN2_TFI1             (LPC17_CAN2_BASE+LPC17_CAN_TFI1_OFFSET)
#define LPC17_CAN2_TID1             (LPC17_CAN2_BASE+LPC17_CAN_TID1_OFFSET)
#define LPC17_CAN2_TDA1             (LPC17_CAN2_BASE+LPC17_CAN_TDA1_OFFSET)
#define LPC17_CAN2_TDB1             (LPC17_CAN2_BASE+LPC17_CAN_TDB1_OFFSET)
#define LPC17_CAN2_TFI2             (LPC17_CAN2_BASE+LPC17_CAN_TFI2_OFFSET)
#define LPC17_CAN2_TID2             (LPC17_CAN2_BASE+LPC17_CAN_TID2_OFFSET)
#define LPC17_CAN2_TDA2             (LPC17_CAN2_BASE+LPC17_CAN_TDA2_OFFSET)
#define LPC17_CAN2_TDB2             (LPC17_CAN2_BASE+LPC17_CAN_TDB2_OFFSET)
#define LPC17_CAN2_TFI3             (LPC17_CAN2_BASE+LPC17_CAN_TFI3_OFFSET)
#define LPC17_CAN2_TID3             (LPC17_CAN2_BASE+LPC17_CAN_TID3_OFFSET)
#define LPC17_CAN2_TDA3             (LPC17_CAN2_BASE+LPC17_CAN_TDA3_OFFSET)
#define LPC17_CAN2_TDB3             (LPC17_CAN2_BASE+LPC17_CAN_TDB3_OFFSET)

/* Register bit definitions *********************************************************/
/* CAN acceptance filter registers */
/* Acceptance Filter Register */

#define CANAF_AFMR_ACCOFF           (1 << 0)  /* Bit 0:  AF non-operational; All RX messages ignored */
#define CANAF_AFMR_ACCBP            (1 << 1)  /* Bit 1:  AF bypass: All RX messages accepted */
#define CANAF_AFMR_EFCAN            (1 << 2)  /* Bit 2:  Enable Full CAN mode */
                                              /* Bits 3-31: Reserved */
/* Standard Frame Individual Start Address Register */
                                              /* Bits 0-1: Reserved */
#define CANAF_SFFSA_SHIFT           (2)       /* Bits 2-10: Address of Standard Identifiers in AF Lookup RAM */
#define CANAF_SFFSA_MASK            (0x01ff << CANAF_SFFSA_SHIFT)
                                              /* Bits 11-31: Reserved */
/* Standard Frame Group Start Address Register */
                                              /* Bits 0-1: Reserved */
#define CANAF_SFFGRPSA_SHIFT        (2)       /* Bits 2-10: Address of grouped Standard Identifiers in AF Lookup RAM */
#define CANAF_SFFGRPSA_MASK         (0x01ff << CANAF_SFFGRPSA_SHIFT)
                                              /* Bits 11-31: Reserved */
/* Extended Frame Start Address Register */
                                              /* Bits 0-1: Reserved */
#define CANAF_EFFSA_SHIFT           (2)       /* Bits 2-10: Address of Extended Identifiers in AF Lookup RAM */
#define CANAF_EFFSA_MASK            (0x01ff << CANAF_EFFSA_SHIFT)
                                              /* Bits 11-31: Reserved */
/* Extended Frame Group Start Address Register */
                                              /* Bits 0-1: Reserved */
#define CANAF_EFFGRPSA_SHIFT        (2)       /* Bits 2-10: Address of grouped Extended Identifiers in AF Lookup RAM */
#define CANAF_EFFGRPSA_MASK         (0x01ff << CANAF_EFFGRPSA_SHIFT)
                                              /* Bits 11-31: Reserved */
/* End of AF Tables register */
                                              /* Bits 0-1: Reserved */
#define CANAF_EOT_SHIFT             (2)       /* Bits 2-10: Last active address in last active AF table */
#define CANAF_EOT_MASK              (0x01ff << CANAF_EOT_SHIFT)
                                              /* Bits 11-31: Reserved */
/* LUT Error Address register */
                                              /* Bits 0-1: Reserved */
#define CANAF_LUTERRAD_SHIFT        (2)       /* Bits 2-10: Address in AF Lookup RAM of error */
#define CANAF_LUTERRAD_MASK         (0x01ff << CANAF_EOT_SHIFT)
                                              /* Bits 11-31: Reserved */
/* LUT Error Register */

#define CANAF_LUTERR_LUTERR         (1 << 0)  /* Bit 0: AF error in AF RAM tables */
                                              /* Bits 1-31: Reserved */
/* FullCAN interrupt enable register */

#define CANAF_FCANIE_FCANIE         (1 << 0)  /* Bit 0: Global FullCAN Interrupt Enable */
                                              /* Bits 1-31: Reserved */

/* FullCAN interrupt and capture register 0 */

#define CANAF_FCANIC0_INTPND(n)     (1 << (n)) /* n=0,1,2,... 31 */

/* FullCAN interrupt and capture register 1 */

#define CANAF_FCANIC1_INTPND(n)     (1 << ((n)-32)) /* n=32,33,...63 */

/* Central CAN registers */
/* CAN Central Transmit Status Register */

#define CAN_TXSR_TS1                (1 << 0)  /* Bit 0:  CAN1 sending */
#define CAN_TXSR_TS2                (1 << 1)  /* Bit 1:  CAN2 sending */
                                              /* Bits 2-7: Reserved */
#define CAN_TXSR_TBS1               (1 << 8)  /* Bit 8:  All 3 CAN1 TX buffers available */
#define CAN_TXSR_TBS2               (1 << 9)  /* Bit 9:  All 3 CAN2 TX buffers available */
                                              /* Bits 10-15: Reserved */
#define CAN_TXSR_TCS1               (1 << 16) /* Bit 16:  All CAN1 xmissions completed */
#define CAN_TXSR_TCS2               (1 << 17) /* Bit 17:  All CAN2 xmissions completed */
                                              /* Bits 18-31: Reserved */
/* CAN Central Receive Status Register */

#define CAN_RXSR_RS1                (1 << 0)  /* Bit 0:  CAN1 receiving */
#define CAN_RXSR_RS2                (1 << 1)  /* Bit 1:  CAN2 receiving */
                                              /* Bits 2-7: Reserved */
#define CAN_RXSR_RB1                (1 << 8)  /* Bit 8:  CAN1 received message available */
#define CAN_RXSR_RB2                (1 << 9)  /* Bit 9:  CAN2 received message available */
                                              /* Bits 10-15: Reserved */
#define CAN_RXSR_DOS1               (1 << 16) /* Bit 16:  All CAN1 message lost */
#define CAN_RXSR_DOS2               (1 << 17) /* Bit 17:  All CAN2 message lost */
                                              /* Bits 18-31: Reserved */
/* CAN Central Miscellaneous Register */

#define CAN_MSR_E1                  (1 << 0)  /* Bit 0:  CAN1 error counters at limit */
#define CAN_MSR_E2                  (1 << 1)  /* Bit 1:  CAN2 error counters at limit */
                                              /* Bits 2-7: Reserved */
#define CAN_MSR_BS1                 (1 << 8)  /* Bit 8:  CAN1 busy */
#define CAN_MSR_BS2                 (1 << 9)  /* Bit 7:  CAN2 busy */
                                              /* Bits 10-31: Reserved */
/* CAN1/2 registers */
/* CAN operating mode */

#define CAN_MOD_RM                  (1 << 0)  /* Bit 0:  Reset Mode */
#define CAN_MOD_LOM                 (1 << 1)  /* Bit 1:  Listen Only Mode */
#define CAN_MOD_STM                 (1 << 2)  /* Bit 2:  Self Test Mode */
#define CAN_MOD_TPM                 (1 << 3)  /* Bit 3:  Transmit Priority Mode */
#define CAN_MOD_SM                  (1 << 4)  /* Bit 4:  Sleep Mode */
#define CAN_MOD_RPM                 (1 << 5)  /* Bit 5:  Receive Polarity Mode */
                                              /* Bit 6:  Reserved */
#define CAN_MOD_TM                  (1 << 7)  /* Bit 7:  Test Mode
                                              /* Bits 8-31: Reserved */
/* Command bits */

#define CAN_CMR_

/* Controller Status and Error Counters */

#define CAN_GSR_

/* Interrupt status */

#define CAN_ICR_

/* Interrupt Enable */

#define CAN_IER_

/* Bus Timing */

#define CAN_BTR_

/* Error Warning Limit */

#define CAN_EWL_

/* Status Register */

#define CAN_SR_

/* Receive frame status */

#define CAN_RFS_

/* Received Identifier */

#define CAN_RID_

/* Received data bytes 1-4 */

#define CAN_RDA_

/* Received data bytes 5-8 */

#define CAN_RDB_

/* Transmit frame info (Tx Buffer 1) */

#define CAN_TFI1_

/* Transmit Identifier (Tx Buffer 1) */

#define CAN_TID1_

/* Transmit data bytes 1-4 (Tx Buffer 1) */

#define CAN_TDA1_

/* Transmit data bytes 5-8 (Tx Buffer 1) */

#define CAN_TDB1_

/* Transmit frame info (Tx Buffer 2) */

#define CAN_TFI2_

/* Transmit Identifier (Tx Buffer 2) */

#define CAN_TID2_

/* Transmit data bytes 1-4 (Tx Buffer 2) */

#define CAN_TDA2_

/* Transmit data bytes 5-8 (Tx Buffer 2) */

#define CAN_TDB2_

/* Transmit frame info (Tx Buffer 3) */

#define CAN_TFI3_

/* Transmit Identifier (Tx Buffer 3) */

#define CAN_TID3_

/* Transmit data bytes 1-4 (Tx Buffer 3) */

#define CAN_TDA3_

/* Transmit data bytes 5-8 (Tx Buffer 3) */

#define CAN_TDB3_

            (1 << xx)  /* Bit xx:  
_SHIFT        (xx)       /* Bits xx-yy: 
_MASK         (xx << yy)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_CAN_H */
