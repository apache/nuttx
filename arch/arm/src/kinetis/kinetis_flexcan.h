/************************************************************************************
 * arch/arm/src/kinetis/kinetis_flexcan.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_FLEXCAN_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_FLEXCAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_CAN_MCR_OFFSET      0x0000 /* Module Configuration Register */
#define KINETIS_CAN_CTRL1_OFFSET    0x0004 /* Control 1 Register */
#define KINETIS_CAN_TIMER_OFFSET    0x0008 /* Free Running Timer */
#define KINETIS_CAN_RXMGMASK_OFFSET 0x0010 /* Rx Mailboxes Global Mask Register */
#define KINETIS_CAN_RX14MASK_OFFSET 0x0014 /* Rx 14 Mask Register */
#define KINETIS_CAN_RX15MASK_OFFSET 0x0018 /* Rx 15 Mask Register */
#define KINETIS_CAN_ECR_OFFSET      0x001c /* Error Counter */
#define KINETIS_CAN_ESR1_OFFSET     0x0020 /* Error and Status 1 Register */
#define KINETIS_CAN_IMASK2_OFFSET   0x0024 /* Interrupt Masks 2 Register */
#define KINETIS_CAN_IMASK1_OFFSET   0x0028 /* Interrupt Masks 1 Register */
#define KINETIS_CAN_IFLAG2_OFFSET   0x002c /* Interrupt Flags 2 Register */
#define KINETIS_CAN_IFLAG1_OFFSET   0x0030 /* Interrupt Flags 1 Register */
#define KINETIS_CAN_CTRL2_OFFSET    0x0034 /* Control 2 Register */
#define KINETIS_CAN_ESR2_OFFSET     0x0038 /* Error and Status 2 Register */
#define KINETIS_CAN_CRCR_OFFSET     0x0044 /* CRC Register */
#define KINETIS_CAN_RXFGMASK_OFFSET 0x0048 /* Rx FIFO Global Mask Register */
#define KINETIS_CAN_RXFIR_OFFSET    0x004c /* Rx FIFO Information Register */

#define KINETIS_CAN_RXIMR_OFFSET(n) (0x0880+((n)<<2)) /* Rn Individual Mask Registers */
#define KINETIS_CAN_RXIMR0_OFFSET   0x0880 /* R0 Individual Mask Registers */
#define KINETIS_CAN_RXIMR1_OFFSET   0x0884 /* R1 Individual Mask Registers */
#define KINETIS_CAN_RXIMR2_OFFSET   0x0888 /* R2 Individual Mask Registers */
#define KINETIS_CAN_RXIMR3_OFFSET   0x088c /* R3 Individual Mask Registers */
#define KINETIS_CAN_RXIMR4_OFFSET   0x0890 /* R4 Individual Mask Registers */
#define KINETIS_CAN_RXIMR5_OFFSET   0x0894 /* R5 Individual Mask Registers */
#define KINETIS_CAN_RXIMR6_OFFSET   0x0898 /* R6 Individual Mask Registers */
#define KINETIS_CAN_RXIMR7_OFFSET   0x089c /* R7 Individual Mask Registers */
#define KINETIS_CAN_RXIMR8_OFFSET   0x08a0 /* R8 Individual Mask Registers */
#define KINETIS_CAN_RXIMR9_OFFSET   0x08a4 /* R9 Individual Mask Registers */
#define KINETIS_CAN_RXIMR10_OFFSET  0x08a8 /* R10 Individual Mask Registers */
#define KINETIS_CAN_RXIMR11_OFFSET  0x08ac /* R11 Individual Mask Registers */
#define KINETIS_CAN_RXIMR12_OFFSET  0x08b0 /* R12 Individual Mask Registers */
#define KINETIS_CAN_RXIMR13_OFFSET  0x08b4 /* R13 Individual Mask Registers */
#define KINETIS_CAN_RXIMR14_OFFSET  0x08b8 /* R14 Individual Mask Registers */
#define KINETIS_CAN_RXIMR15_OFFSET  0x08bc /* R15 Individual Mask Registers */

/* Register Addresses ***************************************************************/

#define KINETIS_CAN0_MCR           (KINETIS_CAN0_BASE+KINETIS_CAN_MCR_OFFSET)
#define KINETIS_CAN0_CTRL1         (KINETIS_CAN0_BASE+KINETIS_CAN_CTRL1_OFFSET)
#define KINETIS_CAN0_TIMER         (KINETIS_CAN0_BASE+KINETIS_CAN_TIMER_OFFSET)
#define KINETIS_CAN0_RXMGMASK      (KINETIS_CAN0_BASE+KINETIS_CAN_RXMGMASK_OFFSET)
#define KINETIS_CAN0_RX14MASK      (KINETIS_CAN0_BASE+KINETIS_CAN_RX14MASK_OFFSET)
#define KINETIS_CAN0_RX15MASK      (KINETIS_CAN0_BASE+KINETIS_CAN_RX15MASK_OFFSET)
#define KINETIS_CAN0_ECR           (KINETIS_CAN0_BASE+KINETIS_CAN_ECR_OFFSET)
#define KINETIS_CAN0_ESR1          (KINETIS_CAN0_BASE+KINETIS_CAN_ESR1_OFFSET)
#define KINETIS_CAN0_IMASK2        (KINETIS_CAN0_BASE+KINETIS_CAN_IMASK2_OFFSET)
#define KINETIS_CAN0_IMASK1        (KINETIS_CAN0_BASE+KINETIS_CAN_IMASK1_OFFSET)
#define KINETIS_CAN0_IFLAG2        (KINETIS_CAN0_BASE+KINETIS_CAN_IFLAG2_OFFSET)
#define KINETIS_CAN0_IFLAG1        (KINETIS_CAN0_BASE+KINETIS_CAN_IFLAG1_OFFSET)
#define KINETIS_CAN0_CTRL2         (KINETIS_CAN0_BASE+KINETIS_CAN_CTRL2_OFFSET)
#define KINETIS_CAN0_ESR2          (KINETIS_CAN0_BASE+KINETIS_CAN_ESR2_OFFSET)
#define KINETIS_CAN0_CRCR          (KINETIS_CAN0_BASE+KINETIS_CAN_CRCR_OFFSET)
#define KINETIS_CAN0_RXFGMASK      (KINETIS_CAN0_BASE+KINETIS_CAN_RXFGMASK_OFFSET)
#define KINETIS_CAN0_RXFIR         (KINETIS_CAN0_BASE+KINETIS_CAN_RXFIR_OFFSET)

#define KINETIS_CAN0_RXIMR(n)      (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR_OFFSET(n))
#define KINETIS_CAN0_RXIMR0        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR0_OFFSET)
#define KINETIS_CAN0_RXIMR1        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR1_OFFSET)
#define KINETIS_CAN0_RXIMR2        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR2_OFFSET)
#define KINETIS_CAN0_RXIMR3        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR3_OFFSET)
#define KINETIS_CAN0_RXIMR4        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR4_OFFSET)
#define KINETIS_CAN0_RXIMR5        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR5_OFFSET)
#define KINETIS_CAN0_RXIMR6        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR6_OFFSET)
#define KINETIS_CAN0_RXIMR7        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR7_OFFSET)
#define KINETIS_CAN0_RXIMR8        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR8_OFFSET)
#define KINETIS_CAN0_RXIMR9        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR9_OFFSET)
#define KINETIS_CAN0_RXIMR10       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR10_OFFSET)
#define KINETIS_CAN0_RXIMR11       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR11_OFFSET)
#define KINETIS_CAN0_RXIMR12       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR12_OFFSET)
#define KINETIS_CAN0_RXIMR13       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR13_OFFSET)
#define KINETIS_CAN0_RXIMR14       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR14_OFFSET)
#define KINETIS_CAN0_RXIMR15       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR15_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Module Configuration Register */
#define CAN_MCR_
/* Control 1 Register */
#define CAN_CTRL1_
/* Free Running Timer */
#define CAN_TIMER_
/* Rx Mailboxes Global Mask Register */
#define CAN_RXMGMASK_
/* Rx 14 Mask Register */
#define CAN_RX14MASK_
/* Rx 15 Mask Register */
#define CAN_RX15MASK_
/* Error Counter */
#define CAN_ECR_
/* Error and Status 1 Register */
#define CAN_ESR1_
/* Interrupt Masks 2 Register */
#define CAN_IMASK2_
/* Interrupt Masks 1 Register */
#define CAN_IMASK1_
/* Interrupt Flags 2 Register */
#define CAN_IFLAG2_
/* Interrupt Flags 1 Register */
#define CAN_IFLAG1_
/* Control 2 Register */
#define CAN_CTRL2_
/* Error and Status 2 Register */
#define CAN_ESR2_
/* CRC Register */
#define CAN_CRCR_
/* Rx FIFO Global Mask Register */
#define CAN_RXFGMASK_
/* Rx FIFO Information Register */
#define CAN_RXFIR_

/* Rn Individual Mask Registers */
#define CAN_RXIMR_

                (1 << nn)  /* Bit nn:  
_SHIFT          (nn)       /* Bits nn-nn: 
_MASK           (nn << nn)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_FLEXCAN_H */
