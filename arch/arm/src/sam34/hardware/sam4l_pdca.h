/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam4l_pdca.h
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_PDCA_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_PDCA_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* PDCA channel offsets *****************************************************************/

#define SAM_PDCA_CHAN_OFFSET(n) ((n) << 6)
#define SAM_PDCA_CHAN0_OFFSET    0x0000
#define SAM_PDCA_CHAN1_OFFSET    0x0040
#define SAM_PDCA_CHAN2_OFFSET    0x0080
#define SAM_PDCA_CHAN3_OFFSET    0x00c0
#define SAM_PDCA_CHAN4_OFFSET    0x0100
#define SAM_PDCA_CHAN5_OFFSET    0x0140
#define SAM_PDCA_CHAN6_OFFSET    0x0180
#define SAM_PDCA_CHAN7_OFFSET    0x01c0
#define SAM_PDCA_CHAN8_OFFSET    0x0200
#define SAM_PDCA_CHAN9_OFFSET    0x0240
#define SAM_PDCA_CHAN10_OFFSET   0x0280
#define SAM_PDCA_CHAN11_OFFSET   0x02c0
#define SAM_PDCA_CHAN12_OFFSET   0x0300
#define SAM_PDCA_CHAN13_OFFSET   0x0340
#define SAM_PDCA_CHAN14_OFFSET   0x0380
#define SAM_PDCA_CHAN15_OFFSET   0x03c0

/* PDCA register offsets ****************************************************************/
/* Channel register offsets */

#define SAM_PDCA_MAR_OFFSET      0x0000 /* Memory Address Register */
#define SAM_PDCA_PSR_OFFSET      0x0004 /* Peripheral Select Register */
#define SAM_PDCA_TCR_OFFSET      0x0008 /* Transfer Counter Register */
#define SAM_PDCA_MARR_OFFSET     0x000c /* Memory Address Reload Register */
#define SAM_PDCA_TCRR_OFFSET     0x0010 /* Transfer Counter Reload Register */
#define SAM_PDCA_CR_OFFSET       0x0014 /* Control Register */
#define SAM_PDCA_MR_OFFSET       0x0018 /* Mode Register */
#define SAM_PDCA_SR_OFFSET       0x001c /* Status Register */
#define SAM_PDCA_IER_OFFSET      0x0020 /* Interrupt Enable Register */
#define SAM_PDCA_IDR_OFFSET      0x0024 /* Interrupt Disable Register */
#define SAM_PDCA_IMR_OFFSET      0x0028 /* Interrupt Mask Register */
#define SAM_PDCA_ISR_OFFSET      0x002c /* Interrupt Status Register */

/* Global register offsets */

#define SAM_PDCA_VERSION_OFFSET  0x834 /* Version Register */

/* PDCA channel addresses ***************************************************************/
/* Channel register base addresses */

#define SAM_PDCA_CHAN(n)         (SAM_PDCA_BASE+SAM_PDCA_CHAN_OFFSET(n))
#define SAM_PDCA_CHAN0           (SAM_PDCA_BASE+SAM_PDCA_CHAN0_OFFSET)
#define SAM_PDCA_CHAN1           (SAM_PDCA_BASE+SAM_PDCA_CHAN1_OFFSET)
#define SAM_PDCA_CHAN2           (SAM_PDCA_BASE+SAM_PDCA_CHAN2_OFFSET)
#define SAM_PDCA_CHAN3           (SAM_PDCA_BASE+SAM_PDCA_CHAN3_OFFSET)
#define SAM_PDCA_CHAN4           (SAM_PDCA_BASE+SAM_PDCA_CHAN4_OFFSET)
#define SAM_PDCA_CHAN5           (SAM_PDCA_BASE+SAM_PDCA_CHAN5_OFFSET)
#define SAM_PDCA_CHAN6           (SAM_PDCA_BASE+SAM_PDCA_CHAN6_OFFSET)
#define SAM_PDCA_CHAN7           (SAM_PDCA_BASE+SAM_PDCA_CHAN7_OFFSET)
#define SAM_PDCA_CHAN8           (SAM_PDCA_BASE+SAM_PDCA_CHAN8_OFFSET)
#define SAM_PDCA_CHAN9           (SAM_PDCA_BASE+SAM_PDCA_CHAN9_OFFSET)
#define SAM_PDCA_CHAN10          (SAM_PDCA_BASE+SAM_PDCA_CHAN10_OFFSET)
#define SAM_PDCA_CHAN11          (SAM_PDCA_BASE+SAM_PDCA_CHAN11_OFFSET)
#define SAM_PDCA_CHAN12          (SAM_PDCA_BASE+SAM_PDCA_CHAN12_OFFSET)
#define SAM_PDCA_CHAN13          (SAM_PDCA_BASE+SAM_PDCA_CHAN13_OFFSET)
#define SAM_PDCA_CHAN14          (SAM_PDCA_BASE+SAM_PDCA_CHAN14_OFFSET)
#define SAM_PDCA_CHAN15          (SAM_PDCA_BASE+SAM_PDCA_CHAN15_OFFSET)

/* PDCA register addresses **************************************************************/
/* Channel register addresses */

#define SAM_PDCA_MAR(n)          (SAM_PDCA_CHAN(n)+SAM_PDCA_MAR_OFFSET)
#define SAM_PDCA_PSR(n)          (SAM_PDCA_CHAN(n)+SAM_PDCA_PSR_OFFSET)
#define SAM_PDCA_TCR(n)          (SAM_PDCA_CHAN(n)+SAM_PDCA_TCR_OFFSET)
#define SAM_PDCA_MARR(n)         (SAM_PDCA_CHAN(n)+SAM_PDCA_MARR_OFFSET)
#define SAM_PDCA_TCRR(n)         (SAM_PDCA_CHAN(n)+SAM_PDCA_TCRR_OFFSET)
#define SAM_PDCA_CR(n)           (SAM_PDCA_CHAN(n)+SAM_PDCA_CR_OFFSET)
#define SAM_PDCA_MR(n)           (SAM_PDCA_CHAN(n)+SAM_PDCA_MR_OFFSET)
#define SAM_PDCA_SR(n)           (SAM_PDCA_CHAN(n)+SAM_PDCA_SR_OFFSET)
#define SAM_PDCA_IER(n)          (SAM_PDCA_CHAN(n)+SAM_PDCA_IER_OFFSET)
#define SAM_PDCA_IDR(n)          (SAM_PDCA_CHAN(n)+SAM_PDCA_IDR_OFFSET)
#define SAM_PDCA_IMR(n)          (SAM_PDCA_CHAN(n)+SAM_PDCA_IMR_OFFSET)
#define SAM_PDCA_ISR(n)          (SAM_PDCA_CHAN(n)+SAM_PDCA_ISR_OFFSET)

/* Global register offsets */

#define SAM_PDCA_VERSION         (SAM_PDCA_BASE+SAM_PDCA_VERSION_OFFSET)

/* PDCA register bit definitions ********************************************************/

/* Memory Address Register (32-bit address) */

/* Peripheral Select Register */

#define PDCA_PSR_MASK            0xff   /* Bit 0-7: Peripheral identifier */

/* Transfer Counter Register */

#define PDCA_TCR_MASK            0xffff /* Bits 0-15: Transfer Counter Value

/* Memory Address Reload Register (32-bit address) */

/* Transfer Counter Reload Register */

#define PDCA_TCRR_MASK           0xffff /* Bits 0-15: Transfer Counter Reload Value */

/* Control Register */

#define PDCA_CR_TEN              (1 << 0)  /* Bit 0:  Transfer Enable */
#define PDCA_CR_TDIS             (1 << 1)  /* Bit 1:  Transfer Disable */
#define PDCA_CR_ECLR             (1 << 8)  /* Bit 8:  Transfer Error Clear */

/* Mode Register */

#define PDCA_MR_SIZE_SHIFT       (0)       /* Bits 0-1: Size of Transfer */
#define PDCA_MR_SIZE_MASK        (3 << PDCA_MR_SIZE_SHIFT)
#  define PDCA_MR_SIZE_BYTE      (0 << PDCA_MR_SIZE_SHIFT)
#  define PDCA_MR_SIZE_HWORD     (1 << PDCA_MR_SIZE_SHIFT)
#  define PDCA_MR_SIZE_WORD      (2 << PDCA_MR_SIZE_SHIFT)
#define PDCA_MR_ETRIG            (1 << 2)  /* Bit 2:  Event Trigger */
#define PDCA_MR_RING             (1 << 3)  /* Bit 3:  Ring Buffer */

/* Status Register */

#define PDCA_SR_TEN              (1 << 0)  /* Bit 0:  Transfer Enabled */

/* Interrupt Enable Register */

#define PDCA_IER_

/* Interrupt Disable Register */
/* Interrupt Mask Register */
/* Interrupt Status Register */

#define PDCA_INT_RCZ             (1 << 2)  /* Bit 0:  Reload Counter Zero */
#define PDCA_INT_TRC             (1 << 2)  /* Bit 1:  Transfer Complete */
#define PDCA_INT_TERR            (1 << 2)  /* Bit 2:  Transfer Error */

/* Global register offsets */

/* Version Register */

#define PDCA_VERSION_SHIFT       (0)        /* Bits 0-11: Version Number */
#define PDCA_VERSION_MASK        (0xfff << PDCA_VERSION_VERSION_SHIFT)
#define PDCA_VARIANT_SHIFT       (16)       /* Bits 16-19: Variant Number */
#define PDCA_VARIANT_MASK        (15 << PDCA_VARIANT_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_PDCA_H */
