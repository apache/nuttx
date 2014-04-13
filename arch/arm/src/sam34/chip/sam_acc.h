/****************************************************************************************
 * arch/arm/src/sam34/chip/sam_rtt.h
 *Analog Comparator Controller (ACC) definitions for the SAM4E
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM_ACC_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM_ACC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* ACC register offsets *****************************************************************/

#define SAM_ACC_CR_OFFSET          0x0000 /* Control Register */
#define SAM_ACC_MR_OFFSET          0x0004 /* Mode Register */
#define SAM_ACC_IER_OFFSET         0x0024 /* Interrupt Enable Register */
#define SAM_ACC_IDR_OFFSET         0x0028 /* Interrupt Disable Register */
#define SAM_ACC_IMR_OFFSET         0x002c /* Interrupt Mask Register */
#define SAM_ACC_ISR_OFFSET         0x0030 /* Interrupt Status Register */
#define SAM_ACC_ACR_OFFSET         0x0094 /* Analog Control Register */
#define SAM_ACC_WPMR_OFFSET        0x00e4 /* Write Protect Mode Register */
#define SAM_ACC_WPSR_OFFSET        0x00e8 /* Write Protect Status Register */

/* ACC register addresses **************************************************************/

#define SAM_ACC_CR                 (SAM_ACC_BASE+SAM_ACC_CR_OFFSET)
#define SAM_ACC_MR                 (SAM_ACC_BASE+SAM_ACC_MR_OFFSET)
#define SAM_ACC_IER                (SAM_ACC_BASE+SAM_ACC_IER_OFFSET)
#define SAM_ACC_IDR                (SAM_ACC_BASE+SAM_ACC_IDR_OFFSET)
#define SAM_ACC_IMR                (SAM_ACC_BASE+SAM_ACC_IMR_OFFSET)
#define SAM_ACC_ISR                (SAM_ACC_BASE+SAM_ACC_ISR_OFFSET)
#define SAM_ACC_ACR                (SAM_ACC_BASE+SAM_ACC_ACR_OFFSET
#define SAM_ACC_WPMR               (SAM_ACC_BASE+SAM_ACC_WPMR_OFFSET)
#define SAM_ACC_WPSR               (SAM_ACC_BASE+SAM_ACC_WPSR_OFFSET

/* ACC register bit definitions ********************************************************/

/* Control Register */

#define ACC_CR_SWRST               (1 << 0)  /* Bit 0:  Software reset */

/* Mode Register */

#define ACC_MR_SELMINUS_SHIFT      (0)       /* Bits 0-2: Selection for minus comparator input */
#define ACC_MR_SELMINUS_MASK       (7 << ACC_MR_SELMINUS_SHIFT)
#  define ACC_MR_SELMINUS_TS       (0 << ACC_MR_SELMINUS_SHIFT) /* Select TS */
#  define ACC_MR_SELMINUS_ADVREF   (1 << ACC_MR_SELMINUS_SHIFT) /* Select ADVREF */
#  define ACC_MR_SELMINUS_DAC0     (2 << ACC_MR_SELMINUS_SHIFT) /* Select DAC0 */
#  define ACC_MR_SELMINUS_DAC1     (3 << ACC_MR_SELMINUS_SHIFT) /* Select DAC1 */
#  define ACC_MR_SELMINUS_AD0      (4 << ACC_MR_SELMINUS_SHIFT) /* Select AD0 */
#  define ACC_MR_SELMINUS_AD1      (5 << ACC_MR_SELMINUS_SHIFT) /* Select AD1 */
#  define ACC_MR_SELMINUS_AD2      (6 << ACC_MR_SELMINUS_SHIFT) /* Select AD2 */
#  define ACC_MR_SELMINUS_AD3      (7 << ACC_MR_SELMINUS_SHIFT) /* Select AD3 */
#define ACC_MR_SELPLUS_SHIFT       (4)       /* Bits 4-6: Selection for plus comparator input */
#define ACC_MR_SELPLUS_MASK        (7 << ACC_MR_SELPLUS_SHIFT)
#  define ACC_MR_SELPLUS_AD(n)     ((uint32_t)(n) << ACC_MR_SELPLUS_SHIFT) /* Select ADn, n=0-7 */
#  define ACC_MR_SELPLUS_AD0       (0 << ACC_MR_SELPLUS_SHIFT) /* Select AD0 */
#  define ACC_MR_SELPLUS_AD1       (1 << ACC_MR_SELPLUS_SHIFT) /* Select AD1 */
#  define ACC_MR_SELPLUS_AD2       (2 << ACC_MR_SELPLUS_SHIFT) /* Select AD2 */
#  define ACC_MR_SELPLUS_AD3       (3 << ACC_MR_SELPLUS_SHIFT) /* Select AD3 */
#  define ACC_MR_SELPLUS_AD4       (4 << ACC_MR_SELPLUS_SHIFT) /* Select AD4 */
#  define ACC_MR_SELPLUS_AD5       (5 << ACC_MR_SELPLUS_SHIFT) /* Select AD5 */
#  define ACC_MR_SELPLUS_AD6       (6 << ACC_MR_SELPLUS_SHIFT) /* Select AD6 */
#  define ACC_MR_SELPLUS_AD7       (7 << ACC_MR_SELPLUS_SHIFT) /* Select AD7 */
#define ACC_MR_ACEN                (1 << 8)  /* Bit 8:  Analog comparator enable */
#define ACC_MR_EDGETYP_SHIFT       (9)       /* Bits 9-10: Edge type */
#define ACC_MR_EDGETYP_MASK        (3 << ACC_MR_EDGETYP_SHIFT)
#  define ACC_MR_EDGETYP_RISING    (0 << ACC_MR_EDGETYP_SHIFT) /* Only rising edge of comparator output */
#  define ACC_MR_EDGETYP_FALLING   (1 << ACC_MR_EDGETYP_SHIFT) /* Falling edge of comparator output */
#  define ACC_MR_EDGETYP_ANY       (2 << ACC_MR_EDGETYP_SHIFT) /* Any edge of comparator output */
#define ACC_MR_INV                 (1 << 12) /* Bit 12: Invert comparator output */
#define ACC_MR_SELFS               (1 << 13) /* Bit 13: Selection of fault source */
#define ACC_MR_FE                  (1 << 14) /* Bit 14: Fault enable */

/* Interrupt Enable, Interrupt Disable, Interrupt Mask, and  Interrupt Status */

#define ACC_INT_CE                 (1 << 0)  /* Bit 0:  Comparison edge interrupt */

/* Interrupt Status Register (only) */

#define ACC_ISR_SCO                (1 << 1)  /* Bit 1: Synchronized Comparator Output */
#define ACC_ISR_MASK               (1 << 31)

/* Analog Control Register */

#define ACC_ACR_ISEL               (1 << 0)  /* Bit 0:  Current selection */
#define ACC_ACR_HYST_SHIFT         (1)       /* Bits 1-2: Hysteresis selection */
#define ACC_ACR_HYST_MASK          (3 << ACC_ACR_HYST_SHIFT)

/* Write Protect Mode Register */

#define ACC_WPMR_WPEN              (1 << 0)  /* Bit 0:  Write Protect Enable */
#define ACC_WPMR_WPKEY_SHIFT       (8)       /* Bits 8-31: Write Protect KEY */
#define ACC_WPMR_WPKEY_MASK        (0x00ffffff << ACC_WPMR_WPKEY_SHIFT)
#  define ACC_WPMR_WPKEY           (0x00414343 << ACC_WPMR_WPKEY_SHIFT)

/* Write Protect Status Register */

#define ACC_WPSR_WPROTERR          (1 << 0)  /* Bit 0:  Write protection error */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM_ACC_H */
