/************************************************************************************
 * arch/arm/src/lpc11xx/chip/lpc11_timer.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_TIMER_H
#define __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc11_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC11_TMR_IR_OFFSET       0x0000 /* Interrupt Register */
#define LPC11_TMR_TCR_OFFSET      0x0004 /* Timer Control Register */
#define LPC11_TMR_TC_OFFSET       0x0008 /* Timer Counter */
#define LPC11_TMR_PR_OFFSET       0x000c /* Prescale Register */
#define LPC11_TMR_PC_OFFSET       0x0010 /* Prescale Counter */
#define LPC11_TMR_MCR_OFFSET      0x0014 /* Match Control Register */
#define LPC11_TMR_MR0_OFFSET      0x0018 /* Match Register 0 */
#define LPC11_TMR_MR1_OFFSET      0x001c /* Match Register 1 */
#define LPC11_TMR_MR2_OFFSET      0x0020 /* Match Register 2 */
#define LPC11_TMR_MR3_OFFSET      0x0024 /* Match Register 3 */
#define LPC11_TMR_CCR_OFFSET      0x0028 /* Capture Control Register */
#define LPC11_TMR_CR0_OFFSET      0x002c /* Capture Register 0 */
#define LPC11_TMR_CR1_OFFSET      0x0030 /* Capture Register 1 */
#define LPC11_TMR_EMR_OFFSET      0x003c /* External Match Register */
#define LPC11_TMR_CTCR_OFFSET     0x0070 /* Count Control Register */
#define LPC11_TMR_PWMC_OFFSET     0x0074 /* PWM Control Register */

/* Register addresses ***************************************************************/

#define LPC11_TMR16B0IR           (LPC11_TMR16B0_BASE+LPC11_TMR_IR_OFFSET)
#define LPC11_TMR16B0TCR          (LPC11_TMR16B0_BASE+LPC11_TMR_TCR_OFFSET)
#define LPC11_TMR16B0TC           (LPC11_TMR16B0_BASE+LPC11_TMR_TC_OFFSET)
#define LPC11_TMR16B0PR           (LPC11_TMR16B0_BASE+LPC11_TMR_PR_OFFSET)
#define LPC11_TMR16B0PC           (LPC11_TMR16B0_BASE+LPC11_TMR_PC_OFFSET)
#define LPC11_TMR16B0MCR          (LPC11_TMR16B0_BASE+LPC11_TMR_MCR_OFFSET)
#define LPC11_TMR16B0MR0          (LPC11_TMR16B0_BASE+LPC11_TMR_MR0_OFFSET)
#define LPC11_TMR16B0MR1          (LPC11_TMR16B0_BASE+LPC11_TMR_MR1_OFFSET)
#define LPC11_TMR16B0MR2          (LPC11_TMR16B0_BASE+LPC11_TMR_MR2_OFFSET)
#define LPC11_TMR16B0MR3          (LPC11_TMR16B0_BASE+LPC11_TMR_MR3_OFFSET)
#define LPC11_TMR16B0CCR          (LPC11_TMR16B0_BASE+LPC11_TMR_CCR_OFFSET)
#define LPC11_TMR16B0CR0          (LPC11_TMR16B0_BASE+LPC11_TMR_CR0_OFFSET)
#define LPC11_TMR16B0CR1          (LPC11_TMR16B0_BASE+LPC11_TMR_CR1_OFFSET)
#define LPC11_TMR16B0EMR          (LPC11_TMR16B0_BASE+LPC11_TMR_EMR_OFFSET)
#define LPC11_TMR16B0CTCR         (LPC11_TMR16B0_BASE+LPC11_TMR_CTCR_OFFSET)
#define LPC11_TMR16B0PWMC         (LPC11_TMR16B0_BASE+LPC11_TMR_PWMC_OFFSET)

#define LPC11_TMR16B1IR           (LPC11_TMR16B1_BASE+LPC11_TMR_IR_OFFSET)
#define LPC11_TMR16B1TCR          (LPC11_TMR16B1_BASE+LPC11_TMR_TCR_OFFSET)
#define LPC11_TMR16B1TC           (LPC11_TMR16B1_BASE+LPC11_TMR_TC_OFFSET)
#define LPC11_TMR16B1PR           (LPC11_TMR16B1_BASE+LPC11_TMR_PR_OFFSET)
#define LPC11_TMR16B1PC           (LPC11_TMR16B1_BASE+LPC11_TMR_PC_OFFSET)
#define LPC11_TMR16B1MCR          (LPC11_TMR16B1_BASE+LPC11_TMR_MCR_OFFSET)
#define LPC11_TMR16B1MR0          (LPC11_TMR16B1_BASE+LPC11_TMR_MR0_OFFSET)
#define LPC11_TMR16B1MR1          (LPC11_TMR16B1_BASE+LPC11_TMR_MR1_OFFSET)
#define LPC11_TMR16B1MR2          (LPC11_TMR16B1_BASE+LPC11_TMR_MR2_OFFSET)
#define LPC11_TMR16B1MR3          (LPC11_TMR16B1_BASE+LPC11_TMR_MR3_OFFSET)
#define LPC11_TMR16B1CCR          (LPC11_TMR16B1_BASE+LPC11_TMR_CCR_OFFSET)
#define LPC11_TMR16B1CR0          (LPC11_TMR16B1_BASE+LPC11_TMR_CR0_OFFSET)
#define LPC11_TMR16B1CR1          (LPC11_TMR16B1_BASE+LPC11_TMR_CR1_OFFSET)
#define LPC11_TMR16B1EMR          (LPC11_TMR16B1_BASE+LPC11_TMR_EMR_OFFSET)
#define LPC11_TMR16B1CTCR         (LPC11_TMR16B1_BASE+LPC11_TMR_CTCR_OFFSET)
#define LPC11_TMR16B1PWMC         (LPC11_TMR16B1_BASE+LPC11_TMR_PWMC_OFFSET)

#define LPC11_TMR32B0IR           (LPC11_TMR32B0_BASE+LPC11_TMR_IR_OFFSET)
#define LPC11_TMR32B0TCR          (LPC11_TMR32B0_BASE+LPC11_TMR_TCR_OFFSET)
#define LPC11_TMR32B0TC           (LPC11_TMR32B0_BASE+LPC11_TMR_TC_OFFSET)
#define LPC11_TMR32B0PR           (LPC11_TMR32B0_BASE+LPC11_TMR_PR_OFFSET)
#define LPC11_TMR32B0PC           (LPC11_TMR32B0_BASE+LPC11_TMR_PC_OFFSET)
#define LPC11_TMR32B0MCR          (LPC11_TMR32B0_BASE+LPC11_TMR_MCR_OFFSET)
#define LPC11_TMR32B0MR0          (LPC11_TMR32B0_BASE+LPC11_TMR_MR0_OFFSET)
#define LPC11_TMR32B0MR1          (LPC11_TMR32B0_BASE+LPC11_TMR_MR1_OFFSET)
#define LPC11_TMR32B0MR2          (LPC11_TMR32B0_BASE+LPC11_TMR_MR2_OFFSET)
#define LPC11_TMR32B0MR3          (LPC11_TMR32B0_BASE+LPC11_TMR_MR3_OFFSET)
#define LPC11_TMR32B0CCR          (LPC11_TMR32B0_BASE+LPC11_TMR_CCR_OFFSET)
#define LPC11_TMR32B0CR0          (LPC11_TMR32B0_BASE+LPC11_TMR_CR0_OFFSET)
#define LPC11_TMR32B0CR1          (LPC11_TMR32B0_BASE+LPC11_TMR_CR1_OFFSET)
#define LPC11_TMR32B0EMR          (LPC11_TMR32B0_BASE+LPC11_TMR_EMR_OFFSET)
#define LPC11_TMR32B0CTCR         (LPC11_TMR32B0_BASE+LPC11_TMR_CTCR_OFFSET)
#define LPC11_TMR32B0PWMC         (LPC11_TMR32B0_BASE+LPC11_TMR_PWMC_OFFSET)

#define LPC11_TMR32B1IR           (LPC11_TMR32B1_BASE+LPC11_TMR_IR_OFFSET)
#define LPC11_TMR32B1TCR          (LPC11_TMR32B1_BASE+LPC11_TMR_TCR_OFFSET)
#define LPC11_TMR32B1TC           (LPC11_TMR32B1_BASE+LPC11_TMR_TC_OFFSET)
#define LPC11_TMR32B1PR           (LPC11_TMR32B1_BASE+LPC11_TMR_PR_OFFSET)
#define LPC11_TMR32B1PC           (LPC11_TMR32B1_BASE+LPC11_TMR_PC_OFFSET)
#define LPC11_TMR32B1MCR          (LPC11_TMR32B1_BASE+LPC11_TMR_MCR_OFFSET)
#define LPC11_TMR32B1MR0          (LPC11_TMR32B1_BASE+LPC11_TMR_MR0_OFFSET)
#define LPC11_TMR32B1MR1          (LPC11_TMR32B1_BASE+LPC11_TMR_MR1_OFFSET)
#define LPC11_TMR32B1MR2          (LPC11_TMR32B1_BASE+LPC11_TMR_MR2_OFFSET)
#define LPC11_TMR32B1MR3          (LPC11_TMR32B1_BASE+LPC11_TMR_MR3_OFFSET)
#define LPC11_TMR32B1CCR          (LPC11_TMR32B1_BASE+LPC11_TMR_CCR_OFFSET)
#define LPC11_TMR32B1CR0          (LPC11_TMR32B1_BASE+LPC11_TMR_CR0_OFFSET)
#define LPC11_TMR32B1CR1          (LPC11_TMR32B1_BASE+LPC11_TMR_CR1_OFFSET)
#define LPC11_TMR32B1EMR          (LPC11_TMR32B1_BASE+LPC11_TMR_EMR_OFFSET)
#define LPC11_TMR32B1CTCR         (LPC11_TMR32B1_BASE+LPC11_TMR_CTCR_OFFSET)
#define LPC11_TMR32B1PWMC         (LPC11_TMR32B1_BASE+LPC11_TMR_PWMC_OFFSET)


/* Register bit definitions *********************************************************/
/* Registers holding 32-bit numeric values (no bit field definitions):
 *
 *   Timer Counter (TC)
 *   Prescale Register (PR)
 *   Prescale Counter (PC)
 *   Match Register 0 (MR0)
 *   Match Register 1 (MR1)
 *   Match Register 2 (MR2)
 *   Match Register 3 (MR3)
 *   Capture Register 0 (CR0)
 *   Capture Register 1 (CR1)
 */

/* Interrupt Register */

#define TMR_MR0INT                (1 << 0)  /* Bit 0:  Match channel 0 interrupt */
#define TMR_MR1INT                (1 << 1)  /* Bit 1:  Match channel 1 interrupt */
#define TMR_MR2INT                (1 << 2)  /* Bit 2:  Match channel 2 interrupt */
#define TMR_MR3INT                (1 << 3)  /* Bit 3:  Match channel 3 interrupt */
#define TMR_CR0INT                (1 << 4)  /* Bit 4:  Capture channel 0 interrupt */
#define TMR_CR1INT                (1 << 5)  /* Bit 5:  Capture channel 1 interrupt */
                                            /* Bits 6-31: Reserved */
/* Timer Control Register */

#define TMR_TCR_CEN               (1 << 0)  /* Bit 0:  Counter Enable */
#define TMR_TCR_CRST              (1 << 1)  /* Bit 1:  Counter Reset */
                                            /* Bits 2-31: Reserved */
/* Match Control Register */

#define TMR_MCR_MR0I              (1 << 0)  /* Bit 0:  Interrupt on MR0 */
#define TMR_MCR_MR0R              (1 << 1)  /* Bit 1:  Reset on MR0 */
#define TMR_MCR_MR0S              (1 << 2)  /* Bit 2:  Stop on MR0 */
#define TMR_MCR_MR1I              (1 << 3)  /* Bit 3:  Interrupt on MR1 */
#define TMR_MCR_MR1R              (1 << 4)  /* Bit 4:  Reset on MR1 */
#define TMR_MCR_MR1S              (1 << 5)  /* Bit 5:  Stop on MR1 */
#define TMR_MCR_MR2I              (1 << 6)  /* Bit 6:  Interrupt on MR2 */
#define TMR_MCR_MR2R              (1 << 7)  /* Bit 7:  Reset on MR2 */
#define TMR_MCR_MR2S              (1 << 8)  /* Bit 8:  Stop on MR2 */
#define TMR_MCR_MR3I              (1 << 9)  /* Bit 9:  Interrupt on MR3 */
#define TMR_MCR_MR3R              (1 << 10) /* Bit 10: Reset on MR3 */
#define TMR_MCR_MR3S              (1 << 11) /* Bit 11: Stop on MR3 */
                                            /* Bits 12-31: Reserved */
/* Capture Control Register */

#define TMR_CCR_CAP0RE            (1 << 0)  /* Bit 0: Capture on CAPn.0 rising edge */
#define TMR_CCR_CAP0FE            (1 << 1)  /* Bit 1: Capture on CAPn.0 falling edge */
#define TMR_CCR_CAP0I             (1 << 2)  /* Bit 2: Interrupt on CAPn.0 */
#define TMR_CCR_CAP1RE            (1 << 3)  /* Bit 3: Capture on CAPn.1 rising edge */
#define TMR_CCR_CAP1FE            (1 << 4)  /* Bit 4: Capture on CAPn.1 falling edge */
#define TMR_CCR_CAP1I             (1 << 5)  /* Bit 5: Interrupt on CAPn.1 */
                                            /* Bits 6-31: Reserved */
/* External Match Register */

#define TMR_EMR_NOTHING           (0)       /* Do Nothing */
#define TMR_EMR_CLEAR             (1)       /* Clear external match bit MATn.m */
#define TMR_EMR_SET               (2)       /* Set external match bit MATn.m */
#define TMR_EMR_TOGGLE            (3)       /* Toggle external match bit MATn.m */

#define TMR_EMR_EM0               (1 << 0)  /* Bit 0:  External Match 0 */
#define TMR_EMR_EM1               (1 << 1)  /* Bit 1:  External Match 1 */
#define TMR_EMR_EM2               (1 << 2)  /* Bit 2:  External Match 2 */
#define TMR_EMR_EM3               (1 << 3)  /* Bit 3:  External Match 3 */
#define TMR_EMR_EMC0_SHIFT        (4)       /* Bits 4-5: External Match Control 0 */
#define TMR_EMR_EMC0_MASK         (3 << TMR_EMR_EMC0_SHIFTy)
#  define TMR_EMR_EMC0_NOTHING    (TMR_EMR_NOTHING << TMR_EMR_EMC0_SHIFT)
#  define TMR_EMR_EMC0_CLEAR      (TMR_EMR_CLEAR << TMR_EMR_EMC0_SHIFT)
#  define TMR_EMR_EMC0_SET        (TMR_EMR_SET << TMR_EMR_EMC0_SHIFT)
#  define TMR_EMR_EMC0_TOGGLE     (TMR_EMR_TOGGLE << TMR_EMR_EMC0_SHIFT)
#define TMR_EMR_EMC1_SHIFT        (6)       /* Bits 6-7: External Match Control 1 */
#define TMR_EMR_EMC1_MASK         (3 << TMR_EMR_EMC1_SHIFT)
#  define TMR_EMR_EMC1_NOTHING    (TMR_EMR_NOTHING << TMR_EMR_EMC1_SHIFT)
#  define TMR_EMR_EMC1_CLEAR      (TMR_EMR_CLEAR << TMR_EMR_EMC1_SHIFT)
#  define TMR_EMR_EMC1_SET        (TMR_EMR_SET << TMR_EMR_EMC1_SHIFT)
#  define TMR_EMR_EMC1_TOGGLE     (TMR_EMR_TOGGLE << TMR_EMR_EMC1_SHIFT)
#define TMR_EMR_EMC2_SHIFT        (8)       /* Bits 8-9: External Match Control 2 */
#define TMR_EMR_EMC2_MASK         (3 << TMR_EMR_EMC2_SHIFT)
#  define TMR_EMR_EMC2_NOTHING    (TMR_EMR_NOTHING << TMR_EMR_EMC2_SHIFT)
#  define TMR_EMR_EMC2_CLEAR      (TMR_EMR_CLEAR << TMR_EMR_EMC2_SHIFT)
#  define TMR_EMR_EMC2_SET        (TMR_EMR_SET << TMR_EMR_EMC2_SHIFT)
#  define TMR_EMR_EMC2_TOGGLE     (TMR_EMR_TOGGLE << TMR_EMR_EMC2_SHIFT)
#define TMR_EMR_EMC3_SHIFT        (10)      /* Bits 10-11: External Match Control 3 */
#define TMR_EMR_EMC3_MASK         (3 << TMR_EMR_EMC3_SHIFT)
#  define TMR_EMR_EMC3_NOTHING    (TMR_EMR_NOTHING << TMR_EMR_EMC3_SHIFT)
#  define TMR_EMR_EMC3_CLEAR      (TMR_EMR_CLEAR << TMR_EMR_EMC3_SHIFT)
#  define TMR_EMR_EMC3_SET        (TMR_EMR_SET << TMR_EMR_EMC3_SHIFT)
#  define TMR_EMR_EMC3_TOGGLE     (TMR_EMR_TOGGLE << TMR_EMR_EMC3_SHIFT)
                                            /* Bits 12-31: Reserved */
/* Count Control Register */

#define TMR_CTCR_MODE_SHIFT       (0)       /* Bits 0-1: Counter/Timer Mode */
#define TMR_CTCR_MODE_MASK        (3 << TMR_CTCR_MODE_SHIFT)
#  define TMR_CTCR_MODE_TIMER     (0 << TMR_CTCR_MODE_SHIFT) /* Timer Mode, prescale match */
#  define TMR_CTCR_MODE_CNTRRE    (1 << TMR_CTCR_MODE_SHIFT) /* Counter Mode, CAP rising edge */
#  define TMR_CTCR_MODE_CNTRFE    (2 << TMR_CTCR_MODE_SHIFT) /* Counter Mode, CAP falling edge */
#  define TMR_CTCR_MODE_CNTRBE    (3 << TMR_CTCR_MODE_SHIFT) /* Counter Mode, CAP both edges */
#define TMR_CTCR_INPSEL_SHIFT     (2)       /* Bits 2-3: Count Input Select */
#define TMR_CTCR_INPSEL_MASK      (3 << TMR_CTCR_INPSEL_SHIFT)
#  define TMR_CTCR_INPSEL_CAPNp0  (0 << TMR_CTCR_INPSEL_SHIFT) /* CAPn.0 for TIMERn */
#  define TMR_CTCR_INPSEL_CAPNp1  (1 << TMR_CTCR_INPSEL_SHIFT) /* CAPn.1 for TIMERn */
#define TMR_CTCR_ENCC             (1 << 4)  /* Enable Clear Timer/Prescale when capture event happens */
#define TMR_CTCR_SELCC_SHIFT      (5)       /* Bits 5-7: Selects which capture event will clear Timer/Prescale */
#define TMR_CTCR_SELCC_MASK       (3 << TMR_CTCR_SELCC_SHIFT)
#  define TMR_CTCR_SELCC_RECAP0   (0 << TMR_CTCR_SELCC_SHIFT) /* Rising edge CAP0 clears timer (if bit 4 is set) */
#  define TMR_CTCR_SELCC_FECAP0   (1 << TMR_CTCR_SELCC_SHIFT) /* Falling edge CAP0 clears timer (if bit 4 is set) */
#  define TMR_CTCR_SELCC_RECAP1   (2 << TMR_CTCR_SELCC_SHIFT) /* Rising edge CAP1 clears timer (if bit 4 is set) */
#  define TMR_CTCR_SELCC_FECAP1   (3 << TMR_CTCR_SELCC_SHIFT) /* Falling edge CAP1 clears timer (if bit 4 is set) */
                                            /* Bits 8-31: Reserved */

/* PWM Control register */

#define TMR_PWMC_PWMEN0           (1 << 0) /* PWM channel0 enable */
#define TMR_PWMC_PWMEN1           (1 << 1) /* PWM channel1 enable */
#define TMR_PWMC_PWMEN2           (1 << 2) /* PWM channel2 enable */
#define TMR_PWMC_PWMEN3           (1 << 3) /* PWM channel3 enable */
                                           /* Bits 4-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_TIMER_H */
