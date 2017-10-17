/****************************************************************************
 * arch/arm/include/bcm2708/irq.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_BCM2708_IRQ_H
#define __ARCH_ARM_INCLUDE_BCM2708_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/bcm2708/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Chip-Specific External interrupts */

#if defined(CONFIG_ARCH_CHIP_BCM2835)

/* Interrupt decode algorithm:
 *
 * 1) Check bits 0 through BPR_BIT_LAST in the basic pending register.  For
 *    each bit set, dispatch IRQ = bit number
 * 2) If bits set in pending register 1, check bits IPR1_BIT_FIRST through
 *    IPR1_BIT_LAST for the pending 1 register.  For each bit set, dispatch
 *    IRQ = bit number + IPR1_IRQ_FIRST - IPR1_BIT_FIRST.
 * 2) If bits set in pending register 2, check bits IPR2_BIT_FIRST through
 *    IPR2_BIT_LAST for the pending 2 register.  For each bit set, dispatch
 *    IRQ = bit number + IPR2_IRQ_FIRST - IPR2_BIT_FIRST.
 */

/* Basic pending register */

#define BPR_IRQ_FIRST            0  /* IRQ of first defined bit */
#define BPR_BIT_FIRST            0  /* First defined bit */

#define BCM_IRQ_ARM_TIMER        0  /* Bit 0:  ARM Timer IRQ pending */
#define BCM_IRQ_ARM_MAILBOX      1  /* Bit 1:  ARM Mailbox IRQ pending */
#define BCM_IRQ_ARM_DOORBELL_0   2  /* Bit 2:  ARM Doorbell 0 IRQ pending */
#define BCM_IRQ_ARM_DOORBELL_1   3  /* Bit 3:  ARM Doorbell 2 IRQ pending */
#define BCM_IRQ_GPU0_HALTED      4  /* Bit 4:  GPU0 halted IRQ pending
                                    * (Or GPU1 halted if bit 10 of control
                                    * register 1 is set) */
#define BCM_IRQ_GPU1_HALTED      5  /* Bit 5:  GPU1 halted IRQ pending */
#define BCM_IRQ_ILLEGAL_ACCESS_1 6  /* Bit 6:  Illegal access type 1 IRQ pending */
#define BCM_IRQ_ILLEGAL_ACCESS_0 7  /* Bit 7:  Illegal access type 0 IRQ pending */

#define BCM_BIT_PENDING_1        8  /* Bit 8:  Bits set in pending register 1 */
#define BCM_BIT_PENDING_2        9  /* Bit 9:  Bits set in pending register 2 */

#define BCM_IRQ_GPU_IRQ_7        10 /* Bit 10: GPU IRQ 7 */
#define BCM_IRQ_GPU_IRQ_9        11 /* Bit 10: GPU IRQ 9 */
#define BCM_IRQ_GPU_IRQ_10       12 /* Bit 10: GPU IRQ 10 */
#define BCM_IRQ_GPU_IRQ_18       13 /* Bit 10: GPU IRQ 18 */
#define BCM_IRQ_GPU_IRQ_19       14 /* Bit 10: GPU IRQ 19 */
#define BCM_IRQ_GPU_IRQ_53       15 /* Bit 10: GPU IRQ 53 */
#define BCM_IRQ_GPU_IRQ_54       16 /* Bit 10: GPU IRQ 54 */
#define BCM_IRQ_GPU_IRQ_55       17 /* Bit 10: GPU IRQ 55 */
#define BCM_IRQ_GPU_IRQ_56       18 /* Bit 10: GPU IRQ 56 */
#define BCM_IRQ_GPU_IRQ_57       19 /* Bit 10: GPU IRQ 57 */
#define BCM_IRQ_GPU_IRQ_62       20 /* Bit 10: GPU IRQ 61 */

#define BPR_BIT_IRQMASK          0x001ffcff            /* Mask of defined interrupts */
#define BPR_BIT_LAST             20                    /* IRQ of last defined bit */
#define BPR_IRQ_LAST             20                    /* Last defined bit */

/* IRQ pending 1 register */

#define IPR1_IRQ_FIRST           (BPR_IRQ_LAST + 1)    /* IRQ of first defined bit */
#define IPR1_BIT_FIRST           (29)                  /* First defined bit */

#define BCM_IRQ_AUX_INT          IPR1_IRQ_FIRST        /* Bit 29: Aux interrupt */

#define IPR1_BIT_IRQMASK         0x20000000            /* Mask of defined interrupts */
#define IPR1_IRQ_LAST            BCM_IRQ_AUX_INT       /* IRQ of last defined bit */
#define IPR1_BIT_LAST            (29)                  /* Last defined bit */

/* IRQ pending 1 register */

#define IPR2_IRQ_FIRST           (IPR1_IRQ_LAST + 1)   /* IRQ of first defined bit */
#define IPR2_BIT_FIRST           (11)                  /* First defined bit */

#define BCM_IRQ_I2C_SPI_SLV      (IPR2_IRQ_FIRST + 11) /* Bit 11: 43 I2C/SPI slave */
#define BCM_IRQ_PWA0             (IPR2_IRQ_FIRST + 13) /* Bit 12: 45 PWA0 */
#define BCM_IRQ_PWA1             (IPR2_IRQ_FIRST + 14) /* Bit 14: 46 PWA1 */
#define BCM_IRQ_SMI              (IPR2_IRQ_FIRST + 16) /* Bit 16: 48 SMI */
#define BCM_IRQ_GPIO0            (IPR2_IRQ_FIRST + 17) /* Bit 17: 49 GPIO interrupt 0 */
#define BCM_IRQ_GPIO1            (IPR2_IRQ_FIRST + 18) /* Bit 18: 50 GPIO interrupt 1 */
#define BCM_IRQ_GPIO2            (IPR2_IRQ_FIRST + 19) /* Bit 19: 51 GPIO interrupt 2 */
#define BCM_IRQ_GPIO3            (IPR2_IRQ_FIRST + 20) /* Bit 20: 52 GPIO interrupt 3 */
#define BCM_IRQ_I2C              (IPR2_IRQ_FIRST + 21) /* Bit 21: 53 I2C interrupt */
#define BCM_IRQ_SPI              (IPR2_IRQ_FIRST + 22) /* Bit 22: 54 SPI interrupt */
#define BCM_IRQ_PCM              (IPR2_IRQ_FIRST + 23) /* Bit 23: 55 PCM interrupt */
#define BCM_IRQ_UART             (IPR2_IRQ_FIRST + 24) /* Bit 24: 57 UART interrupt */

#define IPR2_BIT_IRQMASK         0x01ff6800            /* Mask of defined interrupts */
#define IPR2_IRQ_LAST            BCM_IRQ_UART          /* IRQ of last defined bit */
#define IPR2_BIT_LAST            (24)                  /* Last defined bit */

/* Number of interrupts */

#define NR_INTERRUPTS            (IPR2_IRQ_LAST + 1)

/* Second level GPIO interrupts */

#ifdef CONFIG_BCM2708_GPIO_IRQ
#  define BCM_IRQ_GPIO(n)        (NR_INTERRUPTS + (n))
#  define NR_GPIOINTS            (54)
#else
#  define NR_GPIOINTS            (0)
#endif

/* Number of supported IRQs */

#define NR_IRQS                  (NR_INTERRUPTS + NR_GPIOINTS)

#else
#  error Unrecognized BCM2708 chip
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_BCM2708_IRQ_H */
