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

#define BCM_IRQ_ARM_TIMER        (BPR_IRQ_FIRST + 0)  /* Bit 0:  ARM Timer IRQ pending */
#define BCM_IRQ_ARM_MAILBOX      (BPR_IRQ_FIRST + 1)  /* Bit 1:  ARM Mailbox IRQ pending */
#define BCM_IRQ_ARM_DOORBELL_0   (BPR_IRQ_FIRST + 2)  /* Bit 2:  ARM Doorbell 0 IRQ pending */
#define BCM_IRQ_ARM_DOORBELL_1   (BPR_IRQ_FIRST + 3)  /* Bit 3:  ARM Doorbell 2 IRQ pending */
#define BCM_IRQ_VPU0_HALTED      (BPR_IRQ_FIRST + 4)  /* Bit 4:  GPU0 halted IRQ pending
                                                       * (Or GPU1 halted if bit 10 of control
                                                       * register 1 is set) */
#define BCM_IRQ_VPU1_HALTED      (BPR_IRQ_FIRST + 5)  /* Bit 5:  GPU1 halted IRQ pending */
#define BCM_IRQ_ILLEGAL_TYPE0    (BPR_IRQ_FIRST + 6)  /* Bit 6:  Illegal access type 1 IRQ pending */
#define BCM_IRQ_ILLEGAL_TYPE1    (BPR_IRQ_FIRST + 7)  /* Bit 7:  Illegal access type 0 IRQ pending */

#define BCM_IRQ_PENDING1         (BPR_IRQ_FIRST + 8)  /* Bit 8:  Bits set in pending register 1 */
#define BCM_IRQ_PENDING2         (BPR_IRQ_FIRST + 9)  /* Bit 9:  Bits set in pending register 2 */

#define BCM_IRQ_JPEG             (BPR_IRQ_FIRST + 10) /* Bit 10: GPU IRQ 7 */
#define BCM_IRQ_USB              (BPR_IRQ_FIRST + 11) /* Bit 11: GPU IRQ 9 */
#define BCM_IRQ_3D               (BPR_IRQ_FIRST + 12) /* Bit 12: GPU IRQ 10 */
#define BCM_IRQ_DMA2             (BPR_IRQ_FIRST + 13) /* Bit 13: GPU IRQ 18 */
#define BCM_IRQ_DMA3             (BPR_IRQ_FIRST + 14) /* Bit 14: GPU IRQ 19 */
#define BCM_IRQ_I2C              (BPR_IRQ_FIRST + 15) /* Bit 15: GPU IRQ 53 */
#define BCM_IRQ_SPI              (BPR_IRQ_FIRST + 16) /* Bit 16: GPU IRQ 54 */
#define BCM_IRQ_I2SPCM           (BPR_IRQ_FIRST + 17) /* Bit 17: GPU IRQ 55 */
#define BCM_IRQ_SDIO             (BPR_IRQ_FIRST + 18) /* Bit 18: GPU IRQ 56 */
#define BCM_IRQ_UART             (BPR_IRQ_FIRST + 19) /* Bit 19: GPU IRQ 57 */
#define BCM_IRQ_ARASANSDIO       (BPR_IRQ_FIRST + 20) /* Bit 20: GPU IRQ 61 */

#define BPR_BIT_IRQMASK          0x001ffcff            /* Mask of defined interrupts */
#define BPR_BIT_LAST             BCM_IRQ_ARASANSDIO    /* IRQ of last defined bit */
#define BPR_IRQ_LAST             20                    /* Last defined bit */

/* IRQ pending 1 register */

#define IPR1_IRQ_FIRST           (BPR_IRQ_LAST + 1)    /* IRQ of first defined bit */
#define IPR1_BIT_FIRST           (0)                   /* First defined bit */

#define BCM_IRQ_TIMER0           (IPR1_IRQ_FIRST + 0)  /* Bit 0: System Timer Compare Register 0 */
#define BCM_IRQ_TIMER1           (IPR1_IRQ_FIRST + 1)  /* Bit 1: System Timer Compare Register 1 */
#define BCM_IRQ_TIMER2           (IPR1_IRQ_FIRST + 2)  /* Bit 2: System Timer Compare Register 2 */
#define BCM_IRQ_TIMER3           (IPR1_IRQ_FIRST + 3)  /* Bit 3: System Timer Compare Register 3 */
#define BCM_IRQ_CODEC0           (IPR1_IRQ_FIRST + 4)
#define BCM_IRQ_CODEC1           (IPR1_IRQ_FIRST + 5)
#define BCM_IRQ_CODEC2           (IPR1_IRQ_FIRST + 6)
#define BCM_IRQ_VC_JPEG          (IPR1_IRQ_FIRST + 7)
#define BCM_IRQ_ISP              (IPR1_IRQ_FIRST + 8)
#define BCM_IRQ_VC_USB           (IPR1_IRQ_FIRST + 9)  /* Bit 9: USB Controller */
#define BCM_IRQ_VC_3D            (IPR1_IRQ_FIRST + 10)
#define BCM_IRQ_TRANSPOSER       (IPR1_IRQ_FIRST + 11)
#define BCM_IRQ_MULTICORESYNC0   (IPR1_IRQ_FIRST + 12)
#define BCM_IRQ_MULTICORESYNC1   (IPR1_IRQ_FIRST + 13)
#define BCM_IRQ_MULTICORESYNC2   (IPR1_IRQ_FIRST + 14)
#define BCM_IRQ_MULTICORESYNC3   (IPR1_IRQ_FIRST + 15)
#define BCM_IRQ_DMA0             (IPR1_IRQ_FIRST + 16)
#define BCM_IRQ_DMA1             (IPR1_IRQ_FIRST + 17)
#define BCM_IRQ_VC_DMA2          (IPR1_IRQ_FIRST + 18)
#define BCM_IRQ_VC_DMA3          (IPR1_IRQ_FIRST + 19)
#define BCM_IRQ_DMA4             (IPR1_IRQ_FIRST + 20)
#define BCM_IRQ_DMA5             (IPR1_IRQ_FIRST + 21)
#define BCM_IRQ_DMA6             (IPR1_IRQ_FIRST + 22)
#define BCM_IRQ_DMA7             (IPR1_IRQ_FIRST + 23)
#define BCM_IRQ_DMA8             (IPR1_IRQ_FIRST + 24)
#define BCM_IRQ_DMA9             (IPR1_IRQ_FIRST + 25)
#define BCM_IRQ_DMA10            (IPR1_IRQ_FIRST + 26)
#define BCM_IRQ_DMA11            (IPR1_IRQ_FIRST + 27)
#define BCM_IRQ_DMA12            (IPR1_IRQ_FIRST + 28)
#define BCM_IRQ_AUX              (IPR1_IRQ_FIRST + 29) /* Bit 29: Aux interrupt */
#define BCM_IRQ_ARM              (IPR1_IRQ_FIRST + 30)
#define BCM_IRQ_VPUDMA           (IPR1_IRQ_FIRST + 31)

#define IPR1_BIT_IRQMASK         0x20000000            /* Mask of defined interrupts */
#define IPR1_IRQ_LAST            BCM_IRQ_VPUDMA        /* IRQ of last defined bit */
#define IPR1_BIT_LAST            (31)                  /* Last defined bit */

/* IRQ pending 1 register */

#define IPR2_IRQ_FIRST           (IPR1_IRQ_LAST + 1)   /* IRQ of first defined bit */
#define IPR2_BIT_FIRST           (0)                   /* First defined bit */

#define BCM_IRQ_HOSTPORT         (IPR2_IRQ_FIRST + 0)
#define BCM_IRQ_VIDEOSCALER      (IPR2_IRQ_FIRST + 1)
#define BCM_IRQ_CCP2TX           (IPR2_IRQ_FIRST + 2)
#define BCM_IRQ_SDC              (IPR2_IRQ_FIRST + 3)
#define BCM_IRQ_DSI0             (IPR2_IRQ_FIRST + 4)
#define BCM_IRQ_AVE              (IPR2_IRQ_FIRST + 5)
#define BCM_IRQ_CAM0             (IPR2_IRQ_FIRST + 6)
#define BCM_IRQ_CAM1             (IPR2_IRQ_FIRST + 7)
#define BCM_IRQ_HDMI0            (IPR2_IRQ_FIRST + 8)
#define BCM_IRQ_HDMI1            (IPR2_IRQ_FIRST + 9)
#define BCM_IRQ_PIXELVALVE1      (IPR2_IRQ_FIRST + 10)
#define BCM_IRQ_I2CSPISLV        (IPR2_IRQ_FIRST + 11) /* Bit 11: I2C/SPI slave */
#define BCM_IRQ_DSI1             (IPR2_IRQ_FIRST + 12)
#define BCM_IRQ_PWA0             (IPR2_IRQ_FIRST + 13) /* Bit 13: PWA0 */
#define BCM_IRQ_PWA1             (IPR2_IRQ_FIRST + 14) /* Bit 14: PWA1 */
#define BCM_IRQ_CPR              (IPR2_IRQ_FIRST + 15)
#define BCM_IRQ_SMI              (IPR2_IRQ_FIRST + 16) /* Bit 16: SMI */
#define BCM_IRQ_GPIO0            (IPR2_IRQ_FIRST + 17) /* Bit 17: GPIO interrupt 0 */
#define BCM_IRQ_GPIO1            (IPR2_IRQ_FIRST + 18) /* Bit 18: GPIO interrupt 1 */
#define BCM_IRQ_GPIO2            (IPR2_IRQ_FIRST + 19) /* Bit 19: GPIO interrupt 2 */
#define BCM_IRQ_GPIO3            (IPR2_IRQ_FIRST + 20) /* Bit 20: GPIO interrupt 3 */
#define BCM_IRQ_VC_I2C           (IPR2_IRQ_FIRST + 21) /* Bit 21: I2C interrupt */
#define BCM_IRQ_VC_SPI           (IPR2_IRQ_FIRST + 22) /* Bit 22: SPI interrupt */
#define BCM_IRQ_VC_I2SPCM        (IPR2_IRQ_FIRST + 23) /* Bit 23: PCM audio interrupt */
#define BCM_IRQ_VC_SDIO          (IPR2_IRQ_FIRST + 24) /* Bit 24: SDIO interrupt */
#define BCM_IRQ_VC_UART          (IPR2_IRQ_FIRST + 25)
#define BCM_IRQ_SLIMBUS          (IPR2_IRQ_FIRST + 26)
#define BCM_IRQ_VEC              (IPR2_IRQ_FIRST + 27)
#define BCM_IRQ_CPG              (IPR2_IRQ_FIRST + 28)
#define BCM_IRQ_RNG              (IPR2_IRQ_FIRST + 29)
#define BCM_IRQ_VC_ARASANSDIO    (IPR2_IRQ_FIRST + 30) /* Bit 30: SD Host Controller */
#define BCM_IRQ_AVSPMON          (IPR2_IRQ_FIRST + 31)

#define IPR2_BIT_IRQMASK         0xffffffff            /* Mask of defined interrupts */
#define IPR2_IRQ_LAST            BCM_IRQ_AVSPMON       /* IRQ of last defined bit */
#define IPR2_BIT_LAST            (31)                  /* Last defined bit */

/* Number of hardware interrupt vectors */

#define BCM_IRQ_NVECTORS         (IPR2_IRQ_LAST + 1)

/* Second level GPIO interrupts */

#ifdef CONFIG_BCM2708_GPIO_IRQ
#  define BCM_IRQ_GPIO(n)        (BCM_IRQ_NVECTORS + (n)) /* IRQ number of pin n */
#  define BCM_IRQ_GPIO0_FIRST    (BCM_IRQ_NVECTORS)       /* IRQ number of first GPIO0 interrupt */
#  define BCM_IRQ_GPIO1_FIRST    (BCM_IRQ_NVECTORS + 32)  /* IRQ number of first GPIO1 interrupt */
#  define BCM_IRQ_NGPIOINTS      (54)
#else
#  define BCM_IRQ_NGPIOINTS      (0)
#endif

/* Number of supported IRQs */

#define NR_IRQS                  (BCM_IRQ_NVECTORS + BCM_IRQ_NGPIOINTS)

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
