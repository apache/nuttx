/************************************************************************************************
 * arch/arm/include/max326xx/max32660.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

/* This file should never be included directly but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_MAX326XX_MAX32660_IRQ_H
#define __ARCH_ARM_INCLUDE_MAX326XX_MAX32660_IRQ_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* External interrupts (vectors >= 16) */

#define MAX326_IRQ_PF         (MAX326_IRQ_EXTINT + 0)    /*  0  Power Fail */
#define MAX326_IRQ_WDT0       (MAX326_IRQ_EXTINT + 1)    /*  1  Watchdog 0 Interrupt */
                                                         /*  2  Reserved */
#define MAX326_IRQ_RTC        (MAX326_IRQ_EXTINT + 3)    /*  3  RTC Interrupt */
                                                         /*  4  Reserved */
#define MAX326_IRQ_TMR0       (MAX326_IRQ_EXTINT + 5)    /*  5  Timer 0 Interrupt */
#define MAX326_IRQ_TMR1       (MAX326_IRQ_EXTINT + 6)    /*  6  Timer 1 Interrupt */
#define MAX326_IRQ_TMR2       (MAX326_IRQ_EXTINT + 7)    /*  7  Timer 2 Interrupt */
                                                         /*  8-12  Reserved */
#define MAX326_IRQ_I2C0       (MAX326_IRQ_EXTINT + 13)   /* 13  I2C0 */
#define MAX326_IRQ_UART0      (MAX326_IRQ_EXTINT + 14)   /* 14  UART 0 */
#define MAX326_IRQ_UART1      (MAX326_IRQ_EXTINT + 15)   /* 15  UART 1 */
#define MAX326_IRQ_SPI        (MAX326_IRQ_EXTINT + 16)   /* 16  SPI17Y */
#define MAX326_IRQ_SPIMSS     (MAX326_IRQ_EXTINT + 17)   /* 17  SPIMSS/I2S */
                                                         /* 18-22  Reserved */
#define MAX326_IRQ_FLC        (MAX326_IRQ_EXTINT + 23)   /* 23  Flash Controller (FLC) */
#define MAX326_IRQ_GPIO0      (MAX326_IRQ_EXTINT + 24)   /* 24  GPIO P0.1-7 External Interrupts */
                                                         /* 25-27  Reserved */
#define MAX326_IRQ_DMA0       (MAX326_IRQ_EXTINT + 28)   /* 28  DMA0 */
#define MAX326_IRQ_DMA1       (MAX326_IRQ_EXTINT + 29)   /* 29  DMA1 */
#define MAX326_IRQ_DMA2       (MAX326_IRQ_EXTINT + 30)   /* 30  DMA2 */
#define MAX326_IRQ_DMA3       (MAX326_IRQ_EXTINT + 31)   /* 31  DMA3 */
                                                         /* 32-35  Reserved */
#define MAX326_IRQ_I2C1       (MAX326_IRQ_EXTINT + 36)   /* 36  I2C1 */
                                                         /* 37-53  Reserved */
#define MAX326_IRQ_WAKEUP     (MAX326_IRQ_EXTINT + 54)   /* 54  GPIO Wakeup */

/* Number of external interrupts and number of true interrupt vectors */

#define MAX326_IRQ_NEXTINT    55
#define MAX326_IRQ_NVECTORS   (MAX326_IRQ_EXTINT + MAX326_IRQ_NEXTINT)

/* If GPIO pin interrupts are used then there is a second level of interrupt decode */

#ifdef CONFIG_MAX326XX_GPIOIRQ
/* Up to 14 pins are available as interrupt sources, depending on the MAX32660 package */

#  define MAX326_IRQ_P0_0     (MAX326_IRQ_NEXTINT + 0)
#  define MAX326_IRQ_P0_1     (MAX326_IRQ_NEXTINT + 1)
#  define MAX326_IRQ_P0_2     (MAX326_IRQ_NEXTINT + 2)
#  define MAX326_IRQ_P0_3     (MAX326_IRQ_NEXTINT + 3)
#  define MAX326_IRQ_P0_4     (MAX326_IRQ_NEXTINT + 4)
#  define MAX326_IRQ_P0_5     (MAX326_IRQ_NEXTINT + 5)
#  define MAX326_IRQ_P0_6     (MAX326_IRQ_NEXTINT + 6)
#  define MAX326_IRQ_P0_7     (MAX326_IRQ_NEXTINT + 7)
#  define MAX326_IRQ_P0_8     (MAX326_IRQ_NEXTINT + 8)
#  define MAX326_IRQ_P0_9     (MAX326_IRQ_NEXTINT + 9)
#  define MAX326_IRQ_P0_10    (MAX326_IRQ_NEXTINT + 10)
#  define MAX326_IRQ_P0_11    (MAX326_IRQ_NEXTINT + 11)
#  define MAX326_IRQ_P0_12    (MAX326_IRQ_NEXTINT + 12)
#  define MAX326_IRQ_P0_13    (MAX326_IRQ_NEXTINT + 13)

#  define MAX326_IRQ_GPIO1ST   MAX326_IRQ_P0_0
#  define MAX326_IRQ_GPIOLAST  MAX326_IRQ_P0_13
#  define MAX326_IRQ_NPININT   14
#else
#  define MAX326_IRQ_NPININT   0
#endif

/* Total number of interrupts handled by the OS */

#define NR_IRQS               (MAX326_IRQ_NVECTORS + MAX326_IRQ_NPININT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Inline functions
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Function Prototypes
 ************************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_MAX326XX_MAX32660_IRQ_H */
