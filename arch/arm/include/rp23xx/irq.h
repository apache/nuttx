/****************************************************************************
 * arch/arm/include/rp23xx/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_RP23XX_IRQ_H
#define __ARCH_ARM_INCLUDE_RP23XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC. This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define RP23XX_IRQ_RESERVED    (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                   /* Vector  0: Reset stack pointer value */
                                   /* Vector  1: Reset (not handler as an IRQ) */
#define RP23XX_IRQ_NMI         (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define RP23XX_IRQ_HARDFAULT   (3) /* Vector  3: Hard fault */
                                   /* Vector  4-10: Reserved */
#define RP23XX_IRQ_SVCALL     (11) /* Vector 11: SVC call */
                                   /* Vector 12-13: Reserved */
#define RP23XX_IRQ_PENDSV     (14) /* Vector 14: Pendable system service request */
#define RP23XX_IRQ_SYSTICK    (15) /* Vector 15: System tick */
#define RP23XX_IRQ_EXTINT     (16) /* Vector 16: Vector number of the first external interrupt */

/* External interrupts (vectors >= 16).  These definitions are
 * chip-specific
 */

#define RP23XX_TIMER0_IRQ_0     (RP23XX_IRQ_EXTINT+0)
#define RP23XX_TIMER0_IRQ_1     (RP23XX_IRQ_EXTINT+1)
#define RP23XX_TIMER0_IRQ_2     (RP23XX_IRQ_EXTINT+2)
#define RP23XX_TIMER0_IRQ_3     (RP23XX_IRQ_EXTINT+3)
#define RP23XX_TIMER1_IRQ_0     (RP23XX_IRQ_EXTINT+4)
#define RP23XX_TIMER1_IRQ_1     (RP23XX_IRQ_EXTINT+5)
#define RP23XX_TIMER1_IRQ_2     (RP23XX_IRQ_EXTINT+6)
#define RP23XX_TIMER1_IRQ_3     (RP23XX_IRQ_EXTINT+7)
#define RP23XX_PWM_IRQ_WRAP_0   (RP23XX_IRQ_EXTINT+8)
#define RP23XX_PWM_IRQ_WRAP_1   (RP23XX_IRQ_EXTINT+9)
#define RP23XX_DMA_IRQ_0        (RP23XX_IRQ_EXTINT+10)
#define RP23XX_DMA_IRQ_1        (RP23XX_IRQ_EXTINT+11)
#define RP23XX_DMA_IRQ_2        (RP23XX_IRQ_EXTINT+12)
#define RP23XX_DMA_IRQ_3        (RP23XX_IRQ_EXTINT+13)
#define RP23XX_USBCTRL_IRQ      (RP23XX_IRQ_EXTINT+14)
#define RP23XX_PIO0_IRQ_0       (RP23XX_IRQ_EXTINT+15)
#define RP23XX_PIO0_IRQ_1       (RP23XX_IRQ_EXTINT+16)
#define RP23XX_PIO1_IRQ_0       (RP23XX_IRQ_EXTINT+17)
#define RP23XX_PIO1_IRQ_1       (RP23XX_IRQ_EXTINT+18)
#define RP23XX_PIO2_IRQ_0       (RP23XX_IRQ_EXTINT+19)
#define RP23XX_PIO2_IRQ_1       (RP23XX_IRQ_EXTINT+20)
#define RP23XX_IO_IRQ_BANK0     (RP23XX_IRQ_EXTINT+21)
#define RP23XX_IO_IRQ_BANK0_NS  (RP23XX_IRQ_EXTINT+22)
#define RP23XX_IO_IRQ_QSPI      (RP23XX_IRQ_EXTINT+23)
#define RP23XX_IO_IRQ_QSPI_NS   (RP23XX_IRQ_EXTINT+24)
#define RP23XX_SIO_IRQ_FIFO     (RP23XX_IRQ_EXTINT+25)
#define RP23XX_SIO_IRQ_BELL     (RP23XX_IRQ_EXTINT+26)
#define RP23XX_SIO_IRQ_FIFO_NS  (RP23XX_IRQ_EXTINT+27)
#define RP23XX_SIO_IRQ_BELL_NS  (RP23XX_IRQ_EXTINT+28)
#define RP23XX_SIO_IRQ_MTIMECMP (RP23XX_IRQ_EXTINT+29)
#define RP23XX_CLOCKS_IRQ       (RP23XX_IRQ_EXTINT+30)
#define RP23XX_SPI0_IRQ         (RP23XX_IRQ_EXTINT+31)
#define RP23XX_SPI1_IRQ         (RP23XX_IRQ_EXTINT+32)
#define RP23XX_UART0_IRQ        (RP23XX_IRQ_EXTINT+33)
#define RP23XX_UART1_IRQ        (RP23XX_IRQ_EXTINT+34)
#define RP23XX_ADC_IRQ_FIFO     (RP23XX_IRQ_EXTINT+35)
#define RP23XX_I2C0_IRQ         (RP23XX_IRQ_EXTINT+36)
#define RP23XX_I2C1_IRQ         (RP23XX_IRQ_EXTINT+37)
#define RP23XX_OTP_IRQ          (RP23XX_IRQ_EXTINT+38)
#define RP23XX_TRNG_IRQ         (RP23XX_IRQ_EXTINT+39)
#define RP23XX_PROC0_IRQ_CTI    (RP23XX_IRQ_EXTINT+40)
#define RP23XX_PROC1_IRQ_CTI    (RP23XX_IRQ_EXTINT+41)
#define RP23XX_PLL_SYS_IRQ      (RP23XX_IRQ_EXTINT+42)
#define RP23XX_PLL_USB_IRQ      (RP23XX_IRQ_EXTINT+43)
#define RP23XX_POWMAN_IRQ_POW   (RP23XX_IRQ_EXTINT+44)
#define RP23XX_POWMAN_IRQ_TIMER (RP23XX_IRQ_EXTINT+45)
#define RP23XX_SPAREIRQ_IRQ_0   (RP23XX_IRQ_EXTINT+46)
#define RP23XX_SPAREIRQ_IRQ_1   (RP23XX_IRQ_EXTINT+47)
#define RP23XX_SPAREIRQ_IRQ_2   (RP23XX_IRQ_EXTINT+48)
#define RP23XX_SPAREIRQ_IRQ_3   (RP23XX_IRQ_EXTINT+49)
#define RP23XX_SPAREIRQ_IRQ_4   (RP23XX_IRQ_EXTINT+50)
#define RP23XX_SPAREIRQ_IRQ_5   (RP23XX_IRQ_EXTINT+51)

#define RP23XX_IRQ_NEXTINT      (52)
#define RP23XX_IRQ_NIRQS        (RP23XX_IRQ_EXTINT+RP23XX_IRQ_NEXTINT)

#define NR_VECTORS              RP23XX_IRQ_NIRQS
#define NR_IRQS                 RP23XX_IRQ_NIRQS

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
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
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_RP23XX_IRQ_H */
