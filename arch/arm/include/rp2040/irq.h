/****************************************************************************
 * arch/arm/include/rp2040/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_RP2040_IRQ_H
#define __ARCH_ARM_INCLUDE_RP2040_IRQ_H

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

#define RP2040_IRQ_RESERVED    (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                   /* Vector  0: Reset stack pointer value */
                                   /* Vector  1: Reset (not handler as an IRQ) */
#define RP2040_IRQ_NMI         (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define RP2040_IRQ_HARDFAULT   (3) /* Vector  3: Hard fault */
                                   /* Vector  4-10: Reserved */
#define RP2040_IRQ_SVCALL     (11) /* Vector 11: SVC call */
                                   /* Vector 12-13: Reserved */
#define RP2040_IRQ_PENDSV     (14) /* Vector 14: Pendable system service request */
#define RP2040_IRQ_SYSTICK    (15) /* Vector 15: System tick */
#define RP2040_IRQ_EXTINT     (16) /* Vector 16: Vector number of the first external interrupt */

/* External interrupts (vectors >= 16).  These definitions are
 * chip-specific
 */

#define RP2040_TIMER_IRQ_0    (RP2040_IRQ_EXTINT+0)
#define RP2040_TIMER_IRQ_1    (RP2040_IRQ_EXTINT+1)
#define RP2040_TIMER_IRQ_2    (RP2040_IRQ_EXTINT+2)
#define RP2040_TIMER_IRQ_3    (RP2040_IRQ_EXTINT+3)
#define RP2040_PWM_IRQ_WRAP   (RP2040_IRQ_EXTINT+4)
#define RP2040_USBCTRL_IRQ    (RP2040_IRQ_EXTINT+5)
#define RP2040_XIP_IRQ        (RP2040_IRQ_EXTINT+6)
#define RP2040_PIO0_IRQ_0     (RP2040_IRQ_EXTINT+7)
#define RP2040_PIO0_IRQ_1     (RP2040_IRQ_EXTINT+8)
#define RP2040_PIO1_IRQ_0     (RP2040_IRQ_EXTINT+9)
#define RP2040_PIO1_IRQ_1     (RP2040_IRQ_EXTINT+10)
#define RP2040_DMA_IRQ_0      (RP2040_IRQ_EXTINT+11)
#define RP2040_DMA_IRQ_1      (RP2040_IRQ_EXTINT+12)
#define RP2040_IO_IRQ_BANK0   (RP2040_IRQ_EXTINT+13)
#define RP2040_IO_IRQ_QSPI    (RP2040_IRQ_EXTINT+14)
#define RP2040_SIO_IRQ_PROC0  (RP2040_IRQ_EXTINT+15)
#define RP2040_SIO_IRQ_PROC1  (RP2040_IRQ_EXTINT+16)
#define RP2040_CLOCKS_IRQ     (RP2040_IRQ_EXTINT+17)
#define RP2040_SPI0_IRQ       (RP2040_IRQ_EXTINT+18)
#define RP2040_SPI1_IRQ       (RP2040_IRQ_EXTINT+19)
#define RP2040_UART0_IRQ      (RP2040_IRQ_EXTINT+20)
#define RP2040_UART1_IRQ      (RP2040_IRQ_EXTINT+21)
#define RP2040_ADC_IRQ_FIFO   (RP2040_IRQ_EXTINT+22)
#define RP2040_I2C0_IRQ       (RP2040_IRQ_EXTINT+23)
#define RP2040_I2C1_IRQ       (RP2040_IRQ_EXTINT+24)
#define RP2040_RTC_IRQ        (RP2040_IRQ_EXTINT+25)

#define RP2040_IRQ_NEXTINT    (32)
#define RP2040_IRQ_NIRQS      (RP2040_IRQ_EXTINT+RP2040_IRQ_NEXTINT)

#define NR_VECTORS            RP2040_IRQ_NIRQS
#define NR_IRQS               RP2040_IRQ_NIRQS

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

#endif /* __ARCH_ARM_INCLUDE_RP2040_IRQ_H */

