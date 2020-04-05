/****************************************************************************
 * arch/arm/include/efm32s/efm32g_irq.h
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
 ****************************************************************************/

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_EFM32G_IRQ_H
#define __ARCH_ARM_INCLUDE_EFM32G_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be
 * found in nuttx/arch/arm/include/efm32/irq.h
 *
 * External interrupts (vectors >= 16)
 */
                                                         /* IRQ# Source      */
#define EFM32_IRQ_DMA        (EFM32_IRQ_INTERRUPTS + 0)  /*  0   DMA         */
#define EFM32_IRQ_GPIO_EVEN  (EFM32_IRQ_INTERRUPTS + 1)  /*  1   GPIO_EVEN   */
#define EFM32_IRQ_TIMER0     (EFM32_IRQ_INTERRUPTS + 2)  /*  2   TIMER0      */
#define EFM32_IRQ_USART0_RX  (EFM32_IRQ_INTERRUPTS + 3)  /*  3   USART0_RX   */
#define EFM32_IRQ_USART0_TX  (EFM32_IRQ_INTERRUPTS + 4)  /*  4   USART0_TX   */
#define EFM32_IRQ_ACMP       (EFM32_IRQ_INTERRUPTS + 5)  /*  5   ACMP0/ACMP1 */
#define EFM32_IRQ_ADC0       (EFM32_IRQ_INTERRUPTS + 6)  /*  6   ADC0        */
#define EFM32_IRQ_DAC0       (EFM32_IRQ_INTERRUPTS + 7)  /*  7   DAC0        */
#define EFM32_IRQ_I2C0       (EFM32_IRQ_INTERRUPTS + 8)  /*  8   I2C0        */
#define EFM32_IRQ_GPIO_ODD   (EFM32_IRQ_INTERRUPTS + 9)  /*  9   GPIO_ODD    */
#define EFM32_IRQ_TIMER1     (EFM32_IRQ_INTERRUPTS + 10) /* 10   TIMER1      */
#define EFM32_IRQ_TIMER2     (EFM32_IRQ_INTERRUPTS + 11) /* 11   TIMER2      */
#define EFM32_IRQ_USART1_RX  (EFM32_IRQ_INTERRUPTS + 12) /* 12   USART1_RX   */
#define EFM32_IRQ_USART1_TX  (EFM32_IRQ_INTERRUPTS + 13) /* 13   USART1_TX   */
#define EFM32_IRQ_USART2_RX  (EFM32_IRQ_INTERRUPTS + 14) /* 14   USART2_RX   */
#define EFM32_IRQ_USART2_TX  (EFM32_IRQ_INTERRUPTS + 15) /* 15   USART2_TX   */
#define EFM32_IRQ_UART0_RX   (EFM32_IRQ_INTERRUPTS + 16) /* 16   UART0_RX    */
#define EFM32_IRQ_UART0_TX   (EFM32_IRQ_INTERRUPTS + 17) /* 17   UART0_TX    */
#define EFM32_IRQ_LEUART0    (EFM32_IRQ_INTERRUPTS + 18) /* 18   LEUART0     */
#define EFM32_IRQ_LEUART1    (EFM32_IRQ_INTERRUPTS + 19) /* 19   LEUART1     */
#define EFM32_IRQ_LETIMER0   (EFM32_IRQ_INTERRUPTS + 20) /* 20   LETIMER0    */
#define EFM32_IRQ_PCNT0      (EFM32_IRQ_INTERRUPTS + 21) /* 21   PCNT0       */
#define EFM32_IRQ_PCNT1      (EFM32_IRQ_INTERRUPTS + 22) /* 22   PCNT1       */
#define EFM32_IRQ_PCNT2      (EFM32_IRQ_INTERRUPTS + 23) /* 23   PCNT2       */
#define EFM32_IRQ_RTC        (EFM32_IRQ_INTERRUPTS + 24) /* 24   RTC         */
#define EFM32_IRQ_CMU        (EFM32_IRQ_INTERRUPTS + 25) /* 25   CMU         */
#define EFM32_IRQ_VCMP       (EFM32_IRQ_INTERRUPTS + 26) /* 26   VCMP        */
#define EFM32_IRQ_LCD        (EFM32_IRQ_INTERRUPTS + 27) /* 27   LCD         */
#define EFM32_IRQ_MSC        (EFM32_IRQ_INTERRUPTS + 28) /* 28   MSC         */
#define EFM32_IRQ_AES        (EFM32_IRQ_INTERRUPTS + 29) /* 29   AES         */

#define EFM32_PERIPH_INTS    (30)
#define EFM32_IRQ_NVECTORS   (EFM32_IRQ_INTERRUPTS + EFM32_PERIPH_INTS)

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
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_EFM32G_IRQ_H */
