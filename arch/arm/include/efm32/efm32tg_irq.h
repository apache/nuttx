/****************************************************************************
 * arch/arm/include/efm32s/efm32tg_irq.h
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Author: Pierre-noel Bouteville <pnb990@gmail.com>
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_EFM32TG_IRQ_H
#define __ARCH_ARM_INCLUDE_EFM32TG_IRQ_H

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

#define EFM32_IRQ_DMA         (EFM32_IRQ_INTERRUPTS + 0)
#define EFM32_IRQ_GPIO_EVEN   (EFM32_IRQ_INTERRUPTS + 1)
#define EFM32_IRQ_TIMER0      (EFM32_IRQ_INTERRUPTS + 2)
#define EFM32_IRQ_USART0_RX   (EFM32_IRQ_INTERRUPTS + 3)
#define EFM32_IRQ_USART0_TX   (EFM32_IRQ_INTERRUPTS + 4)
#define EFM32_IRQ_ACMP        (EFM32_IRQ_INTERRUPTS + 5)
#define EFM32_IRQ_ADC0        (EFM32_IRQ_INTERRUPTS + 6)
#define EFM32_IRQ_DAC0        (EFM32_IRQ_INTERRUPTS + 7)
#define EFM32_IRQ_I2C0        (EFM32_IRQ_INTERRUPTS + 8)
#define EFM32_IRQ_GPIO_ODD    (EFM32_IRQ_INTERRUPTS + 9)
#define EFM32_IRQ_TIMER1      (EFM32_IRQ_INTERRUPTS + 10)
#define EFM32_IRQ_USART1_RX   (EFM32_IRQ_INTERRUPTS + 11)
#define EFM32_IRQ_USART1_TX   (EFM32_IRQ_INTERRUPTS + 12)
#define EFM32_IRQ_LESENSE     (EFM32_IRQ_INTERRUPTS + 13)
#define EFM32_IRQ_LEUART0     (EFM32_IRQ_INTERRUPTS + 14)
#define EFM32_IRQ_LETIMER0    (EFM32_IRQ_INTERRUPTS + 15)
#define EFM32_IRQ_PCNT0       (EFM32_IRQ_INTERRUPTS + 16)
#define EFM32_IRQ_RTC         (EFM32_IRQ_INTERRUPTS + 17)
#define EFM32_IRQ_CMU         (EFM32_IRQ_INTERRUPTS + 18)
#define EFM32_IRQ_VCMP        (EFM32_IRQ_INTERRUPTS + 19)
#define EFM32_IRQ_LCD         (EFM32_IRQ_INTERRUPTS + 20)
#define EFM32_IRQ_MSC         (EFM32_IRQ_INTERRUPTS + 21)
#define EFM32_IRQ_AES         (EFM32_IRQ_INTERRUPTS + 22)

#define EFM32_PERIPH_INTS     (23)
#define EFM32_IRQ_NVECTORS    (EFM32_IRQ_INTERRUPTS + EFM32_PERIPH_INTS)

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

#endif /* __ARCH_ARM_INCLUDE_EFM32TG_IRQ_H */
