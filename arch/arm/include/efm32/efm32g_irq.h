/****************************************************************************
 * arch/arm/include/efm32/efm32g_irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_EFM32_EFM32G_IRQ_H
#define __ARCH_ARM_INCLUDE_EFM32_EFM32G_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
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

                                                         /* IRQ# Source     */
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
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_EFM32_EFM32G_IRQ_H */
