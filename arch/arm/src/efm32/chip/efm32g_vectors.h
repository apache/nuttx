/*****************************************************************************
 * arch/arm/src/efm32/chip/efm32ggxxx_vectors.h
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
 *****************************************************************************/

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/
/* This file is included by efm32_vectors.S.  It provides the macro VECTOR
 * that supplies each EFM32G vector in terms of a (lower-case) ISR label and
 * an (upper-case) IRQ number as defined in arch/arm/include/efm32/efm32g_irq.h.
 * efm32_vectors.S will defined the VECTOR in different ways in order to
 * generate the interrupt vectors and handlers in their final form.
 *
 * Vectors for low and medium density devices
 */

#if defined(CONFIG_EFM32_EFM32G)

/* If the common ARMv7-M vector handling is used, then all it needs is the
 * following definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 30 interrupt table entries for I/O interrupts. */

# define ARMV7M_PERIPHERAL_INTERRUPTS 30

#else
                                                      /* IRQ# Source      */
  VECTOR(EFM32_IRQ_DMA,        EFM32_IRQ_DMA       ) /*  0   DMA         */
  VECTOR(EFM32_IRQ_GPIO_EVEN,  EFM32_IRQ_GPIO_EVEN ) /*  1   GPIO_EVEN   */
  VECTOR(EFM32_IRQ_TIMER0,     EFM32_IRQ_TIMER0    ) /*  2   TIMER0      */
  VECTOR(EFM32_IRQ_USART0_RX,  EFM32_IRQ_USART0_RX ) /*  3   USART0_RX   */
  VECTOR(EFM32_IRQ_USART0_TX,  EFM32_IRQ_USART0_TX ) /*  4   USART0_TX   */
  VECTOR(EFM32_IRQ_ACMP,       EFM32_IRQ_ACMP      ) /*  5   ACMP0/ACMP1 */
  VECTOR(EFM32_IRQ_ADC0,       EFM32_IRQ_ADC0      ) /*  6   ADC0        */
  VECTOR(EFM32_IRQ_DAC0,       EFM32_IRQ_DAC0      ) /*  7   DAC0        */
  VECTOR(EFM32_IRQ_I2C0,       EFM32_IRQ_I2C0      ) /*  8   I2C0        */
  VECTOR(EFM32_IRQ_GPIO_ODD,   EFM32_IRQ_GPIO_ODD  ) /*  9   GPIO_ODD    */
  VECTOR(EFM32_IRQ_TIMER1,     EFM32_IRQ_TIMER1    ) /* 10   TIMER1      */
  VECTOR(EFM32_IRQ_TIMER2,     EFM32_IRQ_TIMER2    ) /* 11   TIMER2      */
  VECTOR(EFM32_IRQ_USART1_RX,  EFM32_IRQ_USART1_RX ) /* 12   USART1_RX   */
  VECTOR(EFM32_IRQ_USART1_TX,  EFM32_IRQ_USART1_TX ) /* 13   USART1_TX   */
  VECTOR(EFM32_IRQ_USART2_RX,  EFM32_IRQ_USART2_RX ) /* 14   USART2_RX   */
  VECTOR(EFM32_IRQ_USART2_TX,  EFM32_IRQ_USART2_TX ) /* 15   USART2_TX   */
  VECTOR(EFM32_IRQ_UART0_RX,   EFM32_IRQ_UART0_RX  ) /* 16   UART0_RX    */
  VECTOR(EFM32_IRQ_UART0_TX,   EFM32_IRQ_UART0_TX  ) /* 17   UART0_TX    */
  VECTOR(EFM32_IRQ_LEUART0,    EFM32_IRQ_LEUART0   ) /* 18   LEUART0     */
  VECTOR(EFM32_IRQ_LEUART1,    EFM32_IRQ_LEUART1   ) /* 19   LEUART1     */
  VECTOR(EFM32_IRQ_LETIMER0,   EFM32_IRQ_LETIMER0  ) /* 20   LETIMER0    */
  VECTOR(EFM32_IRQ_PCNT0,      EFM32_IRQ_PCNT0     ) /* 21   PCNT0       */
  VECTOR(EFM32_IRQ_PCNT1,      EFM32_IRQ_PCNT1     ) /* 22   PCNT1       */
  VECTOR(EFM32_IRQ_PCNT2,      EFM32_IRQ_PCNT2     ) /* 23   PCNT2       */
  VECTOR(EFM32_IRQ_RTC,        EFM32_IRQ_RTC       ) /* 24   RTC         */
  VECTOR(EFM32_IRQ_CMU,        EFM32_IRQ_CMU       ) /* 25   CMU         */
  VECTOR(EFM32_IRQ_VCMP,       EFM32_IRQ_VCMP      ) /* 26   VCMP        */
  VECTOR(EFM32_IRQ_LCD,        EFM32_IRQ_LCD       ) /* 27   LCD         */
  VECTOR(EFM32_IRQ_MSC,        EFM32_IRQ_MSC       ) /* 28   MSC         */
  VECTOR(EFM32_IRQ_AES,        EFM32_IRQ_AES       ) /* 29   AES         */

#endif

#else
#  error "Unknown EFM32 Type"
#endif
