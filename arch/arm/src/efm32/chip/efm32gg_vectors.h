/*****************************************************************************
 * arch/arm/src/efm32/chip/efm32ggxxx_vectors.h
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
 *****************************************************************************/

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/
/* This file is included by efm32_vectors.S.  It provides the macro VECTOR
 * that supplies each EFM32GG vector in terms of a (lower-case) ISR label and
 * an (upper-case) IRQ number as defined in arch/arm/include/efm32/efm32gg_irq.h.
 * efm32_vectors.S will defined the VECTOR in different ways in order to
 * generate the interrupt vectors and handlers in their final form.
 *
 * Vectors for low and medium density devices
 */

#if defined(CONFIG_EFM32_EFM32GG)

/* If the common ARMv7-M vector handling is used, then all it needs is the
 * following definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 39 interrupt table entries for I/O interrupts. */

# define ARMV7M_PERIPHERAL_INTERRUPTS 39

#else
  VECTOR(EFM32_IRQ_DMA,         EFM32_IRQ_DMA       ) /*  0: EFM32_IRQ_DMA          */
  VECTOR(EFM32_IRQ_GPIO_EVEN,   EFM32_IRQ_GPIO_EVEN ) /*  1: EFM32_IRQ_GPIO_EVEN    */
  VECTOR(EFM32_IRQ_TIMER0,      EFM32_IRQ_TIMER0    ) /*  2: EFM32_IRQ_TIMER0       */
  VECTOR(EFM32_IRQ_USART0_RX,   EFM32_IRQ_USART0_RX ) /*  3: EFM32_IRQ_USART0_RX    */
  VECTOR(EFM32_IRQ_USART0_TX,   EFM32_IRQ_USART0_TX ) /*  4: EFM32_IRQ_USART0_TX    */
  VECTOR(EFM32_IRQ_USB,         EFM32_IRQ_USB       ) /*  5: EFM32_IRQ_USB          */
  VECTOR(EFM32_IRQ_ACMP,        EFM32_IRQ_ACMP      ) /*  6: EFM32_IRQ_ACMP         */
  VECTOR(EFM32_IRQ_ADC0,        EFM32_IRQ_ADC0      ) /*  7: EFM32_IRQ_ADC0         */
  VECTOR(EFM32_IRQ_DAC0,        EFM32_IRQ_DAC0      ) /*  8: EFM32_IRQ_DAC0         */
  VECTOR(EFM32_IRQ_I2C0,        EFM32_IRQ_I2C0      ) /*  9: EFM32_IRQ_I2C0         */
  VECTOR(EFM32_IRQ_I2C1,        EFM32_IRQ_I2C1      ) /* 10: EFM32_IRQ_I2C1         */
  VECTOR(EFM32_IRQ_GPIO_ODD,    EFM32_IRQ_GPIO_ODD  ) /* 11: EFM32_IRQ_GPIO_ODD     */
  VECTOR(EFM32_IRQ_TIMER1,      EFM32_IRQ_TIMER1    ) /* 12: EFM32_IRQ_TIMER1       */
  VECTOR(EFM32_IRQ_TIMER2,      EFM32_IRQ_TIMER2    ) /* 13: EFM32_IRQ_TIMER2       */
  VECTOR(EFM32_IRQ_TIMER3,      EFM32_IRQ_TIMER3    ) /* 14: EFM32_IRQ_TIMER3       */
  VECTOR(EFM32_IRQ_USART1_RX,   EFM32_IRQ_USART1_RX ) /* 15: EFM32_IRQ_USART1_RX    */
  VECTOR(EFM32_IRQ_USART1_TX,   EFM32_IRQ_USART1_TX ) /* 16: EFM32_IRQ_USART1_TX    */
  VECTOR(EFM32_IRQ_LESENSE,     EFM32_IRQ_LESENSE   ) /* 17: EFM32_IRQ_LESENSE      */
  VECTOR(EFM32_IRQ_USART2_RX,   EFM32_IRQ_USART2_RX ) /* 18: EFM32_IRQ_USART2_RX    */
  VECTOR(EFM32_IRQ_USART2_TX,   EFM32_IRQ_USART2_TX ) /* 19: EFM32_IRQ_USART2_TX    */
  VECTOR(EFM32_IRQ_UART0_RX,    EFM32_IRQ_UART0_RX  ) /* 20: EFM32_IRQ_UART0_RX     */
  VECTOR(EFM32_IRQ_UART0_TX,    EFM32_IRQ_UART0_TX  ) /* 21: EFM32_IRQ_UART0_TX     */
  VECTOR(EFM32_IRQ_UART1_RX,    EFM32_IRQ_UART1_RX  ) /* 22: EFM32_IRQ_UART1_RX     */
  VECTOR(EFM32_IRQ_UART1_TX,    EFM32_IRQ_UART1_TX  ) /* 23: EFM32_IRQ_UART1_TX     */
  VECTOR(EFM32_IRQ_LEUART0,     EFM32_IRQ_LEUART0   ) /* 24: EFM32_IRQ_LEUART0      */
  VECTOR(EFM32_IRQ_LEUART1,     EFM32_IRQ_LEUART1   ) /* 25: EFM32_IRQ_LEUART1      */
  VECTOR(EFM32_IRQ_LETIMER0,    EFM32_IRQ_LETIMER0  ) /* 26: EFM32_IRQ_LETIMER0     */
  VECTOR(EFM32_IRQ_PCNT0,       EFM32_IRQ_PCNT0     ) /* 27: EFM32_IRQ_PCNT0        */
  VECTOR(EFM32_IRQ_PCNT1,       EFM32_IRQ_PCNT1     ) /* 28: EFM32_IRQ_PCNT1        */
  VECTOR(EFM32_IRQ_PCNT2,       EFM32_IRQ_PCNT2     ) /* 29: EFM32_IRQ_PCNT2        */
  VECTOR(EFM32_IRQ_RTC,         EFM32_IRQ_RTC       ) /* 30: EFM32_IRQ_RTC          */
  VECTOR(EFM32_IRQ_BURTC,       EFM32_IRQ_BURTC     ) /* 31: EFM32_IRQ_BURTC        */
  VECTOR(EFM32_IRQ_CMU,         EFM32_IRQ_CMU       ) /* 32: EFM32_IRQ_CMU          */
  VECTOR(EFM32_IRQ_VCMP,        EFM32_IRQ_VCMP      ) /* 33: EFM32_IRQ_VCMP         */
  VECTOR(EFM32_IRQ_LCD,         EFM32_IRQ_LCD       ) /* 34: EFM32_IRQ_LCD          */
  VECTOR(EFM32_IRQ_MSC,         EFM32_IRQ_MSC       ) /* 35: EFM32_IRQ_MSC          */
  VECTOR(EFM32_IRQ_AES,         EFM32_IRQ_AES       ) /* 36: EFM32_IRQ_AES          */
  VECTOR(EFM32_IRQ_EBI,         EFM32_IRQ_EBI       ) /* 37: EFM32_IRQ_EBI          */
  VECTOR(EFM32_IRQ_EMI,         EFM32_IRQ_EMI       ) /* 38: EFM32_IRQ_EMI          */
#endif

#else
#  error "Unknown EFM32 Type"
#endif
