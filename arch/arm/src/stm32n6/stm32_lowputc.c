/****************************************************************************
 * arch/arm/src/stm32n6/stm32_lowputc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "stm32.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select USART parameters for the selected console.  Only USART1 is
 * supported in this initial port.
 */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define STM32N6_CONSOLE_BASE     STM32_USART1_BASE
#    define STM32N6_APBCLOCK         STM32_HSI_FREQUENCY  /* USART1SEL=HSI via CCIPR13 */
#    define STM32N6_CONSOLE_APBREG   STM32_RCC_APB2ENSR
#    define STM32N6_CONSOLE_APBEN    RCC_APB2ENR_USART1EN
#    define STM32N6_CONSOLE_BAUD     CONFIG_USART1_BAUD
#    define STM32N6_CONSOLE_BITS     CONFIG_USART1_BITS
#    define STM32N6_CONSOLE_PARITY   CONFIG_USART1_PARITY
#    define STM32N6_CONSOLE_2STOP    CONFIG_USART1_2STOP
#    define STM32N6_CONSOLE_TX       GPIO_USART1_TX
#    define STM32N6_CONSOLE_RX       GPIO_USART1_RX
#  endif

  /* CR1 settings */

#  if STM32N6_CONSOLE_BITS == 9
#    define USART_CR1_M0_VALUE USART_CR1_M0
#    define USART_CR1_M1_VALUE 0
#  elif STM32N6_CONSOLE_BITS == 7
#    define USART_CR1_M0_VALUE 0
#    define USART_CR1_M1_VALUE USART_CR1_M1
#  else /* 8 bits */
#    define USART_CR1_M0_VALUE 0
#    define USART_CR1_M1_VALUE 0
#  endif

#  if STM32N6_CONSOLE_PARITY == 1 /* odd parity */
#    define USART_CR1_PARITY_VALUE (USART_CR1_PCE|USART_CR1_PS)
#  elif STM32N6_CONSOLE_PARITY == 2 /* even parity */
#    define USART_CR1_PARITY_VALUE USART_CR1_PCE
#  else /* no parity */
#    define USART_CR1_PARITY_VALUE 0
#  endif

#  define USART_CR1_CLRBITS \
    (USART_CR1_UE | USART_CR1_UESM | USART_CR1_RE | USART_CR1_TE | USART_CR1_PS | \
     USART_CR1_PCE | USART_CR1_WAKE | USART_CR1_M0 | USART_CR1_M1 | \
     USART_CR1_MME | USART_CR1_OVER8 | USART_CR1_DEDT_MASK | \
     USART_CR1_DEAT_MASK | USART_CR1_ALLINTS)

#  define USART_CR1_SETBITS (USART_CR1_M0_VALUE|USART_CR1_M1_VALUE|USART_CR1_PARITY_VALUE)

  /* CR2 settings */

#  if STM32N6_CONSOLE_2STOP != 0
#    define USART_CR2_STOP2_VALUE USART_CR2_STOP2
#  else
#    define USART_CR2_STOP2_VALUE 0
#  endif

#  define USART_CR2_CLRBITS \
    (USART_CR2_ADDM7 | USART_CR2_LBDL | USART_CR2_LBDIE | USART_CR2_LBCL | \
     USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_CLKEN | USART_CR2_STOP_MASK | \
     USART_CR2_LINEN | USART_CR2_SWAP | USART_CR2_RXINV | USART_CR2_TXINV | \
     USART_CR2_DATAINV | USART_CR2_MSBFIRST | USART_CR2_ABREN | \
     USART_CR2_ABRMOD_MASK | USART_CR2_RTOEN | USART_CR2_ADD_MASK)

#  define USART_CR2_SETBITS USART_CR2_STOP2_VALUE

  /* CR3 settings */

#  define USART_CR3_CLRBITS \
    (USART_CR3_EIE | USART_CR3_IREN | USART_CR3_IRLP | USART_CR3_HDSEL | \
     USART_CR3_NACK | USART_CR3_SCEN | USART_CR3_DMAR | USART_CR3_DMAT | \
     USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_CTSIE | USART_CR3_ONEBIT | \
     USART_CR3_OVRDIS | USART_CR3_DDRE | USART_CR3_DEM | USART_CR3_DEP | \
     USART_CR3_SCARCNT2_MASK | USART_CR3_WUS_MASK | USART_CR3_WUFIE)

#  define USART_CR3_SETBITS 0

#  undef USE_OVER8

  /* Calculate USART BAUD rate divider.
   *
   * Baud rate for standard USART (SPI mode included):
   *
   * In case of oversampling by 16, the equation is:
   *   baud    = fCK / UARTDIV
   *   UARTDIV = fCK / baud
   *
   * In case of oversampling by 8, the equation is:
   *
   *   baud    = 2 * fCK / UARTDIV
   *   UARTDIV = 2 * fCK / baud
   */

#  define STM32N6_USARTDIV8 \
      (((STM32N6_APBCLOCK << 1) + (STM32N6_CONSOLE_BAUD >> 1)) / STM32N6_CONSOLE_BAUD)
#  define STM32N6_USARTDIV16 \
      ((STM32N6_APBCLOCK + (STM32N6_CONSOLE_BAUD >> 1)) / STM32N6_CONSOLE_BAUD)

  /* Pick OVER8 only when the divisor is small enough that the loss of
   * the low BRR bit (DIV_FRACTION[0]) does not exceed half a step.
   */

#  if STM32N6_USARTDIV8 > 2000
#    define STM32N6_BRR_VALUE STM32N6_USARTDIV16
#  else
#    define USE_OVER8 1
#    define STM32N6_BRR_VALUE \
      ((STM32N6_USARTDIV8 & 0xfff0) | ((STM32N6_USARTDIV8 & 0x000f) >> 1))
#  endif
#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  while ((getreg32(STM32N6_CONSOLE_BASE + STM32_USART_ISR_OFFSET) &
         USART_ISR_TXE) == 0);

  putreg32((uint32_t)ch, STM32N6_CONSOLE_BASE + STM32_USART_TDR_OFFSET);
#endif
}

/****************************************************************************
 * Name: stm32_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void stm32_lowsetup(void)
{
#if defined(HAVE_UART)
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t cr;
#endif

#if defined(HAVE_CONSOLE)
  /* Use the write-1-to-set ENSR alias rather than RMW on ENR so we do
   * not race other producers of the clock-enable bitmap.
   */

  putreg32(STM32N6_CONSOLE_APBEN, STM32N6_CONSOLE_APBREG);
#endif

#ifdef STM32N6_CONSOLE_TX
  stm32_configgpio(STM32N6_CONSOLE_TX);
#endif
#ifdef STM32N6_CONSOLE_RX
  stm32_configgpio(STM32N6_CONSOLE_RX);
#endif

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  cr  = getreg32(STM32N6_CONSOLE_BASE + STM32_USART_CR2_OFFSET);
  cr &= ~USART_CR2_CLRBITS;
  cr |= USART_CR2_SETBITS;
  putreg32(cr, STM32N6_CONSOLE_BASE + STM32_USART_CR2_OFFSET);

  cr  = getreg32(STM32N6_CONSOLE_BASE + STM32_USART_CR1_OFFSET);
  cr &= ~USART_CR1_CLRBITS;
  cr |= USART_CR1_SETBITS;
  putreg32(cr, STM32N6_CONSOLE_BASE + STM32_USART_CR1_OFFSET);

  cr  = getreg32(STM32N6_CONSOLE_BASE + STM32_USART_CR3_OFFSET);
  cr &= ~USART_CR3_CLRBITS;
  cr |= USART_CR3_SETBITS;
  putreg32(cr, STM32N6_CONSOLE_BASE + STM32_USART_CR3_OFFSET);

  putreg32(STM32N6_BRR_VALUE,
           STM32N6_CONSOLE_BASE + STM32_USART_BRR_OFFSET);

  cr  = getreg32(STM32N6_CONSOLE_BASE + STM32_USART_CR1_OFFSET);
#ifdef USE_OVER8
  cr |= USART_CR1_OVER8;
  putreg32(cr, STM32N6_CONSOLE_BASE + STM32_USART_CR1_OFFSET);
#endif

  cr |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
  putreg32(cr, STM32N6_CONSOLE_BASE + STM32_USART_CR1_OFFSET);
#endif
#endif
}
