/****************************************************************************
 * arch/arm/src/n32h7/n32_lowputc.c
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

#include "arm_internal.h"
#include "chip.h"

#include "hardware/n32h7_pinmap.h"
#include "hardware/n32h7_uart.h"
#include "n32_lowputc.h"
#include "n32_rcc.h"
#include "n32_gpio.h"
#include "n32_uart.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select USART parameters for the selected console */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define N32_CONSOLE_BASE     N32_USART1_BASE
#    define N32_AHBCLOCK         N32_AHB_FREQUENCY
#    define N32_CONSOLE_APBREG   N32_RCC_APB1EN3
#    define N32_CONSOLE_APBEN    RCC_APB1EN3_M7USART1EN
#    define N32_CONSOLE_BAUD     CONFIG_USART1_BAUD
#    define N32_CONSOLE_BITS     CONFIG_USART1_BITS
#    define N32_CONSOLE_PARITY   CONFIG_USART1_PARITY
#    define N32_CONSOLE_2STOP    CONFIG_USART1_2STOP
#    define N32_CONSOLE_TX       GPIO_USART1_TX
#    define N32_CONSOLE_RX       GPIO_USART1_RX
#  else
#    error "No valid console configuration"
#  endif

  /* CTRL1 settings */

#  if N32_CONSOLE_BITS == 9
#    define CTRL1_WL USART_CTRL1_WL
#  else /*  N32_CONSOLE_BITS == 8 */
#    define CTRL1_WL 0b0
#  endif

#  if N32_CONSOLE_PARITY == 1
#    define CTRL1_PARITY (USART_CTRL1_PCEN | USART_CTRL1_PSEL)
#  elif N32_CONSOLE_PARITY == 2
#    define CTRL1_PARITY USART_CTRL1_PCEN
#  else
#    define CTRL1_PARITY 0b00
#  endif

#  define USART_CTRL1_CLRBITS  (USART_CTRL1_RXEN | USART_CTRL1_TXEN \
                                |USART_CTRL1_PSEL | USART_CTRL1_PCEN \
                                |USART_CTRL1_OSPM | USART_CTRL1_WL)

#  define USART_CTRL1_SETBITS (CTRL1_WL         | CTRL1_PARITY  )

  /* CTRL2 settings */

#  if N32_CONSOLE_2STOP != 0
#    define CTRL2_STOP2 USART_CTRL2_STPB(N32_CONSOLE_2STOP)
#  else
#    define CTRL2_STOP2 0b0
#  endif

#  define USART_CTRL2_CLRBITS (USART_CTRL2_STPB_MASK)

#  define USART_CTRL2_SETBITS CTRL2_STOP2

  /* CTRL3 settings */

#  define USART_CTRL3_CLRBITS (USART_CTRL3_CTSEN | USART_CTRL3_RTSEN)

#  define USART_CTRL3_SETBITS 0

  /* Calculate USART BAUD rate divider */

  /* Baud rate for standard USART (SPI mode included):
   *
   * In case of oversampling by 16, the equation is:
   *   baud    = fCK / (16 * UARTDIV)
   *   UARTDIV = fCK / (16 * baud)
   *
   */

#  define N32_USARTDIV16 \
    ((25 * (N32_AHBCLOCK / 4)) / (N32_CONSOLE_BAUD))
#  define N32_USARTDIV8 \
    ((25 * (N32_AHBCLOCK / 2)) / (N32_CONSOLE_BAUD))

  /* Use oversampling by 8 only if the divisor is small.  But what is
   * small?
   */
#  undef USE_OVER8
#  if N32_USARTDIV8 > 100
#    define N32_UART_DIV N32_USARTDIV16
#  else
#    define USE_OVER8 1
#    define N32_UART_DIV \
      ((N32_USARTDIV8 & 0xfff0) | ((N32_USARTDIV8 & 0x000f) >> 1))
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
 * Private Data
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
 * Desregvaliption:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE

  /* Wait until the TX data register is empty */

  while ((getreg32(N32_CONSOLE_BASE + N32_USART_STS_OFFSET) &
         USART_STS_TXDE) == 0);

  /* Then send the character */

  putreg32((uint32_t)ch, N32_CONSOLE_BASE + N32_USART_DAT_OFFSET);

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: n32_lowsetup
 *
 * Desregvaliption:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void n32_lowsetup(void)
{
#if defined(HAVE_UART)
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t regval;
  uint32_t tmp;
#endif

#if defined(HAVE_CONSOLE)
  /* Enable USART APB1/2 clock */

  modifyreg32(N32_CONSOLE_APBREG, 0, N32_CONSOLE_APBEN);
#endif

  /* Enable the console USART and configure GPIO pins needed for rx/tx.
   *
   * NOTE: Clocking for selected U[S]ARTs was already provided in n32_rcc.c
   */

#ifdef N32_CONSOLE_TX
  n32_configgpio(N32_CONSOLE_TX);
#endif
#ifdef N32_CONSOLE_RX
  n32_configgpio(N32_CONSOLE_RX);
#endif

  /* Enable and configure the selected console device */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Configure CTRL2 */

  regval  = getreg32(N32_CONSOLE_BASE + N32_USART_CTRL2_OFFSET);
  regval &= ~USART_CTRL2_CLRBITS;
  regval |= USART_CTRL2_SETBITS;
  putreg32(regval, N32_CONSOLE_BASE + N32_USART_CTRL2_OFFSET);

  /* Configure CTRL1 */

  regval  = getreg32(N32_CONSOLE_BASE + N32_USART_CTRL1_OFFSET);
  regval &= ~USART_CTRL1_CLRBITS;
  regval |= USART_CTRL1_SETBITS;
#ifdef USE_OVER8
  regval |= USART_CTRL1_OSPM;
#endif
  putreg32(regval, N32_CONSOLE_BASE + N32_USART_CTRL1_OFFSET);

  /* Configure CTRL3 */

  regval  = getreg32(N32_CONSOLE_BASE + N32_USART_CTRL3_OFFSET);
  regval &= ~USART_CTRL3_CLRBITS;
  regval |= USART_CTRL3_SETBITS;
  putreg32(regval, N32_CONSOLE_BASE + N32_USART_CTRL3_OFFSET);

  /* Configure the USART Baud Rate */

  regval  = (N32_UART_DIV / 100) << 4;
  tmp     = N32_UART_DIV - ((regval >> 4) * 100);
#ifdef USE_OVER8
  tmp = ((((tmp * 8) + 50) / 100)) & ((uint8_t)0x0f);
  if (tmp == 0x08)
    {
      regval  = regval | 0x10;
    }
  else
    {
      regval |= tmp;
    }
#else

  /* Oversampling mode is 16 Samples */

  regval += ((((tmp * 16) + 50) / 100)) & ((uint8_t)0x1f);
#endif
  putreg32(regval, N32_CONSOLE_BASE + N32_USART_BRCF_OFFSET);

  /* Enable Rx, Tx and the USART */

  regval  = getreg32(N32_CONSOLE_BASE + N32_USART_CTRL1_OFFSET);
  regval |= (USART_CTRL1_UEN | USART_CTRL1_TXEN | USART_CTRL1_RXEN);
  putreg32(regval, N32_CONSOLE_BASE + N32_USART_CTRL1_OFFSET);

#endif /* HAVE_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
