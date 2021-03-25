/****************************************************************************
 * arch/arm/src/str71x/str71x_lowputc.c
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

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "str71x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Is there a UART enabled? */

#if defined(CONFIG_STR71X_UART0) || defined(CONFIG_STR71X_UART1) || \
    defined(CONFIG_STR71X_UART2) || defined(CONFIG_STR71X_UART3)
#  define HAVE_UART 1

/* Is there a serial console? */

#  if defined(CONFIG_UART0_SERIAL_CONSOLE) || defined(CONFIG_UART1_SERIAL_CONSOLE) ||\
     defined(CONFIG_UART2_SERIAL_CONSOLE) || defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define HAVE_CONSOLE
#  else
#    undef HAVE_CONSOLE
#  endif

#else
#  undef HAVE_UART
#  undef HAVE_CONSOLE
#endif

/* GPIO0 UART configuration bits.  For each enabled UART:
 *
 *   TX needs to be configured for alternate function, push-pull {1, 1, 1}
 *   RX needs to be configured for input, tristate, cmos {0, 1, 0}
 */

#ifdef CONFIG_STR71X_UART0
#  define STR71X_UART0_GPIO0_MASK    (0x0300)        /* P0.8->U0.TX, B0.9->U0.RX */
#  define STR71X_UART0_GPIO0_PC0BITS (0x0200)
#  define STR71X_UART0_GPIO0_PC1BITS (0x0300)
#  define STR71X_UART0_GPIO0_PC2BITS (0x0200)
#else
#  define STR71X_UART0_GPIO0_MASK    (0)
#  define STR71X_UART0_GPIO0_PC0BITS (0)
#  define STR71X_UART0_GPIO0_PC1BITS (0)
#  define STR71X_UART0_GPIO0_PC2BITS (0)
#endif

#ifdef CONFIG_STR71X_UART1
#  define STR71X_UART1_GPIO0_MASK    (0x0c00)        /* P0,10->U1.RX, P0.11->U1.TX */
#  define STR71X_UART1_GPIO0_PC0BITS (0x0800)
#  define STR71X_UART1_GPIO0_PC1BITS (0x0c00)
#  define STR71X_UART1_GPIO0_PC2BITS (0x0800)
#else
#  define STR71X_UART1_GPIO0_MASK    (0)
#  define STR71X_UART1_GPIO0_PC0BITS (0)
#  define STR71X_UART1_GPIO0_PC1BITS (0)
#  define STR71X_UART1_GPIO0_PC2BITS (0)
#endif

#ifdef CONFIG_STR71X_UART2
#  define STR71X_UART2_GPIO0_MASK    (0x6000)        /* P0.13->U2.RX, P0.14>U2.TX */
#  define STR71X_UART2_GPIO0_PC0BITS (0x4000)
#  define STR71X_UART2_GPIO0_PC1BITS (0x6000)
#  define STR71X_UART2_GPIO0_PC2BITS (0x4000)
#else
#  define STR71X_UART2_GPIO0_MASK    (0)
#  define STR71X_UART2_GPIO0_PC0BITS (0)
#  define STR71X_UART2_GPIO0_PC1BITS (0)
#  define STR71X_UART2_GPIO0_PC2BITS (0)
#endif

#ifdef CONFIG_STR71X_UART3
#  define STR71X_UART3_GPIO0_MASK    (0x0003)        /* P0.0->U3.TX, P0.1->U3.RX */
#  define STR71X_UART3_GPIO0_PC0BITS (0x0001)
#  define STR71X_UART3_GPIO0_PC1BITS (0x0003)
#  define STR71X_UART3_GPIO0_PC2BITS (0x0001)
#else
#  define STR71X_UART3_GPIO0_MASK    (0)
#  define STR71X_UART3_GPIO0_PC0BITS (0)
#  define STR71X_UART3_GPIO0_PC1BITS (0)
#  define STR71X_UART3_GPIO0_PC2BITS (0)
#endif

#define STR71X_UART_GPIO0_MASK \
  (STR71X_UART0_GPIO0_MASK    | STR71X_UART1_GPIO0_MASK | \
   STR71X_UART2_GPIO0_MASK    | STR71X_UART3_GPIO0_MASK)
#define STR71X_UART_GPIO0_PC0BITS \
  (STR71X_UART0_GPIO0_PC0BITS | STR71X_UART1_GPIO0_PC0BITS | \
   STR71X_UART2_GPIO0_PC0BITS | STR71X_UART3_GPIO0_PC0BITS)
#define STR71X_UART_GPIO0_PC1BITS \
  (STR71X_UART0_GPIO0_PC1BITS | STR71X_UART1_GPIO0_PC1BITS | \
   STR71X_UART2_GPIO0_PC1BITS | STR71X_UART3_GPIO0_PC1BITS)
#define STR71X_UART_GPIO0_PC2BITS \
  (STR71X_UART0_GPIO0_PC2BITS | STR71X_UART1_GPIO0_PC2BITS | \
   STR71X_UART2_GPIO0_PC2BITS | STR71X_UART3_GPIO0_PC2BITS)

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define STR71X_UART_BASE     STR71X_UART0_BASE
#  define STR71X_UART_BAUD     CONFIG_UART0_BAUD
#  define STR71X_UART_BITS     CONFIG_UART0_BITS
#  define STR71X_UART_PARITY   CONFIG_UART0_PARITY
#  define STR71X_UART_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define STR71X_UART_BASE     STR71X_UART1_BASE
#  define STR71X_UART_BAUD     CONFIG_UART1_BAUD
#  define STR71X_UART_BITS     CONFIG_UART1_BITS
#  define STR71X_UART_PARITY   CONFIG_UART1_PARITY
#  define STR71X_UART_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define STR71X_UART_BASE     STR71X_UART2_BASE
#  define STR71X_UART_BAUD     CONFIG_UART2_BAUD
#  define STR71X_UART_BITS     CONFIG_UART2_BITS
#  define STR71X_UART_PARITY   CONFIG_UART2_PARITY
#  define STR71X_UART_2STOP    CONFIG_UART2_2STOP
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define STR71X_UART_BASE     STR71X_UART3_BASE
#  define STR71X_UART_BAUD     CONFIG_UART3_BAUD
#  define STR71X_UART_BITS     CONFIG_UART3_BITS
#  define STR71X_UART_PARITY   CONFIG_UART3_PARITY
#  define STR71X_UART_2STOP    CONFIG_UART3_2STOP
#else
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Get mode setting */

#if STR71X_UART_BITS == 7
#  if STR71X_UART_PARITY == 0
#    error "7-bits, no parity mode not supported"
#  else
#    define STR71X_UARTCR_MODE STR71X_UARTCR_MODE7BITP
#  endif
#elif STR71X_UART_BITS == 8
#  if STR71X_UART_PARITY == 0
#    define STR71X_UARTCR_MODE STR71X_UARTCR_MODE8BIT
#  else
#    define STR71X_UARTCR_MODE STR71X_UARTCR_MODE8BITP
#  endif
#elif STR71X_UART_BITS == 9
#  if STR71X_UART_PARITY == 0
#    define STR71X_UARTCR_MODE STR71X_UARTCR_MODE9BIT
#  else
#    error "9-bits with parity not supported"
#  endif
#else
#  error "Number of bits not supported"
#endif

#if STR71X_UART_PARITY == 0 || STR71X_UART_PARITY == 2
#  define STR71X_UARTCR_PARITY (0)
#elif STR71X_UART_PARITY == 1
#  define STR71X_UARTCR_PARITY STR71X_UARTCR_PARITYODD
#else
#  error "Invalid parity selection"
#endif

#if STR71X_UART_2STOP != 0
#  define STR71X_UARTCR_STOP STR71X_UARTCR_STOPBIT20
#else
#  define STR71X_UARTCR_STOP STR71X_UARTCR_STOPBIT10
#endif

#define STR71X_UARTCR_VALUE \
  (STR71X_UARTCR_MODE | STR71X_UARTCR_PARITY   | STR71X_UARTCR_STOP | \
   STR71X_UARTCR_RUN  | STR71X_UARTCR_RXENABLE | STR71X_UARTCR_FIFOENABLE)

/* Calculate BAUD rate from PCLK1:
 *
 * Example:
 *   PCLK1 = 32MHz
 *   STR71X_UART_BAUD = 38,400
 *   UART_BAUDRATE_DIVISOR = 614,400
 *   UART_BAUDRATE = 52.583 (exact) or 52 truncated (1.1% error)
 */

#define UART_BAUDDIVISOR (16 * STR71X_UART_BAUD)
#define UART_BAUDRATE    ((STR71X_PCLK1 + (UART_BAUDDIVISOR/2)) / UART_BAUDDIVISOR)

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
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  /* Wait until the TX FIFO is not full */

  while ((getreg16(STR71X_UART_SR(STR71X_UART_BASE)) &
          STR71X_UARTSR_TF) != 0);

  /* Then send the character */

  putreg16((uint16_t)ch, STR71X_UART_TXBUFR(STR71X_UART_BASE));
#endif
}

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void up_lowsetup(void)
{
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  uint16_t reg16;

  /* Enable the selected console device */

  /* Set the UART baud rate */

  putreg16((uint16_t)UART_BAUDRATE, STR71X_UART_BR(STR71X_UART_BASE));

  /* Configure the UART control registers */

  putreg16(STR71X_UARTCR_VALUE, STR71X_UART_CR(STR71X_UART_BASE));

  /* Clear FIFOs */

  putreg16(0, STR71X_UART_TXRSTR(STR71X_UART_BASE));
  putreg16(0, STR71X_UART_RXRSTR(STR71X_UART_BASE));
#endif

  /* Configure GPIO0 pins to enable all UARTs in the configuration
   * (the serial driver later depends on this configuration)
   */

#ifdef HAVE_UART
  reg16  = getreg16(STR71X_GPIO0_PC0);
  reg16 &= ~STR71X_UART_GPIO0_MASK;
  reg16 |= STR71X_UART_GPIO0_PC0BITS;
  putreg16(reg16, STR71X_GPIO0_PC0);

  reg16 = getreg16(STR71X_GPIO0_PC1);
  reg16 &= ~STR71X_UART_GPIO0_MASK;
  reg16 |= STR71X_UART_GPIO0_PC1BITS;
  putreg16(reg16, STR71X_GPIO0_PC1);

  reg16 = getreg16(STR71X_GPIO0_PC2);
  reg16 &= ~STR71X_UART_GPIO0_MASK;
  reg16 |= STR71X_UART_GPIO0_PC2BITS;
  putreg16(reg16, STR71X_GPIO0_PC2);
#endif
}
