/****************************************************************************
 * arch/arm/src/samv7/sam_lowputc.c
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

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_config.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "sam_lowputc.h"

#include "hardware/sam_uart.h"
#include "hardware/sam_pinmap.h"
#include "hardware/sam_matrix.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef HAVE_SERIAL_CONSOLE

/* BAUD definitions
 *
 * The source clock is selectable and could be one of:
 *
 *   - The peripheral clock
 *   - A division of the peripheral clock, where the divider is product-
 *     dependent, but generally set to 8
 *   - A processor/peripheral independent clock source fully programmable
 *      provided by PMC (PCK)
 *   - The external clock, available on the SCK pin
 *
 * Only the first two options are supported by this driver.  The divided
 * peripheral clock is only used for very low BAUD selections.
 */

#define FAST_USART_CLOCK   BOARD_MCK_FREQUENCY
#define SLOW_USART_CLOCK   (BOARD_MCK_FREQUENCY >> 3)

/* Select USART parameters for the selected console */

#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define SAM_CONSOLE_BASE     SAM_UART0_BASE
#    define SAM_CONSOLE_BAUD     CONFIG_UART0_BAUD
#    define SAM_CONSOLE_BITS     CONFIG_UART0_BITS
#    define SAM_CONSOLE_PARITY   CONFIG_UART0_PARITY
#    define SAM_CONSOLE_2STOP    CONFIG_UART0_2STOP
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define SAM_CONSOLE_BASE     SAM_UART1_BASE
#    define SAM_CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define SAM_CONSOLE_BITS     CONFIG_UART1_BITS
#    define SAM_CONSOLE_PARITY   CONFIG_UART1_PARITY
#    define SAM_CONSOLE_2STOP    CONFIG_UART1_2STOP
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define SAM_CONSOLE_BASE     SAM_UART2_BASE
#    define SAM_CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define SAM_CONSOLE_BITS     CONFIG_UART2_BITS
#    define SAM_CONSOLE_PARITY   CONFIG_UART2_PARITY
#    define SAM_CONSOLE_2STOP    CONFIG_UART2_2STOP
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define SAM_CONSOLE_BASE     SAM_UART3_BASE
#    define SAM_CONSOLE_BAUD     CONFIG_UART3_BAUD
#    define SAM_CONSOLE_BITS     CONFIG_UART3_BITS
#    define SAM_CONSOLE_PARITY   CONFIG_UART3_PARITY
#    define SAM_CONSOLE_2STOP    CONFIG_UART3_2STOP
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define SAM_CONSOLE_BASE     SAM_UART4_BASE
#    define SAM_CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define SAM_CONSOLE_BITS     CONFIG_UART4_BITS
#    define SAM_CONSOLE_PARITY   CONFIG_UART4_PARITY
#    define SAM_CONSOLE_2STOP    CONFIG_UART4_2STOP
#  elif defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define SAM_CONSOLE_BASE     SAM_USART0_BASE
#    define SAM_CONSOLE_BAUD     CONFIG_USART0_BAUD
#    define SAM_CONSOLE_BITS     CONFIG_USART0_BITS
#    define SAM_CONSOLE_PARITY   CONFIG_USART0_PARITY
#    define SAM_CONSOLE_2STOP    CONFIG_USART0_2STOP
#  elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define SAM_CONSOLE_BASE     SAM_USART1_BASE
#    define SAM_CONSOLE_BAUD     CONFIG_USART1_BAUD
#    define SAM_CONSOLE_BITS     CONFIG_USART1_BITS
#    define SAM_CONSOLE_PARITY   CONFIG_USART1_PARITY
#    define SAM_CONSOLE_2STOP    CONFIG_USART1_2STOP
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define SAM_CONSOLE_BASE     SAM_USART2_BASE
#    define SAM_CONSOLE_BAUD     CONFIG_USART2_BAUD
#    define SAM_CONSOLE_BITS     CONFIG_USART2_BITS
#    define SAM_CONSOLE_PARITY   CONFIG_USART2_PARITY
#    define SAM_CONSOLE_2STOP    CONFIG_USART2_2STOP
#  else
#    error "No CONFIG_U[S]ARTn_SERIAL_CONSOLE Setting"
#  endif

/* Select the settings for the mode register */

#  if SAM_CONSOLE_BITS == 5
#    define MR_CHRL_VALUE UART_MR_CHRL_5BITS /* 5 bits */
#  elif SAM_CONSOLE_BITS == 6
#    define MR_CHRL_VALUE UART_MR_CHRL_6BITS  /* 6 bits */
#  elif SAM_CONSOLE_BITS == 7
#    define MR_CHRL_VALUE UART_MR_CHRL_7BITS /* 7 bits */
#  elif SAM_CONSOLE_BITS == 8
#    define MR_CHRL_VALUE UART_MR_CHRL_8BITS /* 8 bits */
#  elif SAM_CONSOLE_BITS == 9 && !defined(CONFIG_UART0_SERIAL_CONSOLE) && \
       !defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define MR_CHRL_VALUE UART_MR_MODE9
#  else
#    error "Invalid number of bits"
#  endif

#  if SAM_CONSOLE_PARITY == 1
#    define MR_PAR_VALUE UART_MR_PAR_ODD
#  elif SAM_CONSOLE_PARITY == 2
#    define MR_PAR_VALUE UART_MR_PAR_EVEN
#  else
#    define MR_PAR_VALUE UART_MR_PAR_NONE
#  endif

#  if SAM_CONSOLE_2STOP != 0
#    define MR_NBSTOP_VALUE UART_MR_NBSTOP_2
#  else
#    define MR_NBSTOP_VALUE UART_MR_NBSTOP_1
#  endif

#  define MR_VALUE (UART_MR_MODE_NORMAL | UART_MR_USCLKS_MCK | \
                    MR_CHRL_VALUE | MR_PAR_VALUE | MR_NBSTOP_VALUE)

#endif /* HAVE_SERIAL_CONSOLE */

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
#ifdef HAVE_SERIAL_CONSOLE
  irqstate_t flags;

  for (; ; )
    {
      /* Wait for the transmitter to be available */

      while ((getreg32(SAM_CONSOLE_BASE + SAM_UART_SR_OFFSET) &
        UART_INT_TXEMPTY) == 0);

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = enter_critical_section();
      if ((getreg32(SAM_CONSOLE_BASE + SAM_UART_SR_OFFSET) &
        UART_INT_TXEMPTY) != 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, SAM_CONSOLE_BASE + SAM_UART_THR_OFFSET);
          leave_critical_section(flags);
          return;
        }

      leave_critical_section(flags);
    }
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#endif
  return ch;
}

/****************************************************************************
 * Name: sam_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void sam_lowsetup(void)
{
#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint64_t divb3;
  uint32_t intpart;
  uint32_t fracpart;
#endif
#if (defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)) || \
    defined(CONFIG_SAMV7_USART1)
  uint32_t regval;
#endif

  /* Enable clocking for all selected UART/USARTs */

#ifdef CONFIG_SAMV7_UART0
  sam_uart0_enableclk();
#endif
#ifdef CONFIG_SAMV7_UART1
  sam_uart1_enableclk();
#endif
#ifdef CONFIG_SAMV7_UART2
  sam_uart2_enableclk();
#endif
#ifdef CONFIG_SAMV7_UART3
  sam_uart3_enableclk();
#endif
#ifdef CONFIG_SAMV7_UART4
  sam_uart4_enableclk();
#endif
#ifdef CONFIG_SAMV7_USART0
  sam_usart0_enableclk();
#endif
#ifdef CONFIG_SAMV7_USART1
  sam_usart1_enableclk();
#endif
#ifdef CONFIG_SAMV7_USART2
  sam_usart2_enableclk();
#endif

  /* Configure UART pins for all selected UART/USARTs */

#ifdef CONFIG_SAMV7_UART0
  sam_configgpio(GPIO_UART0_RXD);
  sam_configgpio(GPIO_UART0_TXD);
#endif

#ifdef CONFIG_SAMV7_UART1
  sam_configgpio(GPIO_UART1_RXD);
  sam_configgpio(GPIO_UART1_TXD);
#endif

#ifdef CONFIG_SAMV7_UART2
  sam_configgpio(GPIO_UART2_RXD);
  sam_configgpio(GPIO_UART2_TXD);
#endif

#ifdef CONFIG_SAMV7_UART3
  sam_configgpio(GPIO_UART3_RXD);
  sam_configgpio(GPIO_UART3_TXD);
#endif

#ifdef CONFIG_SAMV7_UART4
  sam_configgpio(GPIO_UART4_RXD);
  sam_configgpio(GPIO_UART4_TXD);
#endif

#ifdef CONFIG_SAMV7_USART0
  sam_configgpio(GPIO_USART0_RXD);
  sam_configgpio(GPIO_USART0_TXD);
#ifdef CONFIG_USART0_OFLOWCONTROL
  sam_configgpio(GPIO_USART0_CTS);
#endif
#ifdef CONFIG_USART0_IFLOWCONTROL
  sam_configgpio(GPIO_USART0_RTS);
#endif
#endif

#ifdef CONFIG_SAMV7_USART1
  sam_configgpio(GPIO_USART1_RXD);
  sam_configgpio(GPIO_USART1_TXD);
#  ifdef CONFIG_USART1_OFLOWCONTROL
  sam_configgpio(GPIO_USART1_CTS);
#  endif
#  ifdef CONFIG_USART1_IFLOWCONTROL
  sam_configgpio(GPIO_USART1_RTS);
#  endif

  /* To use the USART1 as an USART, the SYSIO Pin4 must be bound to PB4
   * instead of TDI
   */

#  if defined(CONFIG_SAMV7_JTAG_FULL_ENABLE)
#    warning CONFIG_SAMV7_JTAG_FULL_ENABLE is incompatible with CONFIG_SAMV7_USART1.
#    warning The SYSIO Pin4 must be bound to PB4 to use USART1
#  endif

  regval  = getreg32(SAM_MATRIX_CCFG_SYSIO);
  regval |= MATRIX_CCFG_SYSIO_SYSIO4;
  putreg32(regval, SAM_MATRIX_CCFG_SYSIO);

#endif

#ifdef CONFIG_SAMV7_USART2
  sam_configgpio(GPIO_USART2_RXD);
  sam_configgpio(GPIO_USART2_TXD);
#ifdef CONFIG_USART2_OFLOWCONTROL
  sam_configgpio(GPIO_USART2_CTS);
#endif
#ifdef CONFIG_USART2_IFLOWCONTROL
  sam_configgpio(GPIO_USART2_RTS);
#endif
#endif

  /* Configure the console (only) */
#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Reset and disable receiver and transmitter */

  putreg32((UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS),
           SAM_CONSOLE_BASE + SAM_UART_CR_OFFSET);

  /* Disable all interrupts */

  putreg32(0xffffffff, SAM_CONSOLE_BASE + SAM_UART_IDR_OFFSET);

  /* Set up the mode register */

  putreg32(MR_VALUE, SAM_CONSOLE_BASE + SAM_UART_MR_OFFSET);

  /* Configure the console baud:
   *
   *   Fbaud   = USART_CLOCK / (16 * divisor)
   *   divisor = USART_CLOCK / (16 * Fbaud)
   *
   * NOTE: Oversampling by 8 is not supported. This may limit BAUD rates
   * for lower USART clocks.
   */

  divb3    = ((FAST_USART_CLOCK + (SAM_CONSOLE_BAUD << 3)) << 3) /
             (SAM_CONSOLE_BAUD << 4);
  intpart  = (divb3 >> 3);
  fracpart = (divb3 & 7);

  /* Retain the fast MR peripheral clock UNLESS unless using that clock
   * would result in an excessively large divider.
   *
   * REVISIT: The fractional divider is not used.
   */

  if ((intpart & ~UART_BRGR_CD_MASK) != 0)
    {
      /* Use the divided USART clock */

      divb3    = ((SLOW_USART_CLOCK + (SAM_CONSOLE_BAUD << 3)) << 3) /
                 (SAM_CONSOLE_BAUD << 4);
      intpart  = (divb3 >> 3);
      fracpart = (divb3 & 7);

      /* Re-select the clock source */

      regval  = getreg32(SAM_CONSOLE_BASE + SAM_UART_MR_OFFSET);
      regval &= ~UART_MR_USCLKS_MASK;
      regval |= UART_MR_USCLKS_MCKDIV;
      putreg32(regval, SAM_CONSOLE_BASE + SAM_UART_MR_OFFSET);
    }

  /* Save the BAUD divider (the fractional part is not used for UARTs) */

  regval = UART_BRGR_CD(intpart) | UART_BRGR_FP(fracpart);
  putreg32(regval, SAM_CONSOLE_BASE + SAM_UART_BRGR_OFFSET);

  /* Enable receiver & transmitter */

  putreg32((UART_CR_RXEN | UART_CR_TXEN),
           SAM_CONSOLE_BASE + SAM_UART_CR_OFFSET);
#endif
}
