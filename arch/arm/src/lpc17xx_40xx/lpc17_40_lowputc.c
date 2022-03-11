/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_lowputc.c
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

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/lpc17_40_syscon.h"
#include "hardware/lpc17_40_uart.h"

#include "lpc17_40_gpio.h"
#include "lpc17_40_lowputc.h"
#include "lpc17_40_serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC17_40_UART0_BASE
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC17_40_UART1_BASE
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC17_40_UART2_BASE
#  define CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define CONSOLE_BITS     CONFIG_UART2_BITS
#  define CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define CONSOLE_2STOP    CONFIG_UART2_2STOP
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC17_40_UART3_BASE
#  define CONSOLE_BAUD     CONFIG_UART3_BAUD
#  define CONSOLE_BITS     CONFIG_UART3_BITS
#  define CONSOLE_PARITY   CONFIG_UART3_PARITY
#  define CONSOLE_2STOP    CONFIG_UART3_2STOP
#else
#  if defined(HAVE_CONSOLE)
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#  define CONSOLE_BASE     LPC17_40_UART0_BASE
#  define CONSOLE_BAUD     115200
#  define CONSOLE_BITS     8
#  define CONSOLE_PARITY   0
#  define CONSOLE_2STOP    0
#endif

/* Get word length setting for the console */

#if CONSOLE_BITS == 5
#  define CONSOLE_LCR_WLS UART_LCR_WLS_5BIT
#elif CONSOLE_BITS == 6
#  define CONSOLE_LCR_WLS UART_LCR_WLS_6BIT
#elif CONSOLE_BITS == 7
#  define CONSOLE_LCR_WLS UART_LCR_WLS_7BIT
#elif CONSOLE_BITS == 8
#  define CONSOLE_LCR_WLS UART_LCR_WLS_8BIT
#elif defined(HAVE_CONSOLE)
#  error "Invalid CONFIG_UARTn_BITS setting for console"
#endif

/* Get parity setting for the console */

#if CONSOLE_PARITY == 0
#  define CONSOLE_LCR_PAR 0
#elif CONSOLE_PARITY == 1
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_ODD)
#elif CONSOLE_PARITY == 2
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_EVEN)
#elif CONSOLE_PARITY == 3
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK1)
#elif CONSOLE_PARITY == 4
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK0)
#elif defined(HAVE_CONSOLE)
#  error "Invalid CONFIG_UARTn_PARITY setting for CONSOLE"
#endif

/* Get stop-bit setting for the console and UART0-3 */

#if CONSOLE_2STOP != 0
#  define CONSOLE_LCR_STOP UART_LCR_STOP
#else
#  define CONSOLE_LCR_STOP 0
#endif

/* LCR and FCR values for the console */

#define CONSOLE_LCR_VALUE (CONSOLE_LCR_WLS | CONSOLE_LCR_PAR | CONSOLE_LCR_STOP)
#define CONSOLE_FCR_VALUE (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST |\
                           UART_FCR_RXRST | UART_FCR_FIFOEN)

/* Select a CCLK divider to produce the UART PCLK.  The strategy is to select
 * the smallest divisor that results in an solution within range of the
 * 16-bit DLM and DLL divisor:
 *
 *   BAUD = PCLK / (16 * DL), or
 *   DL   = PCLK / BAUD / 16
 *
 * Where for the LPC176x the PCLK is determined by the UART-specific
 * divisor in PCLKSEL0 or PCLKSEL1:
 *
 *   PCLK = CCLK / divisor
 *
 * And for the LPC178x/40xx, the PCLK is determined by the global divisor
 * setting in the PLKSEL register.
 *
 */

#ifdef LPC178x_40xx
  /* Use the global PCLK frequency */

# define CONSOLE_NUMERATOR BOARD_PCLK_FREQUENCY

#else
#  ifdef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
#    define CONSOLE_CCLKDIV  SYSCON_PCLKSEL_CCLK
#    define CONSOLE_NUMERATOR (LPC17_40_CCLK)
#  else
  /* Calculate and optimal PCLKSEL0/1 divisor.
   * First, check divisor == 1.  This works if the upper limit is met:
   *
   *   DL < 0xffff, or
   *   PCLK / BAUD / 16 < 0xffff, or
   *   CCLK / BAUD / 16 < 0xffff, or
   *   CCLK < BAUD * 0xffff * 16
   *   BAUD > CCLK / 0xffff / 16
   *
   * And the lower limit is met (we can't allow DL to get very close to one).
   *
   *   DL >= MinDL
   *   CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 16 / MinDL
   */

#    if CONSOLE_BAUD < (LPC17_40_CCLK / 16 / UART_MINDL)
#      define CONSOLE_CCLKDIV  SYSCON_PCLKSEL_CCLK
#      define CONSOLE_NUMERATOR (LPC17_40_CCLK)

  /* Check divisor == 2.  This works if:
   *
   *   2 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 8
   *
   * And
   *
   *   2 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 8 / MinDL
   */

#    elif CONSOLE_BAUD < (LPC17_40_CCLK / 8 / UART_MINDL)
#      define CONSOLE_CCLKDIV SYSCON_PCLKSEL_CCLK2
#      define CONSOLE_NUMERATOR (LPC17_40_CCLK / 2)

  /* Check divisor == 4.  This works if:
   *
   *   4 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 4
   *
   * And
   *
   *   4 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 4 / MinDL
   */

#    elif CONSOLE_BAUD < (LPC17_40_CCLK / 4 / UART_MINDL)
#     define CONSOLE_CCLKDIV SYSCON_PCLKSEL_CCLK4
#      define CONSOLE_NUMERATOR (LPC17_40_CCLK / 4)

  /* Check divisor == 8.  This works if:
   *
   *   8 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 2
   *
   * And
   *
   *   8 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 2 / MinDL
  */

#    else /* if CONSOLE_BAUD < (LPC17_40_CCLK / 2 / UART_MINDL) */
#      define CONSOLE_CCLKDIV   SYSCON_PCLKSEL_CCLK8
#      define CONSOLE_NUMERATOR (LPC17_40_CCLK /  8)
#    endif
#  endif
#endif /* LPC178x_40xx */

/* Then this is the value to use for the DLM and DLL registers */

#define CONSOLE_DL (CONSOLE_NUMERATOR / (CONSOLE_BAUD << 4))

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
#if defined HAVE_UART && defined HAVE_CONSOLE
  /* Wait for the transmitter to be available */

  while ((getreg32(CONSOLE_BASE + LPC17_40_UART_LSR_OFFSET) &
          UART_LSR_THRE) == 0);

  /* Send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE + LPC17_40_UART_THR_OFFSET);
#endif
}

/****************************************************************************
 * Name: lpc17_40_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 *   The UART0/1/2/3 peripherals are configured using the following
 *   registers:
 *   1. Power: In the PCONP register, set bits PCUART0/1/2/3.
 *      On reset, UART0 and UART 1 are enabled (PCUART0 = 1 and PCUART1 = 1)
 *      and UART2/3 are disabled (PCUART1 = 0 and PCUART3 = 0).
 *   2. Peripheral clock: In the PCLKSEL0 register, select PCLK_UART0 and
 *      PCLK_UART1; in the PCLKSEL1 register,
 *      select PCLK_UART2 and PCLK_UART3.
 *   3. Baud rate: In the LCR register, set bit DLAB = 1. This enables access
 *      to registers DLL and DLM for setting the baud rate. Also, if needed,
 *      set the fractional baud rate in the fractional divider.
 *   4. UART FIFO: Use bit FIFO enable (bit 0) in FCR register to
 *      enable FIFO.
 *   5. Pins: Select UART pins through the PINSEL registers and pin modes
 *      through the PINMODE registers. UART receive pins should not have
 *      pull-down resistors enabled.
 *   6. Interrupts: To enable UART interrupts set bit DLAB = 0 in the LCRF
 *      register. This enables access to IER. Interrupts are enabled
 *      in the NVIC using the appropriate Interrupt Set Enable register.
 *   7. DMA: UART transmit and receive functions can operate with the
 *      GPDMA controller.
 *
 ****************************************************************************/

void lpc17_40_lowsetup(void)
{
#ifdef HAVE_UART
  uint32_t regval;

  /* Step 1: Enable power for all console UART and disable power for
   * other UARTs
   */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval &= ~(SYSCON_PCONP_PCUART0 | SYSCON_PCONP_PCUART1 |
              SYSCON_PCONP_PCUART2 | SYSCON_PCONP_PCUART3);
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  regval |= SYSCON_PCONP_PCUART0;
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  regval |= SYSCON_PCONP_PCUART1;
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  regval |= SYSCON_PCONP_PCUART2;
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
  regval |= SYSCON_PCONP_PCUART3;
#endif
  putreg32(regval, LPC17_40_SYSCON_PCONP);

/* Step 2: Enable peripheral clocking for the console UART and disable
 * clocking for all other UARTs
 */

#ifdef LPC176x
  regval = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~(SYSCON_PCLKSEL0_UART0_MASK | SYSCON_PCLKSEL0_UART1_MASK);
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  regval |= (CONSOLE_CCLKDIV << SYSCON_PCLKSEL0_UART0_SHIFT);
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  regval |= (CONSOLE_CCLKDIV << SYSCON_PCLKSEL0_UART1_SHIFT);
#endif
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);

  regval = getreg32(LPC17_40_SYSCON_PCLKSEL1);
  regval &= ~(SYSCON_PCLKSEL1_UART2_MASK | SYSCON_PCLKSEL1_UART3_MASK);
#if defined(CONFIG_UART2_SERIAL_CONSOLE)
  regval |= (CONSOLE_CCLKDIV << SYSCON_PCLKSEL1_UART2_SHIFT);
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
  regval |= (CONSOLE_CCLKDIV << SYSCON_PCLKSEL1_UART3_SHIFT);
#endif
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL1);
#endif

  /* Configure UART pins for the selected CONSOLE */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  lpc17_40_configgpio(GPIO_UART0_TXD);
  lpc17_40_configgpio(GPIO_UART0_RXD);
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  lpc17_40_configgpio(GPIO_UART1_TXD);
  lpc17_40_configgpio(GPIO_UART1_RXD);
#if defined(CONFIG_UART1_IFLOWCONTROL) || defined(CONFIG_UART1_OFLOWCONTROL)
  lpc17_40_configgpio(GPIO_UART1_CTS);
  lpc17_40_configgpio(GPIO_UART1_DCD);
  lpc17_40_configgpio(GPIO_UART1_DSR);
  lpc17_40_configgpio(GPIO_UART1_DTR);
  lpc17_40_configgpio(GPIO_UART1_RI);
  lpc17_40_configgpio(GPIO_UART1_RTS);
#endif
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  lpc17_40_configgpio(GPIO_UART2_TXD);
  lpc17_40_configgpio(GPIO_UART2_RXD);
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
  lpc17_40_configgpio(GPIO_UART3_TXD);
  lpc17_40_configgpio(GPIO_UART3_RXD);
#endif

  /* Configure the console (only) */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Clear fifos */

  putreg32(UART_FCR_RXRST | UART_FCR_TXRST,
           CONSOLE_BASE + LPC17_40_UART_FCR_OFFSET);

  /* Set trigger */

  putreg32(UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_8,
           CONSOLE_BASE + LPC17_40_UART_FCR_OFFSET);

#ifndef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
  /* Disable FDR (fractional divider),
   * ignored by baudrate calculation => has to be disabled
   */

  putreg32((1 << UART_FDR_MULVAL_SHIFT) + (0 << UART_FDR_DIVADDVAL_SHIFT),
           CONSOLE_BASE + LPC17_40_UART_FDR_OFFSET);

#endif
  /* Set up the LCR and set DLAB=1 */

  putreg32(CONSOLE_LCR_VALUE | UART_LCR_DLAB,
           CONSOLE_BASE + LPC17_40_UART_LCR_OFFSET);

#ifdef CONFIG_LPC17_40_UART_USE_FRACTIONAL_DIVIDER
  up_setbaud(CONSOLE_BASE, CONSOLE_NUMERATOR, CONSOLE_BAUD);
#else
  /* Set the BAUD divisor */

  putreg32(CONSOLE_DL >> 8, CONSOLE_BASE + LPC17_40_UART_DLM_OFFSET);
  putreg32(CONSOLE_DL & 0xff, CONSOLE_BASE + LPC17_40_UART_DLL_OFFSET);
#endif

  /* Clear DLAB */

  putreg32(CONSOLE_LCR_VALUE, CONSOLE_BASE + LPC17_40_UART_LCR_OFFSET);

  /* Configure the FIFOs */

  putreg32(UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST | UART_FCR_RXRST |
           UART_FCR_FIFOEN,
           CONSOLE_BASE + LPC17_40_UART_FCR_OFFSET);
#endif
#endif /* HAVE_UART */
}
