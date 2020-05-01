/****************************************************************************
 * arch/arm/src/efm32/efm32_lowputc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <arch/board/board.h>

#include "arm_arch.h"

#include "hardware/efm32_memorymap.h"
#include "hardware/efm32_usart.h"
#include "hardware/efm32_leuart.h"
#include "hardware/efm32_cmu.h"

#include "efm32_gpio.h"
#include "efm32_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Console U[S]ART base address */

#if defined(HAVE_UART_CONSOLE)
#  if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_BASE   EFM32_USART0_BASE
#    define CONSOLE_BAUD   CONFIG_USART0_BAUD
#    define CONSOLE_PARITY CONFIG_USART0_PARITY
#    define CONSOLE_NBITS  CONFIG_UART0_BITS
#    define CONSOLE_2STOP  CONFIG_UART0_2STOP
#  elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define CONSOLE_BASE   EFM32_USART1_BASE
#    define CONSOLE_BAUD   CONFIG_USART1_BAUD
#    define CONSOLE_PARITY CONFIG_USART1_PARITY
#    define CONSOLE_NBITS  CONFIG_UART1_BITS
#    define CONSOLE_2STOP  CONFIG_UART1_2STOP
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define CONSOLE_BASE   EFM32_USART2_BASE
#    define CONSOLE_BAUD   CONFIG_USART2_BAUD
#    define CONSOLE_PARITY CONFIG_USART2_PARITY
#    define CONSOLE_NBITS  CONFIG_UART2_BITS
#    define CONSOLE_2STOP  CONFIG_UART2_2STOP
#  elif defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_BASE   EFM32_UART0_BASE
#    define CONSOLE_BAUD   CONFIG_UART0_BAUD
#    define CONSOLE_PARITY CONFIG_UART0_PARITY
#    define CONSOLE_NBITS  CONFIG_UART0_BITS
#    define CONSOLE_2STOP  CONFIG_UART0_2STOP
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_BASE   EFM32_UART1_BASE
#    define CONSOLE_BAUD   CONFIG_UART1_BAUD
#    define CONSOLE_PARITY CONFIG_UART1_PARITY
#    define CONSOLE_NBITS  CONFIG_UART1_BITS
#    define CONSOLE_2STOP  CONFIG_UART1_2STOP
#  else
#    error No U[S]ART console is selected????  Internal craziness!!!
#  endif
#elif defined(HAVE_LEUART_CONSOLE)
#  if defined(CONFIG_LEUART0_SERIAL_CONSOLE)
#    define CONSOLE_BASE   EFM32_LEUART0_BASE
#    define CONSOLE_BAUD   CONFIG_LEUART0_BAUD
#    define CONSOLE_PARITY CONFIG_LEUART0_PARITY
#    define CONSOLE_NBITS  CONFIG_LEUART0_BITS
#    define CONSOLE_2STOP  CONFIG_LEUART0_2STOP
#  elif defined(CONFIG_LEUART1_SERIAL_CONSOLE)
#    define CONSOLE_BASE   EFM32_LEUART1_BASE
#    define CONSOLE_BAUD   CONFIG_LEUART1_BAUD
#    define CONSOLE_PARITY CONFIG_LEUART1_PARITY
#    define CONSOLE_NBITS  CONFIG_LEUART1_BITS
#    define CONSOLE_2STOP  CONFIG_LEUART1_2STOP
#  else
#    error No U[S]ART console is selected????  Internal craziness!!!
#  endif
#endif /* HAVE_UART_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_uart_setbaud
 *
 * Description:
 *   Set optimal oversampling and set the baud for this U[S]ART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static void efm32_uart_setbaud(uintptr_t base,  uint32_t baud)
{
  uint64_t clkdiv;
  uint64_t maxover;
  uint32_t oversample;
  uint32_t regval;
  uint32_t ovs;

  /* Select oversampling.  We would like to oversample at the standard value
   * of 16, but we may not be able to achieve the baud with sufficient
   * accuracy if the baud is close to the HFPERCLK frequency.
   *
   * USART baud is generated according to:
   *
   *   baud = fHFPERCLK/(oversample * (1 + CLKDIV/256))
   *
   * Or, equivalently:
   *
   *   CLKDIV     = 256 * (fHFPERCLK / (oversample * baud) - 1)
   *   oversample = 256 * fHFPERCLK / (baud * (CLKDIV + 256))
   *
   * Suppose we insist on a CLKDIV >= 24, then:
   *
   *   MAXoversample = 256 * fHFPERCLK / (280 * baud))
   *
   * Example 1: fHPERCLK = 32MHz, baud=115200
   *   MAXoversample = 254.0, Use oversample = 16
   *   CLKDIV        = 4188.4
   *   baud          = 115200.0
   * Example 2: fHPERCLK = 32.768KHz, baud=2400
   *   MAXoversample = 12.5, Use oversample = 8
   *   CLKDIV        = 180.90
   *   baud          = 2400.0
   */

  maxover = (((uint64_t)BOARD_HFPERCLK_FREQUENCY << 8) / 280) / baud;
  if (maxover >= 16)
    {
      DEBUGASSERT(baud <= (BOARD_HFPERCLK_FREQUENCY / 16));
      oversample = 16;
      ovs        = USART_CTRL_OVS_X16;
    }
  else if (maxover >= 8)
    {
      DEBUGASSERT(baud <= (BOARD_HFPERCLK_FREQUENCY / 8));
      oversample = 8;
      ovs        = USART_CTRL_OVS_X8;
    }
  else if (maxover >= 6)
    {
      DEBUGASSERT(baud <= (BOARD_HFPERCLK_FREQUENCY / 6));
      oversample = 6;
      ovs        = USART_CTRL_OVS_X6;
    }
  else /* if (maxover >= 4) */
    {
      DEBUGASSERT(maxover >= 4 && baud <= (BOARD_HFPERCLK_FREQUENCY / 4));
      oversample = 4;
      ovs        = USART_CTRL_OVS_X4;
    }

  /* CLKDIV in asynchronous mode is given by:
   *
   *   CLKDIV = 256 * (fHFPERCLK / (oversample * baud) - 1)
   *
   * or
   *
   *   CLKDIV = (256 * fHFPERCLK) / (oversample * baud) - 256
   */

  clkdiv = ((uint64_t)BOARD_HFPERCLK_FREQUENCY << 8) / ((uint64_t)baud * oversample);
  if (clkdiv > 256)
    {
      clkdiv -= 256;
    }
  else
    {
      clkdiv = 0;
    }

  /* Set up the selected oversampling */

  regval  = getreg32(base + EFM32_USART_CTRL_OFFSET);
  regval &= ~_USART_CTRL_OVS_MASK;
  regval |= ovs;
  putreg32(regval, base + EFM32_USART_CTRL_OFFSET);

  /* Set up the selected baud divisor.   The computation above already took
   * in account of _USART_CLKDIV_DIV_SHIFT
   */

  regval = (uint32_t)clkdiv & _USART_CLKDIV_MASK;
  putreg32(regval, base + EFM32_USART_CLKDIV_OFFSET);
}
#endif

/****************************************************************************
 * Name: efm32_leuart_setbaud
 *
 * Description:
 *   Set optimal oversampling and set the baud for this U[S]ART.
 *
 ****************************************************************************/

#ifdef HAVE_LEUART_DEVICE
static void efm32_leuart_setbaud(uintptr_t base,  uint32_t baud)
{
  uint32_t clkdiv;

  /* Baud is configured by:
   *
   *   baud = fLEUART / (1 + CLKDIV / 256)
   *
   * Or
   *
   *   CLKDIV = 256 * (fLEUART / baud - 1)
   *   CLKDIV = (256 * fLEUART) / baud - 256
   */

  clkdiv = (BOARD_LFBCLK_FREQUENCY << 8) / baud;
  if (clkdiv > 256)
    {
      clkdiv -= 256;
    }
  else
    {
      clkdiv = 0;
    }

  DEBUGASSERT(clkdiv <= _LEUART_CLKDIV_MASK);

  /* Set up the selected baud */

  putreg32((uint32_t)clkdiv & _LEUART_CLKDIV_DIV_MASK,
           base + EFM32_LEUART_CLKDIV_OFFSET);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void efm32_lowsetup(void)
{
#if defined(HAVE_UART_DEVICE) || defined(HAVE_LEUART_DEVICE) || \
    defined(HAVE_SPI_DEVICE)
  uint32_t regval;
#endif

#if defined(HAVE_UART_DEVICE) || defined(HAVE_SPI_DEVICE)
  /* Enable clocking to configured UART/USART interfaces */

  regval = getreg32(EFM32_CMU_HFPERCLKEN0);
  regval &= ~(CMU_HFPERCLKEN0_USART0
             | CMU_HFPERCLKEN0_USART1
#ifdef CONFIG_EFM32_HAVE_USART2
             | CMU_HFPERCLKEN0_USART2
#endif
#ifdef CONFIG_EFM32_HAVE_UART0
             | CMU_HFPERCLKEN0_UART0
#endif
#ifdef CONFIG_EFM32_HAVE_UART1
             | CMU_HFPERCLKEN0_UART1
#endif
             );

#ifdef CONFIG_EFM32_USART0
  regval |= CMU_HFPERCLKEN0_USART0;
#endif

#ifdef CONFIG_EFM32_USART1
  regval |= CMU_HFPERCLKEN0_USART1;
#endif

#ifdef CONFIG_EFM32_USART2
  regval |= CMU_HFPERCLKEN0_USART2;
#endif

#ifdef CONFIG_EFM32_UART0
  regval |= CMU_HFPERCLKEN0_UART0;
#endif

#ifdef CONFIG_EFM32_UART1
  regval |= CMU_HFPERCLKEN0_UART1;
#endif

  putreg32(regval, EFM32_CMU_HFPERCLKEN0);
#endif /* HAVE_UART_DEVICE */

#ifdef HAVE_LEUART_DEVICE
  /* Enable the LE interface clock must be enabled in CMU_HFCORECLKEN0 */

  regval  = getreg32(EFM32_CMU_HFCORECLKEN0);
  regval |= CMU_HFCORECLKEN0_LE;
  putreg32(regval, EFM32_CMU_HFCORECLKEN0);

  /* Enable clocking to configured LEUART interfaces */

  regval  = getreg32(EFM32_CMU_LFBCLKEN0);
  regval &= ~(CMU_LFBCLKEN0_LEUART0
#ifdef CONFIG_EFM32_LEUART1
             | CMU_LFBCLKEN0_LEUART1
#endif
             );

#ifdef CONFIG_EFM32_LEUART0
  regval |= CMU_LFBCLKEN0_LEUART0;
#endif

#ifdef CONFIG_EFM32_LEUART1
  regval |= CMU_LFBCLKEN0_LEUART1;
#endif

  putreg32(regval, EFM32_CMU_LFBCLKEN0);
#endif /* HAVE_LEUART_DEVICE */

#if defined(HAVE_UART_DEVICE) || defined(HAVE_SPI_DEVICE)
  /* Enable output on U[S]ART output pins */

#ifdef CONFIG_EFM32_USART0
  efm32_configgpio(GPIO_INPUT | GPIO_INT_NONE | BOARD_USART0_RX_GPIO);
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_USART0_TX_GPIO);
#ifdef CONFIG_EFM32_USART0_ISSPI
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_USART0_CLK_GPIO);
#endif
#endif

#ifdef CONFIG_EFM32_USART1
  efm32_configgpio(GPIO_INPUT | GPIO_INT_NONE | BOARD_USART1_RX_GPIO);
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_USART1_TX_GPIO);
#ifdef CONFIG_EFM32_USART1_ISSPI
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_USART1_CLK_GPIO);
#endif
#endif

#ifdef CONFIG_EFM32_USART2
  efm32_configgpio(GPIO_INPUT | GPIO_INT_NONE | BOARD_USART2_RX_GPIO);
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_USART2_TX_GPIO);
#ifdef CONFIG_EFM32_USART2_ISSPI
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_USART2_CLK_GPIO);
#endif
#endif

#ifdef CONFIG_EFM32_UART0
  efm32_configgpio(GPIO_INPUT | GPIO_INT_NONE | BOARD_UART0_RX_GPIO);
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_UART0_TX_GPIO);
#endif

#ifdef CONFIG_EFM32_UART1
  efm32_configgpio(GPIO_INPUT | GPIO_INT_NONE | BOARD_UART1_RX_GPIO);
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_UART1_TX_GPIO);
#endif
#endif /* HAVE_UART_DEVICE */

#ifdef HAVE_LEUART_DEVICE
  /* Enable output on LEUART output pins */

#ifdef CONFIG_EFM32_LEUART0
  efm32_configgpio(GPIO_INPUT | GPIO_INT_NONE | BOARD_LEUART0_RX_GPIO);
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_LEUART0_TX_GPIO);
#endif

#ifdef CONFIG_EFM32_LEUART1
  efm32_configgpio(GPIO_INPUT | GPIO_INT_NONE | BOARD_LEUART1_RX_GPIO);
  efm32_configgpio(GPIO_OUTPUT_PUSHPULL | GPIO_OUTPUT_CLEAR |
                   GPIO_DRIVE_STANDARD | BOARD_LEUART1_TX_GPIO);
#endif
#endif /* HAVE_LEUART_DEVICE */

#if defined(HAVE_UART_DEVICE) || defined(HAVE_SPI_DEVICE)
  /* Set location in the U[S]ART ROUTE registers */

#ifdef CONFIG_EFM32_USART0
  regval = (USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
           (BOARD_USART0_ROUTE_LOCATION << _USART_ROUTE_LOCATION_SHIFT));
#ifdef CONFIG_EFM32_USART0_ISSPI
  regval |= USART_ROUTE_CLKPEN;
#endif
  putreg32(regval, EFM32_USART0_ROUTE);
#endif

#ifdef CONFIG_EFM32_USART1
  regval = (USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
           (BOARD_USART1_ROUTE_LOCATION << _USART_ROUTE_LOCATION_SHIFT));
#ifdef CONFIG_EFM32_USART1_ISSPI
  regval |= USART_ROUTE_CLKPEN;
#endif
  putreg32(regval, EFM32_USART1_ROUTE);
#endif

#ifdef CONFIG_EFM32_USART2
  regval = (USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
           (BOARD_USART2_ROUTE_LOCATION << _USART_ROUTE_LOCATION_SHIFT));
#ifdef CONFIG_EFM32_USART2_ISSPI
  regval |= USART_ROUTE_CLKPEN;
#endif
  putreg32(regval, EFM32_USART2_ROUTE);
#endif

#ifdef CONFIG_EFM32_UART0
  regval = (USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
           (BOARD_UART0_ROUTE_LOCATION << _USART_ROUTE_LOCATION_SHIFT));
  putreg32(regval, EFM32_UART0_ROUTE);
#endif

#ifdef CONFIG_EFM32_UART1
  regval = (USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
           (BOARD_UART1_ROUTE_LOCATION << _USART_ROUTE_LOCATION_SHIFT));
  putreg32(regval, EFM32_UART1_ROUTE);
#endif
#endif /* HAVE_UART_DEVICE */

#ifdef HAVE_LEUART_DEVICE
  /* Set location in the LEUART ROUTE registers */

#ifdef CONFIG_EFM32_LEUART0
  regval = (LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN |
           (BOARD_LEUART0_ROUTE_LOCATION << _LEUART_ROUTE_LOCATION_SHIFT));
  putreg32(regval, EFM32_LEUART0_ROUTE);
#endif

#ifdef CONFIG_EFM32_LEUART1
  regval = (LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN |
           (BOARD_LEUART1_ROUTE_LOCATION << _LEUART_ROUTE_LOCATION_SHIFT));
  putreg32(regval, EFM32_LEUART1_ROUTE);
#endif
#endif /* HAVE_LEUART_DEVICE */

#if defined(HAVE_UART_CONSOLE)
  /* Configure the U[S]ART serial console */

  efm32_uartconfigure(CONSOLE_BASE, CONSOLE_BAUD, CONSOLE_PARITY,
                      CONSOLE_NBITS, CONSOLE_2STOP);

#elif defined(HAVE_LEUART_CONSOLE)
  /* Configure the LEUART serial console */

  efm32_leuartconfigure(CONSOLE_BASE, CONSOLE_BAUD, CONSOLE_PARITY,
                        CONSOLE_NBITS, CONSOLE_2STOP);

#endif
}

/****************************************************************************
 * Name: efm32_lowputc
 *
 * Description:
 *   Output one character to the UART using a simple polling method.
 *
 ****************************************************************************/

#if defined(HAVE_UART_CONSOLE)
void efm32_lowputc(uint32_t ch)
{
  /* The TX Buffer Level (TXBL) status bit indicates the level of the
   * transmit buffer.  If TXBIL is set, TXBL is set whenever the transmit
   * buffer is half-full or empty.
   */

  while ((getreg32(CONSOLE_BASE + EFM32_USART_STATUS_OFFSET) & USART_STATUS_TXBL) == 0);

  /* Then send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE + EFM32_USART_TXDATA_OFFSET);
}

#elif defined(HAVE_LEUART_CONSOLE)
void efm32_lowputc(uint32_t ch)
{
  /* The TX Buffer Level (TXBL) status bit indicates the level of the
   * transmit buffer.  If TXBIL is set, TXBL is set whenever the transmit
   * buffer is half-full or empty.
   */

  while ((getreg32(CONSOLE_BASE + EFM32_LEUART_STATUS_OFFSET) & LEUART_STATUS_TXBL) == 0);

  /* Then send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE + EFM32_LEUART_TXDATA_OFFSET);
}
#endif

/****************************************************************************
 * Name: efm32_uartconfigure
 *
 * Description:
 *   Configure a U[S]ART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void efm32_uartconfigure(uintptr_t base, uint32_t baud, unsigned int parity,
                         unsigned int nbits, bool stop2)
{
  uint32_t regval = 0;

  /* Make sure that the U[S]ART registers are in the reset state (except for
   * ROUTE information which must be preserved).
   */

  efm32_uart_reset(base);

  /* Configure number of data bits */

  switch (nbits)
    {
    case 4:
      regval |= USART_FRAME_DATABITS_FOUR;
      break;

    case 5:
      regval |= USART_FRAME_DATABITS_FIVE;
      break;

    case 6:
      regval |= USART_FRAME_DATABITS_SIX;
      break;

    case 7:
      regval |= USART_FRAME_DATABITS_SEVEN;
      break;

    default:
    case 8:
      regval |= USART_FRAME_DATABITS_EIGHT;
      break;

    case 9:
      regval |= USART_FRAME_DATABITS_NINE;
      break;

    case 10:
      regval |= USART_FRAME_DATABITS_TEN;
      break;

    case 11:
      regval |= USART_FRAME_DATABITS_ELEVEN;
      break;

    case 12:
      regval |= USART_FRAME_DATABITS_TWELVE;
      break;

    case 13:
      regval |= USART_FRAME_DATABITS_THIRTEEN;
      break;

    case 14:
      regval |= USART_FRAME_DATABITS_FOURTEEN;
      break;

    case 15:
      regval |= USART_FRAME_DATABITS_FIFTEEN;
      break;

    case 16:
      regval |= USART_FRAME_DATABITS_SIXTEEN;
      break;
    }

  /* Configure parity */

  switch (parity)
    {
    default:
    case 0:
      regval |= USART_FRAME_PARITY_NONE;
      break;

    case 1:
      regval |= USART_FRAME_PARITY_ODD;
      break;

    case 2:
      regval |= USART_FRAME_PARITY_EVEN;
      break;
    }

  /* Configure stop bits */

  if (stop2)
    {
      regval |= USART_FRAME_STOPBITS_TWO;
    }
  else
    {
      regval |= USART_FRAME_STOPBITS_ONE;
    }

  putreg32(regval, base + EFM32_USART_FRAME_OFFSET);

  /* Set the baud clock divisor */

  efm32_uart_setbaud(base, baud);

  /* Enable the U[S]ART */

  putreg32(USART_CMD_RXEN | USART_CMD_TXEN, base + EFM32_USART_CMD_OFFSET);
}
#endif

/****************************************************************************
 * Name: efm32_leuartconfigure
 *
 * Description:
 *   Configure a LEUART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_LEUART_DEVICE
void efm32_leuartconfigure(uintptr_t base, uint32_t baud, unsigned int parity,
                           unsigned int nbits, bool stop2)
{
  uint32_t regval = 0;

  /* Make sure that the LEUART registers are in the reset state (except for
   * ROUTE information which must be preserved).
   */

  efm32_leuart_reset(base);

  /* Configure number of data bits */

  switch (nbits)
    {
    default:
    case 8:
      regval |= LEUART_CTRL_DATABITS_EIGHT;
      break;

    case 9:
      regval |= LEUART_CTRL_DATABITS_NINE;
      break;
    }

  /* Configure parity */

  switch (parity)
    {
    default:
    case 0:
      regval |= LEUART_CTRL_PARITY_NONE;
      break;

    case 1:
      regval |= LEUART_CTRL_PARITY_ODD;
      break;

    case 2:
      regval |= LEUART_CTRL_PARITY_EVEN;
      break;

    }

  /* Configure stop bits */

  if (stop2)
    {
      regval |= _LEUART_CTRL_STOPBITS_TWO;
    }
  else
    {
      regval |= _LEUART_CTRL_STOPBITS_ONE;
    }

  putreg32(regval, base + EFM32_LEUART_CTRL_OFFSET);

  /* Set the baud clock divisor */

  efm32_leuart_setbaud(base, baud);

  /* Enable the LEUART */

  putreg32(LEUART_CMD_RXEN | LEUART_CMD_TXEN, base + EFM32_LEUART_CMD_OFFSET);
}
#endif

/****************************************************************************
 * Name: efm32_uart_reset
 *
 * Description:
 *   Reset the USART/UART by disabling it and restoring all of the registers
 *   to the initial, reset value.  Only the ROUTE data set by efm32_lowsetup
 *   is preserved.
 *
 ****************************************************************************/

#if defined(HAVE_UART_DEVICE) || defined(HAVE_SPI_DEVICE)
void efm32_uart_reset(uintptr_t base)
{
  putreg32(USART_CMD_RXDIS | USART_CMD_TXDIS | USART_CMD_MASTERDIS |
           USART_CMD_RXBLOCKDIS | USART_CMD_TXTRIDIS | USART_CMD_CLEARTX |
           USART_CMD_CLEARRX, base + EFM32_USART_CMD_OFFSET);
  putreg32(_USART_CTRL_RESETVALUE, base + EFM32_USART_CTRL_OFFSET);
  putreg32(_USART_FRAME_RESETVALUE, base + EFM32_USART_FRAME_OFFSET);
  putreg32(_USART_TRIGCTRL_RESETVALUE, base + EFM32_USART_TRIGCTRL_OFFSET);
  putreg32(_USART_CLKDIV_RESETVALUE, base + EFM32_USART_CLKDIV_OFFSET);
  putreg32(_USART_IEN_RESETVALUE, base + EFM32_USART_IEN_OFFSET);
  putreg32(_USART_IFC_MASK, base + EFM32_USART_IFC_OFFSET);

  putreg32(_USART_IRCTRL_RESETVALUE, base + EFM32_USART_IRCTRL_OFFSET);
#if defined(EFM32_USART_INPUT_OFFSET)
  putreg32(_USART_INPUT_RESETVALUE, base + EFM32_USART_INPUT_OFFSET);
#endif
#if defined(EFM32_USART_I2SCTRL_OFFSET)
  putreg32(_USART_I2SCTRL_RESETVALUE, base + EFM32_USART_I2SCTRL_OFFSET);
#endif
}
#endif

/****************************************************************************
 * Name: efm32_leuart_reset
 *
 * Description:
 *   Reset the USART/UART by disabling it and restoring all of the registers
 *   to the initial, reset value.  Only the ROUTE data set by efm32_lowsetup
 *   is preserved.
 *
 ****************************************************************************/

#ifdef HAVE_LEUART_DEVICE
void efm32_leuart_reset(uintptr_t base)
{
  putreg32(LEUART_CMD_RXDIS | LEUART_CMD_TXDIS | LEUART_CMD_RXBLOCKDIS |
           LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX,
           base + EFM32_LEUART_CMD_OFFSET);
  putreg32(_LEUART_CTRL_RESETVALUE, base + EFM32_LEUART_CTRL_OFFSET);
  putreg32(_LEUART_CLKDIV_RESETVALUE, base + EFM32_LEUART_CLKDIV_OFFSET);
  putreg32(_LEUART_STARTFRAME_RESETVALUE, base + EFM32_LEUART_STARTFRAME_OFFSET);
  putreg32(_LEUART_SIGFRAME_RESETVALUE, base + EFM32_LEUART_SIGFRAME_OFFSET);
  putreg32(_LEUART_IEN_RESETVALUE, base + EFM32_LEUART_IEN_OFFSET);
  putreg32(_LEUART_IFC_MASK, base + EFM32_LEUART_IFC_OFFSET);
  putreg32(_LEUART_PULSECTRL_RESETVALUE, base + EFM32_LEUART_PULSECTRL_OFFSET);
#if defined(EFM32_LEUART_INPUT_OFFSET)
  putreg32(_LEUART_INPUT_RESETVALUE, base + EFM32_LEUART_INPUT_OFFSET);
#endif
}
#endif
