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

#include "up_arch.h"

#include "chip/efm32_memorymap.h"
#include "chip/efm32_usart.h"
#include "chip/efm32_cmu.h"
#include "efm32_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Console U[S]ART base address */

#ifdef HAVE_SERIAL_CONSOLE
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
#    error No console is selected????  Internal craziness!!!
#  endif
#endif /* HAVE_SERIAL_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_setbaud
 *
 * Description:
 *   Set optimal oversampling and set the baud for this U[S]ART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static void efm32_setbaud(uintptr_t base,  uint32_t baud)
{
  uint64_t clkdiv;
  uint32_t oversample;
  uint32_t regval;
  uint32_t ovs;

  /* Select oversampling */

  if (baud <= (BOARD_HFPERCLK_FREQUENCY / 4))
    {
      oversample = 16;
      ovs        = USART_CTRL_OVS_X16;
    }
  else if (baud <= (BOARD_HFPERCLK_FREQUENCY / 6))
    {
      oversample = 16;
      ovs        = USART_CTRL_OVS_X16;
    }
  else if (baud <= (BOARD_HFPERCLK_FREQUENCY / 8))
    {
      oversample = 8;
      ovs        = USART_CTRL_OVS_X8;
    }
  else /* if (baud <= (BOARD_HFPERCLK_FREQUENCY / 16)) */
    {
      DEBUGASSERT(baud <= (BOARD_HFPERCLK_FREQUENCY / 16));
      oversample = 16;
      ovs        = USART_CTRL_OVS_X16;
    }

  /* CLKDIV in asynchronous mode is given by:
   *
   * CLKDIV = 256 * (fHFPERCLK/(oversample * baud) - 1)
   * or
   * CLKDIV = (256 * fHFPERCLK)/(oversample * baud) - 256
   */

  clkdiv  = ((uint64_t)BOARD_HFPERCLK_FREQUENCY << 8) / ((uint64_t)baud * oversample);
  clkdiv -= 256;

  DEBUGASSERT(clkdiv <= _USART_CLKDIV_MASK);

  /* Set up the select oversampling and baud */

  regval = getreg32(base + EFM32_USART_CTRL_OFFSET);
  regval  &= ~_USART_CTRL_OVS_MASK;
  regval  |= ovs;
  putreg32(regval, base + EFM32_USART_CTRL_OFFSET);

  putreg32((uint32_t)clkdiv & _USART_CTRL_OVS_MASK,
           base + EFM32_USART_CLKDIV_OFFSET);
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
#ifdef HAVE_UART_DEVICE
  uint32_t regval;

  /* Enable clocking to configured UART/USART devices */

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

  /* Set location in the ROUTE register */

#ifdef CONFIG_EFM32_USART0
  regval = (USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
           (BOARD_USART0_ROUTE_LOCATION << _USART_ROUTE_LOCATION_SHIFT));
  putreg32(regval, EFM32_USART0_ROUTE);
#endif

#ifdef CONFIG_EFM32_USART1
  regval = (USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
           (BOARD_USART1_ROUTE_LOCATION << _USART_ROUTE_LOCATION_SHIFT));
  putreg32(regval, EFM32_USART1_ROUTE);
#endif

#ifdef CONFIG_EFM32_USART2
  regval = (USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
           (BOARD_USART2_ROUTE_LOCATION << _USART_ROUTE_LOCATION_SHIFT));
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

#ifdef CONFIG_EFM32_LEUART0
#  warning Missing LEUART0 support
#endif

#ifdef CONFIG_EFM32_LEUART1
#  warning Missing LEUART1 support
#endif

#ifdef HAVE_SERIAL_CONSOLE
  /* Configure the serial console */

  efm32_uartconfigure(CONSOLE_BASE, CONSOLE_BAUD, CONSOLE_PARITY,
                      CONSOLE_NBITS, CONSOLE_2STOP);
#endif
}

/*****************************************************************************
 * Name: efm32_lowputc
 *
 * Description:
 *   Output one character to the UART using a simple polling method.
 *
 *****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
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
#endif

/*****************************************************************************
 * Name: efm32_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 *****************************************************************************/

#ifdef HAVE_UART_DEVICE
void efm32_uartconfigure(uintptr_t base, uint32_t baud, unsigned int parity,
                         unsigned int nbits, bool stop2)
{
  uint32_t regval = 0;

  /* Make sure that the U[S]ART registers are in the reset state (except for
   * ROUTE information which must be preserved).
   */

  efm32_uartreset(base);

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

  efm32_setbaud(base, baud);

  /* Enable the U[S]ART */

  putreg32(USART_CMD_RXEN | USART_CMD_TXEN, base + EFM32_USART_CMD_OFFSET);
}
#endif

/*****************************************************************************
 * Name: efm32_uartreset
 *
 * Description:
 *   Reset the USART/UART by disabling it and restoring all of the registers
 *   to the initial, reset value.  Only the ROUTE data set by efm32_lowsetup
 *   is preserved.
 *
 *****************************************************************************/

#ifdef HAVE_UART_DEVICE
void efm32_uartreset(uintptr_t base)
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
