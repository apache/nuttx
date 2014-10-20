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
void efm32_uartconfigure(uintptr_t uart_base, uint32_t baud,
                         unsigned int parity, unsigned int nbits, bool stop2)
{
#warning Missing logic
}
#endif
