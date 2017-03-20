/****************************************************************************
 * arch/arm/src/xmc4/xmc4_lowputc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
#include <errno.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "xmc4_config.h"
#include "chip/xmc4_usic.h"
#include "chip/xmc4_pinmux.h"
#include "xmc4_usic.h"
#include "xmc4_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#if defined(HAVE_UART_CONSOLE)
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_CHAN     USIC0_CHAN0
#    define CONSOLE_FREQ     BOARD_CORECLK_FREQ
#    define CONSOLE_BAUD     CONFIG_UART0_BAUD
#    define CONSOLE_BITS     CONFIG_UART0_BITS
#    define CONSOLE_2STOP    CONFIG_UART0_2STOP
#    define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_CHAN     USIC0_CHAN1
#    define CONSOLE_FREQ     BOARD_CORECLK_FREQ
#    define CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define CONSOLE_BITS     CONFIG_UART1_BITS
#    define CONSOLE_2STOP    CONFIG_UART1_2STOP
#    define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define CONSOLE_CHAN     USIC1_CHAN0
#    define CONSOLE_FREQ     BOARD_BUS_FREQ
#    define CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define CONSOLE_BITS     CONFIG_UART2_BITS
#    define CONSOLE_2STOP    CONFIG_UART2_2STOP
#    define CONSOLE_PARITY   CONFIG_UART2_PARITY
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define CONSOLE_CHAN     USIC1_CHAN1
#    define CONSOLE_FREQ     BOARD_BUS_FREQ
#    define CONSOLE_BAUD     CONFIG_UART3_BAUD
#    define CONSOLE_BITS     CONFIG_UART3_BITS
#    define CONSOLE_2STOP    CONFIG_UART3_2STOP
#    define CONSOLE_PARITY   CONFIG_UART3_PARITY
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define CONSOLE_CHAN     USIC2_CHAN0
#    define CONSOLE_FREQ     BOARD_BUS_FREQ
#    define CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define CONSOLE_BITS     CONFIG_UART4_BITS
#    define CONSOLE_2STOP    CONFIG_UART4_2STOP
#    define CONSOLE_PARITY   CONFIG_UART4_PARITY
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define CONSOLE_CHAN     USIC2_CHAN1
#    define CONSOLE_FREQ     BOARD_BUS_FREQ
#    define CONSOLE_BAUD     CONFIG_UART5_BAUD
#    define CONSOLE_BITS     CONFIG_UART5_BITS
#    define CONSOLE_2STOP    CONFIG_UART5_2STOP
#    define CONSOLE_PARITY   CONFIG_UART5_PARITY
#  elif defined(HAVE_UART_CONSOLE)
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#endif /* HAVE_UART_CONSOLE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_UART_CONSOLE
  /* Wait until the transmit data register is "empty" (TDRE).  This state
   * depends on the TX watermark setting and may not mean that the transmit
   * buffer is truly empty.  It just means that we can now add another
   * character to the transmit buffer without exceeding the watermark.
   *
   * NOTE:  UART0 has an 8-byte deep FIFO; the other UARTs have no FIFOs
   * (1-deep).  There appears to be no way to know when the FIFO is not
   * full (other than reading the FIFO length and comparing the FIFO count).
   * Hence, the FIFOs are not used in this implementation and, as a result
   * TDRE indeed mean that the single output buffer is available.
   *
   * Performance on UART0 could be improved by enabling the FIFO and by
   * redesigning all of the FIFO status logic.
   */
#warning Missing logic

  /* Then write the character to the UART data register */

#warning Missing logic
}

/****************************************************************************
 * Name: xmc4_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void xmc4_lowsetup(void)
{
  uint32_t regval;

  /* Enable peripheral clocking for all enabled UARTs. */
#warning Missing logic

  /* Configure UART pins for the all enabled UARTs */

  /* Configure the console (only) now.  Other UARTs will be configured
   * when the serial driver is opened.
   */

  xmc4_uart_configure(CONSOLE_CHAN, CONSOLE_BAUD, CONSOLE_FREQ, \
                      CONSOLE_PARITY, CONSOLE_BITS, CONSOLE_2STOP);
#endif /* HAVE_UART_DEVICE */
}

/****************************************************************************
 * Name: xmc4_uart_reset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void xmc4_uart_reset(uintptr_t uart_base)
{
  uint8_t regval;

  /* Just disable the transmitter and receiver */
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: xmc4_uart_configure
 *
 * Description:
 *   Enable and configure a USIC channel as a RS-232 UART.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
int xmc4_uart_configure(enum usic_channel_e channel, uint32_t baud,
                        uint32_t clock, unsigned int parity,
                        unsigned int nbits, unsigned int stop2)
{
  uintptr_t base;
  uint32_t oversampling;
  int ret;

  /* Get the base address of the USIC registers associated with this channel */

  base = xmc4_channel_baseaddress(channel);
  if (base == 0)
    {
      return -EINVAL;
    }

  /* Enable the USIC channel */

  ret = xmc4_enable_usic_channel(channel);
  if (ret < 0)
    {
      return ret;
    }

  ret = xmc4_uisc_baudrate(channel, baud, oversampling);

  /* Configure number of bits, stop bits and parity */
#warning Missing logic

  /* Check for odd parity */
#warning Missing logic

  /* Check for even parity */
#warning Missing logic

  /* Check for 9-bit operation */
#warning Missing logic

  /* Calculate baud settings (truncating) */
#warning Missing logic

  /* Configure FIFOs */
#warning Missing logic

  /* Enable RX and TX FIFOs */
#warning Missing logic

  /* Now we can (re-)enable the transmitter and receiver */
#warning Missing logic
}
#endif
