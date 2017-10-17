/****************************************************************************
 * arch/arm/src/bcm2708/bcm_lowputc.c
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
#include <assert.h>

#include "up_arch.h"

#include "chip/bcm2708_aux.h"
#include "bcm_config.h"
#include "bcm_lowputc.h"

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef BCM_HAVE_UART_CONSOLE
#  if defined(CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE)
#    define BCM_CONSOLE_VBASE    BCM_AUX_OFFSET
#    define BCM_CONSOLE_BAUD     CONFIG_BCM2708_MINI_UART_BAUD
#    define BCM_CONSOLE_BITS     CONFIG_BCM2708_MINI_UART_BITS
#    define BCM_CONSOLE_PARITY   CONFIG_BCM2708_MINI_UART_PARITY
#    define BCM_CONSOLE_2STOP    CONFIG_BCM2708_MINI_UART_2STOP
#  elif defined(CONFIG_BCM2708_PL011_UART_SERIAL_CONSOLE)
#    define BCM_CONSOLE_VBASE    BCM_PL011_VBASE
#    define BCM_CONSOLE_BAUD     CONFIG_BCM2708_PL011_UART_BAUD
#    define BCM_CONSOLE_BITS     CONFIG_BCM2708_PL011_UART_BITS
#    define BCM_CONSOLE_PARITY   CONFIG_BCM2708_PL011_UART_PARITY
#    define BCM_CONSOLE_2STOP    CONFIG_BCM2708_PL011_UART_2STOP
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef BCM_HAVE_UART_CONSOLE
static const struct uart_config_s g_console_config =
{
  .baud      = BCM_CONSOLE_BAUD,    /* Configured baud */
  .parity    = BCM_CONSOLE_PARITY,  /* 0=none, 1=odd, 2=even */
  .bits      = BCM_CONSOLE_BITS,    /* Number of bits (5-9) */
  .stopbits2 = BCM_CONSOLE_2STOP,   /* true: Configure with 2 stop bits instead of 1 */
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void bcm_lowsetup(void)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
#ifdef CONFIG_BCM2708_MINI_UART
  /* Disable and configure the Mini-UART */
# warning Missing logic

  /* Configure Mini-pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */
# warning Missing logic
#endif

#ifdef CONFIG_BCM2708_PL011_UART
  /* Disable and configure the PL011 UART */
# warning Missing logic

  /* Configure PL001 pins: RXD and TXD.  Also configure RTS and CTS if flow
   * control is enabled.
   */
# warning Missing logic
#endif

#ifdef BCM_HAVE_UART_CONSOLE
  /* Configure the serial console for initial, non-interrupt driver mode */

  (void)bcm_uart_configure(BCM_CONSOLE_VBASE, &g_console_config);
#endif
#endif /* CONFIG_SUPPRESS_UART_CONFIG */
}

/************************************************************************************
 * Name: bcm_uart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ************************************************************************************/

#ifdef BCM_HAVE_UART
int bcm_uart_configure(uint32_t base, FAR const struct uart_config_s *config)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
#  warning Missing logic
#endif

  return OK;
}
#endif /* BCM_HAVE_UART */

/************************************************************************************
 * Name: bcm_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.  This will even work
 *   BEFORE the console is initialized if we are booting from U-Boot (and the same
 *   UART is used for the console, of course.)
 *
 ************************************************************************************/

#if defined(BCM_HAVE_UART) && defined(CONFIG_DEBUG_FEATURES)
void bcm_lowputc(int ch)
{
#warning Missing logic
}
#endif
