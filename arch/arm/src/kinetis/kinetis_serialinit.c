/****************************************************************************
 * arch/arm/src/kinetis/kinetis_serialinit.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#include "kinetis_config.h"
#include "kinetis.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(HAVE_UART_DEVICE) && !defined(HAVE_LPUART_DEVICE)
#  undef CONFIG_KINETS_LPUART_LOWEST
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_earlyserialinit
 *
 * Description:
 *   Performs the low level UART and LPUART initialization early in debug
 *   so that the serial console will be available during bootup.  This must
 *   be called before arm_serialinit.  NOTE:  This function depends on GPIO
 *   pin configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

#if defined(USE_EARLYSERIALINIT)
void kinetis_earlyserialinit(void)
{
#if defined(HAVE_UART_DEVICE)
  /* Initialize UART drivers */

  kinetis_uart_earlyserialinit();
#endif

#if defined(HAVE_LPUART_DEVICE)
  /* Initialize LPUART drivers */

  kinetis_lpuart_earlyserialinit();
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register all the serial console and serial ports.  This assumes
 *   that kinetis_earlyserialinit was called previously.
 *
 ****************************************************************************/

#if defined(USE_SERIALDRIVER)
void arm_serialinit(void)
{
#if defined(HAVE_UART_DEVICE) ||defined(HAVE_LPUART_DEVICE)
  uint32_t start = 0;
#endif

  /* Register the console and drivers */

#if defined(HAVE_LPUART_DEVICE) && defined(CONFIG_KINETS_LPUART_LOWEST)
  /* Register LPUART drivers in starting positions */

   start = kinetis_lpuart_serialinit(start);
#endif

#if defined(HAVE_UART_DEVICE)
  /* Register UART drivers */

  start = kinetis_uart_serialinit(start);
#endif

#if defined(HAVE_LPUART_DEVICE) && !defined(CONFIG_KINETS_LPUART_LOWEST)
  /* Register LPUART drivers in last positions */

   start = kinetis_lpuart_serialinit(start);
#endif

}
#endif /* USE_SERIALDRIVER */
