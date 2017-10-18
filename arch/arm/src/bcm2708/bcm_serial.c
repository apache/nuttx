/****************************************************************************
 * arch/arm/src/bcm/bcm_serialinit.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#include "bcm_config.h"
#include "bcm_serial.h"

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
 * Name: bcm_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void bcm_earlyserialinit(void)
{
#ifdef CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE
  /* Initialize the Mini-UART console */

  bcm_miniuart_earlyserialinit();
#endif

#ifdef CONFIG_BCM2708_PL011_UART_SERIAL_CONSOLE
  /* Initialize the PL011-UART console */

  bcm_pl011uart_earlyserialinit();
#endif
}
#endif

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register all the serial console and serial ports.  This assumes
 *   that bcm_earlyserialinit was called previously.
 *
 ****************************************************************************/

#ifdef USE_SERIALDRIVER
void up_serialinit(void)
{
#ifdef CONFIG_BCM2708_MINI_UART
  /* Register the Mini-UART serial console and serial ports.  This function
   * will be called by uart_serialinit() if the Mini-UART is enabled.
   */

  bcm_miniuart_serialinit();
#endif

#ifdef CONFIG_BCM2708_PL011_UART
  /* Register the PL011-UART serial console and serial ports.  This function
   * will be called by uart_serialinit() if the Mini-UART is enabled.
   */

  bcm_pl011uart_serialinit();
#endif
}
#endif /* USE_SERIALDRIVER */

