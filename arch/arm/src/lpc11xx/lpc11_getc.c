/****************************************************************************
 * arch/arm/src/lpc11/lpc11_getc.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"

#include "lpc11_getc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC11_UART0_BASE
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC11_UART1_BASE
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC11_UART2_BASE
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getc
 *
 * Description:
 *   Input one byte from the serial console.
 *
 *   REVIST:  If used with the serial driver enabled, then this could
 *   interfere with the serial driver operations.  Serial interrupts should
 *   be disabled when this function executes in that case.
 *
 ****************************************************************************/

int up_getc(void)
{
  uint8_t ch = 0;

#if defined HAVE_UART && defined HAVE_SERIAL_CONSOLE
  /* Wait while the Receiver Data Ready (RDR) is indicating a "empty" FIFO to
   * assure that we have data in the buffer to read.
   */

  while ((getreg32(CONSOLE_BASE+LPC11_UART_LSR_OFFSET) & UART_LSR_RDR) == 0);

  /* Then read a character from the UART data register */

  ch = getreg8(CONSOLE_BASE+LPC11_UART_RBR_OFFSET);
#endif

  return (int)ch;
}
