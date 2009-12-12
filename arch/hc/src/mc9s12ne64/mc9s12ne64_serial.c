/****************************************************************************
 * arch/hc/src/mc9s12ne64/mc9s12ne64_serial.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/types.h>

#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there an SCI enabled? */

#if !defined(CONFIG_HCS12_SCI0) && !defined(CONFIG_HCS12_SCI1)
#  undef HAVE_SERIAL
#else
#  define HAVE_SERIAL 1
#endif

/* Is there a serial console?  Need to have (1) at least SCI enabled, and (2)
 * serial console and file descriptors selected in the configuration
 */

#if defined(HAVE_SERIAL) && defined(CONFIG_USE_SERIALDRIVER)
#  if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_HCS12_SCI0)
#    undef CONFIG_SCI1_SERIAL_CONSOLE
#    define HAVE_SERIAL_CONSOLE 1
#  elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_HCS12_SCI1)
#    undef CONFIG_SCI0_SERIAL_CONSOLE
#    define HAVE_SERIAL_CONSOLE 1
#  else
#    undef CONFIG_SCI0_SERIAL_CONSOLE
#    undef CONFIG_SCI1_SERIAL_CONSOLE
#    undef HAVE_SERIAL_CONSOLE
#  endif
#else
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#  undef CONFIG_USE_SERIALDRIVER
#  undef CONFIG_USE_EARLYSERIALINIT
#endif

#ifdef CONFIG_USE_SERIALDRIVER

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level SCI initialization early in  debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

#ifdef CONFIG_USE_EARLYSERIALINIT
void up_earlyserialinit(void)
{
#error "Serial support not-implemented"
}
#endif

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
#error "Serial support not-implemented"
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#error "Serial support not-implemented"
}

#else /* CONFIG_USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef CONFIG_ARCH_LOWPUTC
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* CONFIG_USE_SERIALDRIVER */
