/************************************************************************************
 * configs/scp16c26/src/up_lcd.c
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <nuttx/arch.h>

#include "up_internal.h"
#include "skp16c26_internal.h"

/* Only use the LCD as a console if there are is no serial console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && !defined(CONFIG_UART0_DISABLE)
#  define HAVE_SERIALCONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && !defined(CONFIG_UART1_DISABLE)
#  define HAVE_SERIALCONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && !defined(CONFIG_UART2_DISABLE)
#  define HAVE_SERIALCONSOLE 1
#else
#  undef HAVE_SERIALCONSOLE
#endif

#ifndef HAVE_SERIALCONSOLE

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Funtions
 ************************************************************************************/

/************************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the hardware used for the console.
 *   Its purpose is to get the console output availabe as soon as possible.
 *
 ************************************************************************************/

void up_lowsetup(void)
{
#	warning "To be provided"
}

/************************************************************************************
 * Name: up_earlyconsoleinit
 *
 * Description:
 *   Performs the low level UART initialization early in  debug so that the serial
 *   console will be available during bootup.  This must be called before
 *   up_consoleinit.
 *
 ************************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DEV_LOWCONSOLE)
void up_earlyconsoleinit(void)
{
  /* In this context, up_earlyconsoleinit does not need to do anything. */
}
#endif

/************************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyconsoleinit was called previously.
 *
 ************************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DEV_LOWCONSOLE)
void up_consoleinit(void)
{
  /* In this context, up_earlyconsoleinit does not need to do anything. */
}
#endif

/************************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one character on the console
 *
 ************************************************************************************/

void up_lowputc(char ch)
{
#	warning "To be provided"
}

/************************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Output one character on the console
 *
 ************************************************************************************/

int up_putc(int ch)
{
  /* Same as up_lowputc in this context */

  up_lowputc(ch);
  return ch;
}

#endif /* HAVE_SERIALCONSOLE */
