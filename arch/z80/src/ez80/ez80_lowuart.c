/****************************************************************************
 * arch/z80/src/ez80/ez80_loweruart.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
#include <string.h>

#include <arch/io.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "chip/chip.h"
#include "common/up_internal.h"

#ifdef CONFIG_USE_LOWUARTINIT

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/* The system clock frequency is defined in the linkcmd file */

extern unsigned long SYS_CLK_FREQ;
#define _DEFCLK ((unsigned long)&SYS_CLK_FREQ)

#ifdef CONFIG_UART0_SERIAL_CONSOLE
#  define ez80_inp(offs)     inp((EZ80_UART0_BASE+(offs)))
#  define ez80_outp(offs,val) outp((EZ80_UART0_BASE+(offs)), (val))
#  define CONFIG_UART_BAUD      CONFIG_UART0_BAUD
#  if CONFIG_UART0_BITS == 7
#    define CONFIG_UART_BITS EZ80_UARTCHAR_7BITS
#  else
#    define CONFIG_UART_BITS EZ80_UARTCHAR_8BITS
#  endif
#  if CONFIG_UART0_2STOP != 0
#    define CONFIG_UART_2STOP EZ80_UARTLCTl_2STOP
#  else
#    define CONFIG_UART_2STOP 0
#  endif
#  if CONFIG_UART0_PARITY == 1 /* Odd parity */
#    define CONFIG_UART_PARITY EZ80_UARTLCTL_PEN
#  elif CONFIG_UART0_PARITY == 2  /* Even parity */
#    define CONFIG_UART_PARITY (EZ80_UARTLCTL_PEN|EZ80_UARTLCTL_EPS)
#  else
#    define CONFIG_UART_PARITY 0
#  endif
#else
#  define ez80_inp(offs)     inp((EZ80_UART1_BASE+(offs)))
#  define ez80_outp(offs.val) outp((EZ80_UART1_BASE+(offs)), (val))
#  define CONFIG_UART_BAUD      CONFIG_UART1_BAUD
#  if CONFIG_UART1_BITS == 7
#    define CONFIG_UART_BITS EZ80_UARTCHAR_7BITS
#  else
#    define CONFIG_UART_BITS EZ80_UARTCHAR_8BITS
#  endif
#  if CONFIG_UART1_2STOP != 0
#    define CONFIG_UART_2STOP EZ80_UARTLCTl_2STOP
#  else
#    define CONFIG_UART_2STOP 0
#  endif
#  if CONFIG_UART1_PARITY == 1 /* Odd parity */
#    define CONFIG_UART_PARITY EZ80_UARTLCTL_PEN
#  elif CONFIG_UART1_PARITY == 2  /* Even parity */
#    define CONFIG_UART_PARITY (EZ80_UARTLCTL_PEN|EZ80_UARTLCTL_EPS)
#  else
#    define CONFIG_UART_PARITY 0
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void ez80_setbaud(void)
{
  uint24 brg_divisor;
  ubyte lctl;

  /* The resulting BAUD and depends on the system clock frequency and the
   * BRG divisor as follows:
   *
   * BAUD = SYSTEM_CLOCK_FREQUENCY / (16 * BRG_Divisor)
   *
   * Or
   *
   * BRG_Divisor = SYSTEM_CLOCK_FREQUENCY / 16 / BAUD
   */

   brg_divisor = ( _DEFCLK + (CONFIG_UART_BAUD << 3)) / (CONFIG_UART_BAUD << 4);

   /* Set the DLAB bit to enable access to the BRG registers */

   lctl = ez80_inp(EZ80_UART_LCTL);
   lctl |= EZ80_UARTLCTL_DLAB;
   ez80_outp(EZ80_UART_LCTL, lctl);

   ez80_outp(EZ80_UART_BRGL, (ubyte)(brg_divisor & 0xff));
   ez80_outp(EZ80_UART_BRGH, (ubyte)(brg_divisor >> 8));

   lctl &= ~EZ80_UARTLCTL_DLAB;
   ez80_outp(EZ80_UART_LCTL, lctl);
}
#endif /* CONFIG_SUPPRESS_UART_CONFIG */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowuartinit
 ****************************************************************************/

void up_lowuartinit(void)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  ubyte reg;

  /* Disable interrupts from the UART */

  reg = ez80_inp(EZ80_UART_IER);
  reg &= ~EZ80_UARTEIR_INTMASK;
  ez80_outp(EZ80_UART_IER, reg);

  /* Set the baud rate */

  ez80_setbaud();
  ez80_outp(EZ80_UART_MCTL, 0);

  /* Set the character properties */

  reg = ez80_inp(EZ80_UART_LCTL);
  reg &= ~EZ80_UARTLCTL_MASK;
  reg |= (CONFIG_UART_BITS | CONFIG_UART_2STOP | CONFIG_UART_PARITY);
  ez80_outp(EZ80_UART_LCTL, reg);

  /* Enable and flush the receive FIFO */

  reg = EZ80_UARTFCTL_FIFOEN;
  ez80_outp(EZ80_UART_FCTL, reg);
  reg |= (EZ80_UARTFCTL_CLRTxF|EZ80_UARTFCTL_CLRRxF);
  ez80_outp(EZ80_UART_FCTL, reg);

  /* Set the receive trigger level to 1 */

  reg |= EZ80_UARTTRIG_1;
  ez80_outp(EZ80_UART_FCTL, reg);
#endif /* CONFIG_SUPPRESS_UART_CONFIG */
}
#endif /* CONFIG_USE_LOWUARTINIT */
