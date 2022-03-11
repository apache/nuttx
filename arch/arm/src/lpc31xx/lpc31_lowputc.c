/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_lowputc.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "lpc31_cgudrvr.h"
#include "lpc31_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Is the UART enabled? */

#ifdef CONFIG_LPC31_UART
#  define HAVE_UART 1

/* Is it a serial console? */

#  ifdef CONFIG_UART_SERIAL_CONSOLE
#    define HAVE_CONSOLE 1

/* Is initialization performed by arm_earlyserialinit()?  Or is UART
 * initialization suppressed?
 */

#    if defined(USE_EARLYSERIALINIT) || defined(CONFIG_SUPPRESS_UART_CONFIG)
#      undef NEED_LOWSETUP
#    else
#      define NEED_LOWSETUP 1
#    endif
#  else
#      undef HAVE_CONSOLE
#      undef NEED_LOWSETUP
#  endif

#else
#  undef HAVE_UART
#  undef HAVE_CONSOLE
#  undef NEED_LOWSETUP
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/

#ifdef HAVE_CONSOLE
static inline void up_waittxready(void)
{
  int tmp;

  /* Limit how long we will wait for the TX available condition */

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      /* Check if the tranmitter holding register (THR) is empty */

      if ((getreg32(LPC31_UART_LSR) & UART_LSR_THRE) != 0)
        {
          /* The THR is empty, return */

          break;
        }
    }
}
#endif

/****************************************************************************
 * Name: up_configbaud
 ****************************************************************************/

#ifdef NEED_LOWSETUP
static inline void up_configbaud(void)
{
  /* In a buckled-up, embedded system, there is no reason to constantly
   * calculate the following.  The calculation can be skipped if the
   * MULVAL, DIVADDVAL, and DIVISOR values are provided in the configuration
   * file.
   */

#ifndef CONFIG_LPC31_UART_MULVAL
  uint32_t qtrclk;
  uint32_t regval;

  /* Test values calculated for every multiplier/divisor combination */

  uint32_t tdiv;
  uint32_t tmperr;
  int      tmulval;
  int      tdivaddval;

  /* Optimal multiplier/divider values */

  uint32_t div       = 0;
  uint32_t errval    = 100000;
  int      mulval    = 1;
  int      divaddval = 0;

  /* Baud is generated using FDR and DLL-DLM registers
   *
   *   baud = clock * (mulval/(mulval+divaddval) / (16 * div)
   *
   * Or
   *
   *   div = (clock/16) * (mulval/(mulval+divaddval) / baud
   *
   * Where mulval    = Fractional divider multiplier
   *       divaddval = Fractional divider pre-scale div
   *       div       = DLL-DLM divisor
   */

  /* Get UART block clock divided by 16 */

  qtrclk = lpc31_clkfreq(CLKID_UARTUCLK, DOMAINID_UART) >> 4;

  /* Try every valid multiplier, tmulval (or until a perfect
   * match is found).
   */

  for (tmulval = 1 ; tmulval <= 15 && errval > 0; tmulval++)
    {
      /* Try every valid pre-scale div, tdivaddval (or until a perfect
       * match is found).
       */

      for (tdivaddval = 0 ; tdivaddval <= 15 && errval > 0; tdivaddval++)
        {
          /* Calculate the divisor with these fractional divider settings */

          uint32_t tmp = (tmulval * qtrclk) / ((tmulval + tdivaddval));
          tdiv         = (tmp + (CONFIG_UART_BAUD >> 1)) / CONFIG_UART_BAUD;

          /* Check if this candidate divisor is within a valid range */

          if (tdiv > 2 && tdiv < 0x10000)
            {
              /* Calculate the actual baud and the error */

              uint32_t actualbaud = tmp / tdiv;

              if (actualbaud <= CONFIG_UART_BAUD)
                {
                  tmperr = CONFIG_UART_BAUD - actualbaud;
                }
              else
                {
                  tmperr = actualbaud - CONFIG_UART_BAUD;
                }

              /* Is this the smallest error we have encountered? */

              if (tmperr < errval)
                {
                  /* Yes,
                   * save these settings as the new,
                   * candidate optimal settings
                   */

                  mulval    = tmulval ;
                  divaddval = tdivaddval;
                  div       = tdiv;
                  errval    = tmperr;
                }
            }
        }
    }

  /* Set the Divisor Latch Access Bit (DLAB) to enable DLL/DLM access */

  regval  = getreg32(LPC31_UART_LCR);
  regval |= UART_LCR_DLAB;
  putreg32(regval, LPC31_UART_LCR);

  /* Configure the MS and LS DLAB registers */

  putreg32(div & UART_DLL_MASK, LPC31_UART_DLL);
  putreg32((div >> 8) & UART_DLL_MASK, LPC31_UART_DLM);

  regval &= ~UART_LCR_DLAB;
  putreg32(regval, LPC31_UART_LCR);

  /* Configure the Fractional Divider Register (FDR) */

  putreg32((mulval    << UART_FDR_MULVAL_SHIFT) |
           (divaddval << UART_FDR_DIVADDVAL_SHIFT),
           LPC31_UART_FDR);
#else
  /* Set the Divisor Latch Access Bit (DLAB) to enable DLL/DLM access */

  regval  = getreg32(LPC31_UART_LCR);
  regval |= UART_LCR_DLAB;
  putreg32(regval, LPC31_UART_LCR);

  /* Configure the MS and LS DLAB registers */

  putreg32(CONFIG_LPC31_UART_DIVISOR & UART_DLL_MASK, LPC31_UART_DLL);
  putreg32((CONFIG_LPC31_UART_DIVISOR >> 8) & UART_DLL_MASK, LPC31_UART_DLM);

  regval &= ~UART_LCR_DLAB;
  putreg32(regval, LPC31_UART_LCR);

  /* Configure the Fractional Divider Register (FDR) */

  putreg32((CONFIG_LPC31_UART_MULVAL    << UART_FDR_MULVAL_SHIFT) |
           (CONFIG_LPC31_UART_DIVADDVAL << UART_FDR_DIVADDVAL_SHIFT),
           LPC31_UART_FDR);
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_lowsetup
 *
 * Description:
 *   Called early in arm_boot.
 *   Performs chip-common low level initialization.
 *
 ****************************************************************************/

void lpc31_lowsetup(void)
{
#ifdef NEED_LOWSETUP
  uint32_t regval;

  /* Enable UART system clock */

  lpc31_enableclock(CLKID_UARTAPBCLK);
  lpc31_enableclock(CLKID_UARTUCLK);

  /* Clear fifos */

  putreg32((UART_FCR_RXFIFORST | UART_FCR_TXFIFORST), LPC31_UART_FCR);

  /* Set trigger */

  putreg32((UART_FCR_FIFOENABLE | UART_FCR_RXTRIGLEVEL_16), LPC31_UART_FCR);

  /* Set up the LCR */

  regval = 0;

#if CONFIG_UART_BITS == 5
  regval |= UART_LCR_WDLENSEL_5BITS;
#elif CONFIG_UART_BITS == 6
  regval |= UART_LCR_WDLENSEL_6BITS;
#elif CONFIG_UART_BITS == 7
  regval |= UART_LCR_WDLENSEL_7BITS;
#else
  regval |= UART_LCR_WDLENSEL_8BITS;
#endif

#if CONFIG_UART_2STOP > 0
  regval |= UART_LCR_NSTOPBITS;
#endif

#if CONFIG_UART_PARITY == 1
  regval |= UART_LCR_PAREN;
#elif CONFIG_UART_PARITY == 2
  regval |= (UART_LCR_PAREVEN | UART_LCR_PAREN);
#endif
  putreg32(regval, LPC31_UART_LCR);

  /* Set the BAUD divisor */

  up_configbaud();

  /* Configure the FIFOs */

  putreg32((UART_FCR_RXTRIGLEVEL_16 | UART_FCR_TXFIFORST |
            UART_FCR_RXFIFORST | UART_FCR_FIFOENABLE),
           LPC31_UART_FCR);

  /* The NuttX serial driver waits for the first THRE interrupt before
   * sending serial data... However, it appears that the lpc313x hardware
   * does not generate that interrupt until a transition from not-empty
   * to empty.  So, the current kludge here is to send one NULL at
   * startup to kick things off.
   */

  putreg32('\0', LPC31_UART_THR);
#endif
}

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  up_waittxready();
  putreg32((uint32_t)ch, LPC31_UART_THR);
#endif
}
