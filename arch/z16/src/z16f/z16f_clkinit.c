/****************************************************************************
 * arch/z16/src/z16f/z16f_clkinit.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based upon sample code included with the Zilog ZDS-II toolchain.
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

#include "chip/chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System clock source value from ZDS target settings */

extern _Erom unsigned long SYS_CLK_SRC;
#define _DEFSRC ((unsigned long)&SYS_CLK_SRC)

/* System clock frequency value from ZDS target settings */

extern _Erom unsigned long SYS_CLK_FREQ;
#define _DEFCLK ((unsigned long)&SYS_CLK_FREQ)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
/****************************************************************************
 *  System clock initialization--DEBUG. Code Using Frequency Input from
 *  ZDS IDE.
 *
 * The sysclk_init function below uses the flexibility of the ZDS debug
 * environment to allow the user to experiment with different clock frequency
 * settings to help determine the frequency requirements of his Project.  The
 * function allows the selection of internal 5.56 MHz, the 10 KHz Watch Dog
 * timer or an external clock Source.  ZNEO supports clock frequency division
 * with the Clock Division Register.  The clock division Register will divide
 * by (a minimum of) 2 or more.  An assumed clock value of 5.5 MHz internal or
 * an external clock of 20 MHz was used as the crystal frequency to match the
 * Demo Target.   The User can enter a new frequency in the OTHER clock dialog
 * Target Setting.  The clock frequency is passed with the variable _DEFFREQ
 * and the clock source is _DEFSRC.
 *
 * NOTE: The UART output is designed to work with 5.56 MHz internal and 20 MHz
 * External clock frequencies at the Default Baud rate of 57.6K Baud.
 * Entering different clock frequencies may cause the UART to stop transmitting
 * unless the user makes changes to the UART routines.
 *
 * Function Not Recommended for Release Code.
 *
 ****************************************************************************/

static void z16f_sysclkinit(int clockid, uint32_t frequency)
{
  int count;
  int temp_oscdiv;

  /* _DEFSRC (SCKSEL Bits 1,0) is passed to program view the .linkcmd file */

  if ((getreg8(Z16F_OSC_CTL) & 0x03) != clockid)
    {
      switch (clockid)
        {
          /* 0: Internal precision oscillator functions as system clock at 5.6 MHz */

          case 0:
            {
              /* Enable 5.6 MHz clock RESET DEFAULT*/

              putreg8(0xe7, Z16F_OSC_CTL); /* Unlock the crystal oscillator */
              putreg8(0x18, Z16F_OSC_CTL);
              putreg8(0xa0, Z16F_OSC_CTL);

              /* Wait for oscillator to stabilize */

              for (count = 0; count < 10000; count++);

              /* Select 5.6 MHz clock (SCKSEL=0) */

              putreg8(0xe7, Z16F_OSC_CTL); /* Unlock the crystal oscillator */
              putreg8(0x18, Z16F_OSC_CTL);
              putreg8(0xa0, Z16F_OSC_CTL);
            }
            break;

          /* 1: Crystal oscillator or external clock driver functions as system clock */

          case 1:
            {
              /* No divider for the oscillator */

              putreg8(0x00, Z16F_OSC_DIV);

              /* Enable external oscillator */

              putreg8(0xe7, Z16F_OSC_CTL); /* Unlock the crystal oscillator */
              putreg8(0x18, Z16F_OSC_CTL);
              putreg8(0xe0, Z16F_OSC_CTL); /* INTEN+XTLEN+WDTEN */

              /* Wait for oscillator to stabilize */

              for (count = 0; count < 10000; count++);

              /* select external oscillator (SCKSEL=2) */

              putreg8(0xe7, Z16F_OSC_CTL); /* Unlock the crystal oscillator */
              putreg8(0x18, Z16F_OSC_CTL);
              putreg8(0xe0 | 1, Z16F_OSC_CTL);
            }
            break;

          /* 2: Reserved */

          default:
          case 2:
            {
              /* Reserved */
            }
            break;

          /* Watchdog Timer oscillator functions as system clock. */

          case 3:
            {
              /* Enable watchdog timer clock */

              putreg8(0xe7, Z16F_OSC_CTL); /* Unlock the crystal oscillator */
              putreg8(0x18, Z16F_OSC_CTL);
              putreg8(0xb0, Z16F_OSC_CTL);

              /* Wait for oscillator to stabilize */

              for (count = 0; count < 10000; count++);

              /* Select watch dog timer clock (SKCSEL=3) */

              putreg8(0xe7, Z16F_OSC_CTL); /* Unlock the crystal oscillator */
              putreg8(0x18, Z16F_OSC_CTL);
              putreg8(0xb0 | 3, Z16F_OSC_CTL);
            }
            break;
        }
    }

  /* Check SysClock Frequency.
   * divide the clock if the user has selected the OTHER option for frequency.
   */

  if (((clockid == 0) && (frequency < 3000000ul)) ||
      ((clockid == 1) && (frequency <= 10000000ul)))
    {
      if ( clockid == 0 )
        {
            temp_oscdiv = (5526000ul / (frequency + 1));
            /* Example @ 32 KHz:  0xAC (172 decimal)*/
        }
      else
        {
            temp_oscdiv = ((20000000ul / (frequency +1)) + 1);
        }

      /* Unlock and Set the Oscillator Division Register (Z16F_OSC_DIV) */

      putreg8(0xe7, Z16F_OSC_CTL); /* Unlock the crystal oscillator */
      putreg8(0x18, Z16F_OSC_CTL);
      putreg8(temp_oscdiv, Z16F_OSC_DIV);
    }

    /* Wait for oscillator to stabilize */

    for (count = 0; count < 10000; count++);
}

#else /* CONFIG_DEBUG_FEATURES */
/****************************************************************************
 * System Clock Initialization Recommended for Release Code
 *
 * The z16f_sysclkinit function below allows the user to switch from
 * Internal to External Clock source and should be used for clock frequency
 * switching to the External Clock.  Note the delay to allow the clock to
 * stabilize.
 ****************************************************************************/

static void z16f_sysclkinit(int clockid, uint32_t frequency)
{
  int count;

  /* In this configuration, we support only the external oscillator/clock
   * the source of the system clock (__DEFCLK is ignored).
   */

  if ((getreg8(Z16F_OSC_CTL) & 0x03) != 1)
    {
       /* No divider for the oscillator */

       putreg8(0x00, Z16F_OSC_DIV);

       /* Enable external oscillator */

       putreg8(0xe7, Z16F_OSC_CTL); /* Unlock the crystal oscillator */
       putreg8(0x18, Z16F_OSC_CTL);
       putreg8(0xe0, Z16F_OSC_CTL); /* INTEN+XTLEN+WDTEN */

       /* Wait for oscillator to stabilize */

       for (count = 0; count < 10000; count++);

       /* Select external oscillator (SCLKSEL=1) */

       putreg8(0xe7, Z16F_OSC_CTL); /* Unlock the crystal oscillator */
       putreg8(0x18, Z16F_OSC_CTL);
       putreg8(0xe0 | 1, Z16F_OSC_CTL); /* Use the external osc/clock as system clock */
    }
}
#endif /* CONFIG_DEBUG_FEATURES */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16f_clkinit
 ****************************************************************************/

void z16f_clkinit(void)
{
  /* _DEFSRC (SCKSEL Bits 1,0) is passed to program view the .linkcmd file */

  z16f_sysclkinit(_DEFSRC, _DEFCLK);
}
