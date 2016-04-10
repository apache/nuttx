/****************************************************************************
 * config/tm4c1294-launchpad/src/tm4c_timer.c
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

#include <debug.h>
#include "tiva_timer.h"
#include "tm4c1294-launchpad.h"

#ifdef CONFIG_TM4C1294_LAUNCHPAD_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_TIMER
#  error CONFIG_TIMER is not defined
#endif

#ifndef CONFIG_TIVA_TIMER32_PERIODIC
#  error CONFIG_TIVA_TIMER32_PERIODIC is not defined
#endif

#if defined(CONFIG_DK_TM4CTM4C1294_LAUNCHPADX_TIMER0)
#  define GPTM 0
#elif defined(CONFIG_TM4C1294_LAUNCHPAD_TIMER1)
#  define GPTM 1
#elif defined(CONFIG_TM4C1294_LAUNCHPAD_TIMER2)
#  define GPTM 2
#elif defined(CONFIG_TM4C1294_LAUNCHPAD_TIMER3)
#  define GPTM 3
#elif defined(CONFIG_TM4C1294_LAUNCHPAD_TIMER4)
#  define GPTM 4
#elif defined(CONFIG_TM4C1294_LAUNCHPAD_TIMER5)
#  define GPTM 5
#elif defined(CONFIG_TM4C1294_LAUNCHPAD_TIMER6)
#  define GPTM 6
#elif defined(CONFIG_TM4C1294_LAUNCHPAD_TIMER7)
#  define GPTM 7
#else
#  error No CONFIG_TM4C1294_LAUNCHPAD_TIMERn definition
#endif

#ifndef CONFIG_TM4C1294_LAUNCHPAD_TIMER_DEVNAME
#  define CONFIG_TM4C1294_LAUNCHPAD_TIMER_DEVNAME "/dev/timer0"
#endif

#undef CONFIG_TM4C1294_LAUNCHPAD_TIMER_ALTCLK
#define ALTCLK false

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_timer_initialize
 *
 * Description:
 *   Configure the timer driver
 *
 ****************************************************************************/

int tiva_timer_initialize(void)
{
  int ret;

  timvdbg("Registering TIMER%d at %s\n",
          GPTM, CONFIG_TM4C1294_LAUNCHPAD_TIMER_DEVNAME);

  ret = tiva_timer_register(CONFIG_TM4C1294_LAUNCHPAD_TIMER_DEVNAME,
                            GPTM, ALTCLK);
  if (ret < 0)
    {
      timdbg("ERROR: Failed to register timer driver: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_TM4C1294_LAUNCHPAD_TIMER */
