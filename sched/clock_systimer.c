/****************************************************************************
 * sched/clock_systimer.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/rtc.h>
#include <nuttx/time.h>

#include <arch/irq.h>

#if !defined(clock_systimer) /* See nuttx/clock.h */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  clock_systimer
 *
 * Description:
 *   Return the current value of the system timer counter
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   The current value of the system timer counter
 *
 * Assumptions:
 *
 ****************************************************************************/

uint32_t clock_systimer(void)
{
#ifdef CONFIG_SYSTEM_UTC
  irqstate_t flags;
  uint32_t system_utc;
  uint32_t tickcount;
#endif

  /* Fetch the g_system_timer value from timer hardware, if available */

#ifdef CONFIG_RTC

  /* Check if the periodic timer is initialized
   *
   * Note that the unit of the g_system_timer and and up_rtc_getclock() do
   * not have the same unit.
   */

  if (g_rtc_enabled)
    {
      /* return up_rtc_getclock(); */
    }
#endif

#ifndef CONFIG_SYSTEM_UTC
  return g_system_timer;
#else
  /* Disable interrupts while g_system_utc and g_tickcount are sampled
   * so that we can be assured that g_system_utc and g_tickcount are based
   * at the same point in time.
   */

  flags = irqsave();
  system_utc = g_system_utc;
  tickcount  = g_tickcount;
  irqrestore(flags);

  return system_utc * TICK_PER_SEC + tickcount;
#endif
}

#endif /* !clock_systtimer */

