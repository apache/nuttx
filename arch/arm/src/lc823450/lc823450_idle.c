/****************************************************************************
 * arch/arm/src/lc823450/lc823450_idle.c
 *
 *   Copyright (C) 2014-2017 Sony Corporation. All rights reserved.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
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

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include "nvic.h"
#include "up_internal.h"
#include "up_arch.h"

#ifdef CONFIG_DVFS
#  include "lc823450_dvfs.h"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LC823450_SLEEP_MODE
static int32_t  g_in_sleep;
static uint64_t g_sleep_t0;
#endif /* CONFIG_LC823450_SLEEP_MODE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_get_current_time()
 ****************************************************************************/

#ifdef CONFIG_LC823450_SLEEP_MODE
static uint64_t up_get_current_time(void)
{
  struct timespec ts;

#ifdef CONFIG_CLOCK_MONOTONIC
  clock_gettime(CLOCK_MONOTONIC, &ts);
#else
  clock_gettime(CLOCK_REALTIME, &ts);
#endif
  return (uint64_t)ts.tv_sec * NSEC_PER_SEC + (uint64_t)ts.tv_nsec;
}
#endif /* CONFIG_LC823450_SLEEP_MODE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  sched_process_timer();
#else

#ifdef CONFIG_LC823450_SLEEP_MODE
  irqstate_t flags;
  flags = enter_critical_section();

  g_sleep_t0 = up_get_current_time();
  g_in_sleep = 1;

  /* Clear SLEEPDEEP flag */

  uint32_t regval  = getreg32(NVIC_SYSCON);
  regval &= ~NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

#ifdef CONFIG_DVFS
  lc823450_dvfs_enter_idle();
#endif

  leave_critical_section(flags);
#endif /* CONFIG_LC823450_SLEEP_MODE */

  /* Sleep until an interrupt occurs to save power */

  asm("WFI");
#endif
}

/****************************************************************************
 * Name: up_update_idle_time()
 *
 * Description:
 *  up_update_idle_time() is the logic that will update idle time
 *
 ****************************************************************************/

#ifdef CONFIG_LC823450_SLEEP_MODE
void up_update_idle_time(void)
{
  if (g_in_sleep)
    {
      g_in_sleep = 0;
      uint64_t t1 = up_get_current_time();
      sched_add_idl_tm(t1 - g_sleep_t0);
    }
}
#endif /* CONFIG_LC823450_SLEEP_MODE */
