/****************************************************************************
 * sched/clock/clock_systimer.c
 *
 *   Copyright (C) 2011, 2014 Gregory Nutt. All rights reserved.
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

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* See nuttx/clock.h */

#undef clock_systimer
 
/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_systimer
 *
 * Description:
 *   Return the current value of the 32-bit system timer counter
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
#ifdef CONFIG_SCHED_TICKLESS
  struct timespec ts;
  uint64_t tmp;

  /* Get the time from the platform specific hardware */

  (void)up_timer_gettime(&ts);

  /* Convert to a 64- then 32-bit value */

  tmp = MSEC2TICK(1000 * (uint64_t)ts.tv_sec + (uint64_t)ts.tv_nsec / 1000000);
  return (uint32_t)(tmp & 0x00000000ffffffff);

#else

#ifdef CONFIG_SYSTEM_TIME64
  /* Return the current system time truncated to 32-bits */

  return (uint32_t)(g_system_timer & 0x00000000ffffffff);
#else
  /* Return the current system time */

  return g_system_timer;
#endif

#endif
}

/****************************************************************************
 * Name: clock_systimer64
 *
 * Description:
 *   Return the current value of the 64-bit system timer counter
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

#ifdef CONFIG_SYSTEM_TIME64
uint64_t clock_systimer64(void)
{
#ifdef CONFIG_SCHED_TICKLESS
  struct timespec ts;

  /* Get the time from the platform specific hardware */

  (void)up_timer_gettime(&ts);

  /* Convert to a 64- then 32-bit value */

  return MSEC2TICK(1000 * (uint64_t)ts.tv_sec + (uint64_t)ts.tv_nsec / 1000000);

#else
  /* Return the current system time */

  return g_system_timer;
#endif
}
#endif
