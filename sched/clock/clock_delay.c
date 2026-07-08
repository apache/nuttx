/****************************************************************************
 * sched/clock/clock_delay.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* NOTE: This file contains the default implementation of `up_*delay`
 * functions, which perform delays by busy-waiting in a for-loop. This
 * functions do not work correctly unless the value
 * `CONFIG_BOARD_LOOPSPERMSEC` is configured to the correct value for the
 * given board. Its value should be how many loops approximately add up to
 * one millisecond of delay.
 *
 * All one needs to do to override the implementation is override
 * `up_udelay`. `up_mdelay` and `up_ndelay` implementations are both
 * dependent on `up_udelay`. If you want them to be independent, you can
 * also override each `up_*delay` function separately.
 *
 * WARNING: These functions are not accurate. If the scheduler suspends a
 * process in the middle of performing a busy-wait for say, 10ms, and then
 * starts it running again, the process will have waited whatever duration
 * the busy-wait was meant to be, plus those 10ms. Thus, these functions
 * should really only be used in situations where there is no better
 * alternative.
 *
 * The definitions of these functions are 'weak', so if another file linked
 * into the binary provides an alternate definition of these functions, that
 * definition is what will be used.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_BOARD_LOOPSPERMSEC) && CONFIG_BOARD_LOOPSPERMSEC == 0
#warning                                                                       \
    "CONFIG_BOARD_LOOPSPERMSEC is set to 0 even though this architecture does" \
    "not rely on timer or alarm drivers for correct timings. up_udelay() and " \
    "similar delay functions will not work correctly. Please determine an "    \
    "appropriate value for CONFIG_BOARD_LOOPSPERMSEC using the calib_udelay "  \
    "application in nuttx-apps. If this configuration is a NuttX provided "    \
    "configuration, it would be appreciated if you submit a patch with the "   \
    "new value to apache/nuttx."
#endif

#ifndef CONFIG_ARCH_HAVE_DYNAMIC_UDELAY
static_assert(CONFIG_BOARD_LOOPSPERMSEC != -1,
              "Configure BOARD_LOOPSPERMSEC to non-default value.");

/* If ARCH_HAVE_DYNAMIC_UDELAY is set, BOARD_LOOPSPERMSEC is unset,
 * calculate these optionally. (Only used in udelay_coarse and
 * that function is excluded from the build if these end up undefined.)
 */

#  define CONFIG_BOARD_LOOPSPER100USEC ((CONFIG_BOARD_LOOPSPERMSEC+5)/10)
#  define CONFIG_BOARD_LOOPSPER10USEC  ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)
#  define CONFIG_BOARD_LOOPSPERUSEC    ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_HAVE_UDELAY

static void udelay_coarse(useconds_t microseconds)
{
  volatile int i;

  /* We'll do this a little at a time because we expect that the
   * CONFIG_BOARD_LOOPSPERUSEC is very inaccurate during to truncation in
   * the divisions of its calculation.  We'll use the largest values that
   * we can in order to prevent significant error buildup in the loops.
   */

  while (microseconds > 1000)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPERMSEC; i++)
        {
        }

      microseconds -= 1000;
    }

  while (microseconds > 100)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPER100USEC; i++)
        {
        }

      microseconds -= 100;
    }

  while (microseconds > 10)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPER10USEC; i++)
        {
        }

      microseconds -= 10;
    }

  while (microseconds > 0)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPERUSEC; i++)
        {
        }

      microseconds--;
    }
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_mdelay
 *
 * Description:
 *   Delay inline for the requested number of milliseconds.
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void weak_function up_mdelay(unsigned int milliseconds)
{
  up_udelay(USEC_PER_MSEC * milliseconds);
}

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Delay inline for the requested number of microseconds.
 *
 *   *** NOT multi-tasking friendly ***
 *
 *   Note that this method is both marked weak and compiled optionally
 *   based on ARCH_HAVE_UDELAY option. This is redundant but kept
 *   as a temporary measure.
 *
 *   It was discovered that sometimes the weak attribute is not enough
 *   to have the method replaced with other, non-weak implementation.
 *   (Described in more detail in help text for the ARCH_HAVE_UDELAY
 *   Kconfig option.) The ARCH_HAVE_UDELAY option is meant to remedy
 *   this but the author of the change does not own all the hardware
 *   that implements its own up_udelay and is affected by this problem.
 *
 *   Enabling ARCH_HAVE_UDELAY for every such chip is therefore impossible,
 *   the change would be untested. Instead, the weak attribute is kept,
 *   preserving previous behaviour of current code. If, at some future
 *   time, the change is tested for other chips that implement up_udelay,
 *   the weak attribute and this comment can be removed.
 *
 *   The same also applies to up_udelay implementations
 *   in drivers/timers/arch_alarm.c and drivers/timers/arch_timer.c
 *   and this comment is referenced there.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_HAVE_UDELAY

void weak_function up_udelay(useconds_t microseconds)
{
  udelay_coarse(microseconds);
}

#endif

/****************************************************************************
 * Name: up_ndelay
 *
 * Description:
 *   Delay inline for the requested number of nanoseconds.
 *
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void weak_function up_ndelay(unsigned long nanoseconds)
{
  up_udelay((nanoseconds + NSEC_PER_USEC - 1) / NSEC_PER_USEC);
}
