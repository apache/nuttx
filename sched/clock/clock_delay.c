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

#define CONFIG_BOARD_LOOPSPER100USEC ((CONFIG_BOARD_LOOPSPERMSEC+5)/10)
#define CONFIG_BOARD_LOOPSPER10USEC  ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)
#define CONFIG_BOARD_LOOPSPERUSEC    ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 ****************************************************************************/

void weak_function up_udelay(useconds_t microseconds)
{
  udelay_coarse(microseconds);
}

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
