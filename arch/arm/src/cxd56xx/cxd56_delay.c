/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_delay.c
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
#include <nuttx/arch.h>

#include <stdint.h>

#include "cxd56_clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CXD56_USE_SYSBUS
#  define CXD56XX_LOOPSPERMSEC_156MHZ 15533ull
#else
#  define CXD56XX_LOOPSPERMSEC_156MHZ 7428ull
#endif

#define CXD56XX_LOOPSPERMSEC_BY_CLOCK(clock) \
  (uint32_t)(CXD56XX_LOOPSPERMSEC_156MHZ * (clock) / 156000000ull)

/* Adjust manually to be up_udelay(1000) is neary equal with up_udelay(999) */

#ifdef CONFIG_CXD56_USE_SYSBUS
#  define CXD56XX_LOOPSPERUSEC_ADJUST 1010ul;
#else
#  define CXD56XX_LOOPSPERUSEC_ADJUST 810ul;
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

void up_mdelay(unsigned int milliseconds)
{
  volatile unsigned int i;
  volatile unsigned int j;
  uint32_t clock = cxd56_get_cpu_baseclk();
  uint32_t loops = CXD56XX_LOOPSPERMSEC_BY_CLOCK(clock);

  for (i = 0; i < milliseconds; i++)
    {
      for (j = 0; j < loops; j++)
        {
        }
    }
}

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Delay inline for the requested number of microseconds.  NOTE:  Because
 *   of all of the setup, several microseconds will be lost before the actual
 *   timing loop begins.  Thus, the delay will always be a few microseconds
 *   longer than requested.
 *
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void up_udelay(useconds_t microseconds)
{
  volatile unsigned int i;
  volatile unsigned int j;
  uint32_t clock = cxd56_get_cpu_baseclk();
  uint32_t loops = CXD56XX_LOOPSPERMSEC_BY_CLOCK(clock);
  uint32_t milliseconds = microseconds / 1000;

  for (i = 0; i < milliseconds; i++)
    {
      for (j = 0; j < loops; j++)
        {
        }
    }

  loops = loops * (microseconds % 1000) / CXD56XX_LOOPSPERUSEC_ADJUST;
  for (i = 0; i < loops; i++)
    {
    }
}
