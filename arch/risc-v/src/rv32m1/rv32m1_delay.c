/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_delay.c
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

#include "rv32m1.h"
#include "rv32m1_timersvc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_BOARD_LOOPSPER100USEC ((CONFIG_BOARD_LOOPSPERMSEC+5)/10)
#define CONFIG_BOARD_LOOPSPER10USEC  ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)
#define CONFIG_BOARD_LOOPSPERUSEC    ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)

/****************************************************************************
 * Data Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  up_sw_mdelay
 *
 * Description:
 *   Delay inline for the requested number of milliseconds in software
 *   way. The codes are copies of built-in up_mdelay.
 *
 ****************************************************************************/

static void up_sw_mdelay(unsigned int milliseconds)
{
  volatile unsigned int i;
  volatile unsigned int j;

  for (i = 0; i < milliseconds; i++)
    {
      for (j = 0; j < CONFIG_BOARD_LOOPSPERMSEC; j++)
        {
        }
    }
}

/****************************************************************************
 * Name:  up_sw_udelay
 *
 * Description:
 *   Delay inline for the requested number of microseconds in software
 *   way. The codes are copies of built-in up_udelay.
 *
 ****************************************************************************/

static void up_sw_udelay(unsigned int microseconds)
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
 * Name:  up_hw_delay
 ****************************************************************************/

static void up_hw_delay(unsigned int period,  unsigned int unit,
                        unsigned int ticks)
{
  uint32_t t;
  uint32_t elapsed;

  /* The current value of timer */

  uint32_t cval = rv32m1_timersvc_value();

  /* The last value of timer */

  uint32_t lval = cval;

  elapsed = 0;

  while (ticks > 0)
    {
      cval = rv32m1_timersvc_value();

      if (cval != lval)
        {
          /* Accumulate the elapsed time */

          elapsed += lval < cval ? cval - lval : cval + period - lval;
          lval = cval;

          if (elapsed >= unit)
            {
              t = elapsed / unit;
              elapsed -= t * unit;

              ticks -= t < ticks ? t : ticks;
            }
        }
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
 *   System Clock Frequency of RV32M1 is variable, the built-in
 *   up_mdelay doesn't work very well for this chip. A Timer
 *   Service facilitates the delay function to get better accuracy.
 *
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void up_mdelay(unsigned int milliseconds)
{
  uint32_t freq   = rv32m1_timersvc_freq();
  uint32_t period = rv32m1_timersvc_period();

  /* When Timer Service is up, and its frequency isn't less than 1KHz,
   * the granularity of Timer Service can provide 1 ms accuracy. In
   * other cases, sofware delay timer is preferred.
   */

  if (!rv32m1_timersvc_up() || freq < 1000u || !period)
    {
      up_sw_mdelay(milliseconds);
    }
  else
    {
      up_hw_delay(period, freq / 1000u, milliseconds);
    }
}

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Delay inline for the requested number of microseconds.
 *   System Clock Frequency of RV32M1 is variable, the built-in
 *   up_udelay doesn't work very well for this chip. A Timer
 *   Service facilitates the delay function to get better accuracy.
 *
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void up_udelay(useconds_t microseconds)
{
  uint32_t freq   = rv32m1_timersvc_freq();
  uint32_t period = rv32m1_timersvc_period();

  /* When Timer Service is up, and its frequency isn't less than 1MHz,
   * the granularity of Timer Service can provide 1 us accuracy. In
   * other cases, sofware delay timer is preferred.
   */

  if (!rv32m1_timersvc_up() || freq < 1000000u || !period)
    {
      up_sw_udelay(microseconds);
    }
  else
    {
      up_hw_delay(period, freq / 1000000u, microseconds);
    }
}
