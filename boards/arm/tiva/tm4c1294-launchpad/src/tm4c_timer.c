/****************************************************************************
 * boards/arm/tiva/tm4c1294-launchpad/src/tm4c_timer.c
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
 * Name: tiva_timer_configure
 *
 * Description:
 *   Configure the timer driver
 *
 ****************************************************************************/

int tiva_timer_configure(void)
{
  int ret;

  tmrinfo("Registering TIMER%d at %s\n",
          GPTM, CONFIG_TM4C1294_LAUNCHPAD_TIMER_DEVNAME);

  ret = tiva_timer_register(CONFIG_TM4C1294_LAUNCHPAD_TIMER_DEVNAME,
                            GPTM, ALTCLK);
  if (ret < 0)
    {
      tmrerr("ERROR: Failed to register timer driver: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_TM4C1294_LAUNCHPAD_TIMER */
