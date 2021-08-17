/****************************************************************************
 * boards/arm/stm32/cloudctrl/src/stm32_relays.c
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

#include <stdint.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <arch/board/board.h>

#include "cloudctrl.h"

#ifdef CONFIG_ARCH_RELAYS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RELAYS_MIN_RESET_TIME 5
#define RELAYS_RESET_MTIME 5
#define RELAYS_POWER_MTIME 50

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_relays_stat = 0;
static bool g_relays_init = false;

static const uint16_t g_relays[NUM_RELAYS] =
{
  GPIO_RELAYS_R00
#ifdef GPIO_RELAYS_R01
  , GPIO_RELAYS_R01
#endif
#ifdef GPIO_RELAYS_R02
  , GPIO_RELAYS_R02
#endif
#ifdef GPIO_RELAYS_R03
  , GPIO_RELAYS_R03
#endif
#ifdef GPIO_RELAYS_R04
  , GPIO_RELAYS_R04
#endif
#ifdef GPIO_RELAYS_R05
  , GPIO_RELAYS_R05
#endif
#ifdef GPIO_RELAYS_R06
  , GPIO_RELAYS_R06
#endif
#ifdef GPIO_RELAYS_R07
  , GPIO_RELAYS_R07
#endif
#ifdef GPIO_RELAYS_R08
  , GPIO_RELAYS_R08
#endif
#ifdef GPIO_RELAYS_R09
  , GPIO_RELAYS_R09
#endif
#ifdef GPIO_RELAYS_R10
  , GPIO_RELAYS_R10
#endif
#ifdef GPIO_RELAYS_R11
  , GPIO_RELAYS_R11
#endif
#ifdef GPIO_RELAYS_R12
  , GPIO_RELAYS_R12
#endif
#ifdef GPIO_RELAYS_R13
  , GPIO_RELAYS_R13
#endif
#ifdef GPIO_RELAYS_R14
  , GPIO_RELAYS_R14
#endif
#ifdef GPIO_RELAYS_R15
  , GPIO_RELAYS_R15
#endif
#ifdef GPIO_RELAYS_R16
  , GPIO_RELAYS_R16
#endif
#ifdef GPIO_RELAYS_R17
  , GPIO_RELAYS_R17
#endif
#ifdef GPIO_RELAYS_R18
  , GPIO_RELAYS_R18
#endif
#ifdef GPIO_RELAYS_R19
  , GPIO_RELAYS_R19
#endif
#ifdef GPIO_RELAYS_R20
  , GPIO_RELAYS_R20
#endif
#ifdef GPIO_RELAYS_R21
  , GPIO_RELAYS_R21
#endif
#ifdef GPIO_RELAYS_R22
  , GPIO_RELAYS_R22
#endif
#ifdef GPIO_RELAYS_R23
  , GPIO_RELAYS_R23
#endif
#ifdef GPIO_RELAYS_R24
  , GPIO_RELAYS_R24
#endif
#ifdef GPIO_RELAYS_R25
  , GPIO_RELAYS_R25
#endif
#ifdef GPIO_RELAYS_R26
  , GPIO_RELAYS_R26
#endif
#ifdef GPIO_RELAYS_R27
  , GPIO_RELAYS_R27
#endif
#ifdef GPIO_RELAYS_R28
  , GPIO_RELAYS_R28
#endif
#ifdef GPIO_RELAYS_R29
  , GPIO_RELAYS_R29
#endif
#ifdef GPIO_RELAYS_R30
  , GPIO_RELAYS_R30
#endif
#ifdef GPIO_RELAYS_R31
  , GPIO_RELAYS_R31
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_relaysinit(void)
{
  int i;

  if (g_relays_init)
    {
      return;
    }

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for some pins but NOT used in this file
   */

  for (i = 0; i < NUM_RELAYS; i++)
    {
      stm32_configgpio(g_relays[i]);
      stm32_gpiowrite(g_relays[i], false);
    }

  g_relays_init = true;
}

void relays_setstat(int relays, bool stat)
{
  if ((unsigned)relays < NUM_RELAYS)
    {
      stm32_gpiowrite(g_relays[relays], stat);
      if (!stat)
        {
          g_relays_stat &= ~(1 << relays);
        }
      else
        {
          g_relays_stat |= (1 << relays);
        }
    }
}

bool relays_getstat(int relays)
{
  if ((unsigned)relays < NUM_RELAYS)
    {
      return (g_relays_stat & (1 << relays)) != 0;
    }

  return false;
}

void relays_setstats(uint32_t relays_stat)
{
  int i;

  for (i = 0; i < NUM_RELAYS; i++)
    {
      relays_setstat(i, (relays_stat & (1 << i)) != 0);
    }
}

uint32_t relays_getstats(void)
{
  return (uint32_t)g_relays_stat;
}

void relays_onoff(int relays, uint32_t mdelay)
{
  if ((unsigned)relays < NUM_RELAYS)
    {
      if (mdelay > 0)
        {
          if (relays_getstat(relays))
            {
              relays_setstat(relays, false);
              nxsig_usleep(RELAYS_MIN_RESET_TIME * 1000 * 1000);
            }

          relays_setstat(relays, true);
          nxsig_usleep(mdelay * 100 * 1000);
          relays_setstat(relays, false);
        }
    }
}

void relays_onoffs(uint32_t relays_stat, uint32_t mdelay)
{
  int i;

  for (i = 0; i < NUM_RELAYS; i++)
    {
      relays_onoff(i, mdelay);
    }
}

void relays_resetmode(int relays)
{
  relays_onoff(relays, RELAYS_RESET_MTIME);
}

void relays_powermode(int relays)
{
  relays_onoff(relays, RELAYS_POWER_MTIME);
}

void relays_resetmodes(uint32_t relays_stat)
{
  relays_onoffs(relays_stat, RELAYS_RESET_MTIME);
}

void relays_powermodes(uint32_t relays_stat)
{
  relays_onoffs(relays_stat, RELAYS_POWER_MTIME);
}

#endif /* CONFIG_ARCH_BUTTONS */
